# OFF Command Deadlock Fix & JS Automation Cleanup

## Summary

When the Pioneer WYT AC intermittently ignores an OFF command over UART, the
component's local `mode` stays `OFF` (set optimistically by `control()`) while
the AC keeps physically running. Because `query_state_()` only updates `mode`
when `state_.power` actually changes, and `power` is `true` for both the
running and not-running sides of this failure, the mode mismatch persists
indefinitely. Home Assistant shows `mode=off`, the JS automation sees
`acState='off'` and `action=off`, and the watchdog never re-fires because its
over-cooling detection requires `mode='cool'`. The AC runs uncontrolled.

This was confirmed at 04:35 UTC on 2026-06-07: OFF failed, mode stuck at OFF
in HA, AC kept cooling the room for ~4.5 hours until the user noticed.

The fix is a new `reconcile_mode_()` method in the ESPHome component that runs
after every successful poll once the command delay has expired, correcting
local `mode`/`action` to match what the AC is actually doing. The JS
automation's existing retry logic does the rest — when it sees the corrected
`mode='cool'`, it re-evaluates and sends another OFF command.

Separately, the JS automation was logging every `syncStandby` event 2–3 times.
That was a redundant call from `entity-state-changed`; `entity-updated` already
covers it. Removed.

Both fixes verified end-to-end on the live AC. The retry loop took 3 attempts
(~18 s) to successfully turn the AC off instead of deadlocking for hours.

## Background

The Pioneer WYT climate component (`components/pioneer/wyt_climate.cpp`) uses
a polling-then-reacting pattern:

1. ESPHome's `climate` integration calls `control()` when the user changes a
   setting (mode, temp, fan).
2. `control()` sets the local `mode`/`action`/`target_temperature`
   *optimistically* (this is what makes the UI feel instant), then calls
   `refresh()` → `send_command()` to push the change over UART to the AC.
3. A 5-second `uart_busy_` guard is set so subsequent polls don't fight the
   outgoing command.
4. After the guard expires, `update()` calls `query_state_()` which reads the
   AC's current state and calls `update_property_()` for each field.
5. `update_property_()` has a guard: it only writes if the value differs from
   what the AC reported.

The watchdog in HA is a JS Engine script (`ha-js-automations/lib/airconControl.js`)
that reads `acState` and `action` from the climate entity. If the room temp
drops below the setpoint minus hysteresis *and* `acState === 'cool'`, it sends
OFF and turns a standby `input_boolean` on.

## Investigation

### The deadlock

Used the HA REST API to pull `climate.living_room_mini_split_2` history on
2026-06-07. The state log shows:

```
04:35:36  → mode=off, action=off, power=false   ← command sent
04:35:36  → mode=off, action=off, power=true    ← AC actually still ON
04:35:38  → mode=off, action=off, power=true    ← AC continues running
... (4.5 hours of this)
09:04:14  → mode=off, action=off, power=false   ← user manually intervened
```

The second line is the key: `power` flipped back to `true` because the AC's
next poll response said so, but `mode`/`action` stayed at `off` because
`update_property_()` saw no change in the optimistic value.

In `query_state_()`:

```cpp
if (new_state.power != old_state.power) {
    this->mode = ... derived from new_state ...;
    ...
}
```

When the OFF command fails, the next poll response has `power=true` (AC still
running) and `mode=Cool` (or whatever it was before). But `old_state.power`
was already `true` from the previous poll (because the AC was already running),
so the `if` is false and the mode branch is skipped entirely. The optimistic
`mode=off` from `control()` is never corrected.

The automation sees `acState='off'` and `action='off'`. Its over-cooling
detector requires `acState === 'cool'`, so it never fires. Even if the user
manually adjusted the setpoint to be warmer, the "resume cooling" condition
(`tempDifference > 0`) never triggers because the room keeps getting colder
(AC is still running) but `mode=off` in HA. Total deadlock.

### Why the JS retry loop matters

The JS automation (`airconControl.js`) has a 15-second evaluation interval and
a 10-second "act" delay. If `reconcile_mode_()` publishes a corrected
`mode=cool`, the next JS evaluation sees the real AC state, triggers the
over-cooling detection, and sends another OFF command. So the ESPHome fix
turns a permanent deadlock into a transient failure that the existing JS
retry logic handles.

## Fixes

### 1. ESPHome: `reconcile_mode_()` in `components/pioneer/wyt_climate.cpp`

After every successful poll (after the 5-second command delay expires),
compare the local `mode` (what we commanded optimistically) against the AC's
actual reported mode (from `state_`). If they differ, correct and publish.

In `wyt_climate.cpp`, after `query_state_()` succeeds in `update()`:

```cpp
this->reconcile_mode_();
```

New method:

```cpp
void WytClimate::reconcile_mode_() {
  climate::ClimateMode actual_mode = this->get_mode();
  if (this->mode != actual_mode) {
    ESP_LOGW(TAG, "Mode mismatch: local=%s, AC=%s. Correcting to AC state.",
             climate::climate_mode_to_string(this->mode),
             climate::climate_mode_to_string(actual_mode));
    this->mode = actual_mode;
    this->action = this->get_action();
    this->publish_state();
  }
}
```

Declaration added to `wyt_climate.h` next to `query_state_()`.

This runs every poll cycle (currently every 2 s in ESPHome, throttled by the
5 s `uart_busy_` guard). When the AC ignores a command, the next poll after
the guard expires finds a mismatch, publishes the correction, and the JS
automation reacts.

### 2. JS: remove redundant `syncStandby()` from `entity-state-changed`

`homeassistant-jsengine` fires **both** `entity-state-changed` (when the state
attribute changes) **and** `entity-updated` (when any attribute changes) for
the same logical event. The previous handler called `syncStandby()` in both,
producing 2–3 log lines per AC state change.

`entity-updated` covers all state changes plus other attribute updates (which
is when we actually need `syncStandby`, since a temperature change without a
mode change wouldn't fire `entity-state-changed`). The
`entity-state-changed` call was strictly redundant.

In `ha-js-automations/src/airconControl.ts`:

```ts
/* 2️⃣ state string changed */
'entity-state-changed'(id: string) {
  if (tracked.has(id)) safeEvaluate();
  if (id === ENT.HUMIDITY) safeEvaluate();
},
```

The `tracked` check (SETPOINT/STANDBY) and the humidity check stay — those
still need to react to pure state changes. `syncStandby()` for AC is gone.

Source TypeScript is rebuilt via `npm run build` — the JS Engine loads
`lib/airconControl.js` from `/opt/jsengine/`, so rebuild + redeploy to apply.

## Verification

### End-to-end retry loop (2026-06-07, 08:16 EDT)

Captured the live ESPHome log
(`logs/7-24-living-room-mini-split-logs-2.txt`). Three OFF attempts after the
automation was triggered by a slider change:

```
08:16:03.279  Control called, mode=OFF
08:16:03.325  UART command sent, uart_busy_=TRUE
08:16:08.329  uart_busy_ timeout expired, clearing
08:16:09.159  Mode mismatch: local=OFF, AC=COOL. Correcting to AC state.
08:16:09.261  Mode: COOL                                       ← JS sees this
08:16:10.312  Control called, mode=OFF                         ← JS retries
08:16:15.371  uart_busy_ timeout expired
08:16:17.165  Mode mismatch: local=OFF, AC=COOL. Correcting to AC state.
08:16:21.225  State changed at byte 7: 0x31 → 0x20             ← power bit cleared
08:16:23.262  Power Usage: 0 W                                 ← AC off
```

3 attempts, ~18 s total, AC confirmed off via power draw. No deadlock.

### Double-logging fix

Re-ran the JS automation after deploying the rebuild. A single AC mode
change now produces one `syncStandby` log line per event instead of 2–3.
Verified in `logs/6-7-26-js-automation-log.txt`.

## Files changed

| Repo | File | Change |
|---|---|---|
| `pioneer-WYT_esphome-components` | `components/pioneer/wyt_climate.cpp` | Add `reconcile_mode_()` call after `query_state_()` + method body |
| `pioneer-WYT_esphome-components` | `components/pioneer/wyt_climate.h` | Declare `reconcile_mode_()` |
| `ha-js-automations` | `src/airconControl.ts` | Remove redundant `syncStandby()` from `entity-state-changed` |
| `ha-js-automations` | `lib/airconControl.js` | Rebuilt from TS source |

## Things deliberately NOT changed

- **`update_property_()` mode branch**: kept as-is. Adding a mode-only branch
  would tightly couple `update_property_()` to the optimistic-write path and
  reintroduce the bug for *every* subsequent field, not just mode. The
  dedicated `reconcile_mode_()` keeps the logic local to the failure mode.
- **`uart_busy_` guard duration**: not touched. The 5 s window is the
  minimum needed for the AC to process a command, and the retry naturally
  re-evaluates after the guard expires.
- **JS evaluation interval**: the 15 s interval is already long enough that
  the natural retry cadence comfortably covers the ESPHome 2 s poll + 5 s
  guard + reconcile cycle.
- **`set_mode()` / `set_fan_mode_()` paths in ESPHome**: not touched. The bug
  is mode reconciliation, not the command-send path. Other command paths
  don't have the same state-writeback issue because their effects are visible
  on the next poll regardless of `power` state.

## How to repeat the deadlock reproduction

If you want to verify the fix again or test edge cases:

1. Flash the updated component onto the mini-split ESPHome device.
2. Start the JS automation.
3. Set `target_temperature` comfortably below room temp, mode = Cool.
4. Wait for the AC to start cooling (power draw should rise).
5. From HA, send `climate.turn_off()` on `climate.living_room_mini_split_2`.
6. Watch the ESPHome log. The first OFF may or may not take depending on
   UART timing; if it fails, you should see:
   - `uart_busy_` timeout
   - `Mode mismatch: local=OFF, AC=COOL. Correcting to AC state.`
   - JS automation re-sends OFF within ~15 s
   - Retry succeeds within 3 attempts

If the AC actually receives the first OFF command, you won't see the
mismatch log — that's the happy path and what we want.
