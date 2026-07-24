# Pioneer WYT IR Remote Fix

## Summary

The i-feel / follow-me IR feature on the Pioneer WYT ESPHome component
(`components/pioneer/`) shipped with two latent bugs that caused the AC to
silently ignore every IR transmission. Both were undetected because the AC
acknowledges IR with a beep regardless of validity, making the failure mode
look like "the feature is unimplemented on this hardware" rather than "we're
transmitting garbage."

Flashing the fixes below made i-feel work end-to-end. Verified by setting the
remote-temp to 85°F and watching the AC ramp the indoor fan from idle to 105+
as the compressor ramped up to chase the artificially-high "room temperature."

## Background

Reverse engineering by Michael Smith (`@mikesmitty`) had already produced a
working sigrok capture pipeline (`read-sigrok-ir.py`) and decoded the Pioneer
WYT IR protocol in detail. The captured `.sr` files in
`components/pioneer/ir-signals/` represent the byte-level output of the
factory remote for power-on, mode-switch, fan-speed, timer, sleep, eco, turbo,
display-toggle, mute, i-feel-on, i-feel-on2, i-feel-update, i-feel-update2,
follow-on, follow-off, and several setpoints.

The README caveated the i-feel feature:

> This feature is still in an alpha-ish state at the moment. The IR protocol as
> used by the remote has been fully decoded, but I'm not certain the feature is
> activated in the unit. Even with the original remote it seems to have no real
> effect. The temp indicated doesn't change what's shown in the pioneer app and
> the unit doesn't react aside from a beep acknowledging the IR command was
> received.

That last sentence — "beep acknowledging the IR command was received" — was
the only diagnostic visible to a user without a logic analyzer. It looks like
"feature unimplemented" but is actually "AC rejects the command silently."

## Investigation

Captured the factory remote with a TSOP4838 receiver on a spare ESPHome
device and dumped Pronto codes. Decoded both Pronto captures (FAN + GENERAL
messages) with a small Python helper and compared against the existing
`*.sr.txt` references in the repo.

### What was NOT the bug

- **Timing constants** (`components/remote_base/pioneer_wyt_protocol.cpp:13-17`):
  3100/1650/500/1100/350 µs. These looked suspicious in git history (commit
  `794fd1f` "update protocol timing and frequency" switched from measured
  3075/1600/486/1100/333 to the tcl112-derived values), but the measured
  values from the user's TSOP4838 capture confirmed the tcl112 values are
  within ~5% of real Pioneer transmissions. The Pioneer WYT receiver is
  tolerant of that. Leave alone.
- **Most byte fields**: correctly computed from `state_` for both FAN and
  GENERAL commands.

### Bug 1 — FAN byte 8 was always zero

`get_fan_command_from_state()` never set `command.unknown5`. This is the field
at offset 8 of the 14-byte FAN message. Every single reference capture in
`components/pioneer/ir-signals/` has this byte set to `0xC0` as a constant —
regardless of mode, fan speed, swing state, temperature setpoint, or
follow-me status:

| Capture                          | FAN byte 8 |
| -------------------------------- | ---------- |
| pioneer-power-on.sr.txt          | `0xC0`     |
| pioneer-display-on.sr.txt        | `0xC0`     |
| pioneer-display-off.sr.txt       | `0xC0`     |
| pioneer-mode-switch-cool.sr.txt  | `0xC0`     |
| pioneer-mode-switch-heat.sr.txt  | `0xC0`     |
| pioneer-fan-auto.sr.txt          | `0xC0`     |
| pioneer-fan-high.sr.txt          | `0xC0`     |
| pioneer-i-feel-on.sr.txt         | `0xC0`     |
| pioneer-follow-on.sr.txt         | `0xC0`     |
| pioneer-follow-off.sr.txt        | `0xC0`     |

The AC treats anything other than `0xC0` here as a malformed FAN message and
ignores the entire transmission. The structure in `components/pioneer/wyt_remote.h`
even names the field `unknown5` because the author never figured out it
wasn't state-dependent.

### Bug 2 — GENERAL byte 6 follow_me bit was never set

`do_remote_temp()` constructed the GENERAL command from the current AC state
via `get_general_command_from_state()`, then overrode `remote_temp` and
`beeper`. It never touched `follow_me`. So the GENERAL message had byte 6
with `follow_me = 0`, regardless of what the user asked for. The AC saw a
"regular climate command with a non-zero `remote_temp` byte" and ignored the
remote temp for control purposes — same "beep and ignore" failure mode as
Bug 1.

The `i-feel` references (`pioneer-i-feel-on.sr.txt` and friends) all have
byte 6 = `0x81` — `follow_me = 1`. The `follow-off` reference has byte 6 =
`0x01` — `follow_me = 0`.

## Fixes

### 1. `components/pioneer/wyt_climate.cpp` — set the FAN byte 8 constant

In `get_fan_command_from_state()`, before `return command;`:

```cpp
// Every FAN message captured from the factory remote has byte 8 = 0xC0 as a
// constant. The Pioneer AC rejects FAN messages with any other value here,
// which is why the IR feature never worked.
command.unknown5 = 0xC0;
```

### 2. `components/pioneer/wyt_climate.cpp` — set follow_me in do_remote_temp

In `do_remote_temp()`, after `general_command.beeper = ...`:

```cpp
// Pioneer remote sets the follow_me flag (byte 6, bit 7) whenever it transmits
// a remote temp reading. Without this, the AC ignores the remote_temp byte
// and only beeps in acknowledgement. temp_c == 0 means "turn follow-me off",
// matching pioneer-follow-off.sr.txt.
general_command.follow_me = (general_command.remote_temp != 0);
```

This makes `pioneer_wyt.remote_temp: 0` (or any falsy value) automatically
send the follow-off form, matching `pioneer-follow-off.sr.txt`.

### 3. `pioneer_wyt.follow_me_off` action

A clean way to exit i-feel without needing a sensor value of 0.

- `components/pioneer/ac_automations.h` — new `FollowMeOffAction` class
- `components/pioneer/climate.py` — register `pioneer_wyt.follow_me_off`
- `components/pioneer/wyt_climate.h` — `do_follow_me_off()` inline wrapper
  that calls `do_remote_temp(0.0f, false)`

## How to verify without a logic analyzer

The fan-speed sensor exposed by the component makes verification cheap. From
the existing Home Assistant automation that triggers
`pioneer_wyt.remote_temp`:

1. With i-feel enabled and `target_temperature` set comfortably below the room
   temp, the compressor should idle (low power, low fan speed).
2. Inject an extreme i-feel value (e.g. 85°F / 29°C). Within ~30 s the
   compressor should ramp hard and the indoor fan should climb significantly.
3. Drop i-feel back to a reasonable value (or call `pioneer_wyt.follow_me_off`).
   The compressor should back off.

The captured live log at `7-24-living-room-mini-split-logs-2.txt` shows the
indoor fan climbing 89 → 91 → 93 → 95 → 97 → 99 → 101 → 103 → 105 within
~16 s of injecting 29.4°C, then dropping back when i-feel was set to 24.2°C.

## Things deliberately NOT changed

- **Timing constants.** The tcl112-derived values match real Pioneer
  transmissions closely enough. Touching them risks introducing new bugs.
- **Mode field on GENERAL.** The references all show `mode = Heat (0x01)` for
  i-feel, but the AC accepts i-feel in any mode — verified by leaving
  `mode = Cool` in the user's setup and observing the i-feel feature work.
  Forcing mode would silently override the user's chosen mode.
- **`setpoint_half_digit` / `left_right_flow` / `up_down_flow` byte
  differences.** All state-dependent and correctly derived from `state_`.

## How to repeat the Pronto decode

If anyone needs to debug a future IR protocol change, the workflow that worked:

1. Capture the factory remote with a TSOP4838 (or compatible 38 kHz IR
   receiver) on any ESPHome device. Configure the receiving device with
   `remote_receiver:` to log Pronto codes (`dump_digit_` / `dump_number_`).
2. Decode the Pronto hex into Pioneer WYT bytes. Pronto values are in units
   of `1000000 / carrier_freq_hz` µs (≈26.3 µs at 38 kHz). The Pioneer WYT
   structure is header (1 mark + 1 space ≈ 3100 + 1600 µs) + initial bit
   mark (≈500 µs) + 112 bits × (bit space + bit mark). Bit 1 space ≈ 1100 µs,
   bit 0 space ≈ 350 µs.
3. Pack the resulting 112 bits into 14 bytes LSB-first. Verify checksum =
   `(0x0F if msg_type == 0x02 else 0x00) + sum(bytes[0..12]) & 0xFF`.
4. Diff the bytes against the existing `components/pioneer/ir-signals/*.sr.txt`
   references. Any byte that varies across captures is state-dependent; any
   byte that's constant across all captures is protocol-mandatory.
