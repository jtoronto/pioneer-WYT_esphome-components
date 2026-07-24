# Learnings - Fix ESPHome Warnings

## Task 1 & 2: synchronous=True

- The `synchronous=True` parameter goes on the `automation.register_action()` call inside the wrapper function, NOT on individual `@register_action` decorator calls
- All C++ `play()` methods in this component are synchronous (call methods and return, no deferred callbacks)
- ESPHome 2026.4.0 introduced this parameter via PR #14606

## Task 3: docker-compose.yml version

- Modern Docker Compose ignores the `version:` attribute and warns about it
- Simple one-line deletion — no other changes needed

## Task 4: custom_components cleanup

- The `custom_components/pioneer/` directory was a leftover from before the migration to `external_components`
- The YAML config already used `external_components` (line 1-4)
- No other YAML configs referenced the pioneer component
- Safe to delete the entire directory

## Task 5: ESP32 advanced settings

- Device boot log confirmed: `ESP32 Chip: ESP32 rev3.0` — so `minimum_chip_revision: "3.0"` is correct
- Device boot log confirmed: `Bootloader supports SRAM1 as IRAM (+40KB)` — so `sram1_as_iram: true` is safe
- **CRITICAL GOTCHA**: Subagent broke YAML indentation (added 2 extra spaces). Always verify YAML structure after edits.
- These settings reduce binary size and provide +40KB IRAM

## Task 6: Verification

- The ESPHome Docker container pulls components from GitHub (`github://jtoronto/pioneer-WYT_esphome-components@improvements`), not the local directory
- Changes must be pushed to GitHub before they take effect
- Device must be reflashed with new config for ESP32 advanced settings to apply
- The `custom_components` and `docker-compose.yml version` warnings were eliminated immediately

## Key Insight

- ESPHome components loaded via `external_components` with a GitHub source are fetched from the remote repository, not the local filesystem
- Local changes must be pushed to GitHub before they're used by the ESPHome Docker container
- The `refresh: 0s` setting means it refreshes every time, but still from the remote source
