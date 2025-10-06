# Pioneer WYT ESPHome Component

## Project Overview

This project provides a custom ESPHome component for controlling and monitoring Pioneer WYT (Diamante) minisplit heat pumps. It enables integration of these AC units into Home Assistant via ESPHome, utilizing both UART communication for primary control and an optional IR remote transmitter for features like "I Feel" or "Follow Me" mode.

The component is designed to work with the unit's internal microcontroller (MCU) via a custom serial protocol, which has been reverse-engineered. It exposes various climate controls (mode, temperature, fan speed, swing mode) and sensors (defrost status, indoor/outdoor fan speed, outdoor temperature, power usage) to ESPHome.

## Building and Running

This is an ESPHome custom component. To use it, you need to include it as an `external_component` in your ESPHome configuration (`.yaml` file).

**Example ESPHome Configuration:**

```yaml
external_components:
  - source: github://mikesmitty/esphome-components@main
    components: [ pioneer, remote_base ]

climate:
  - platform: pioneer
    name: My Heatpump
    beeper: false # Feedback beep when commands are sent
    display: true # Show temperature on the LED display
    # Optional sensors
    defrost_status:
      name: Defrosting
    indoor_fan_speed:
      name: Indoor Fan Speed
    outdoor_fan_speed:
      name: Outdoor Fan Speed
    outdoor_temperature:
      name: Outdoor Temperature
    power_usage:
      name: Power Usage

uart:
  tx_pin: # Hardware-dependent
  rx_pin: # Hardware-dependent
  baud_rate: 9600
  parity: EVEN

# Optional I Feel/Follow Me support via IR transmitter (remote temperature sensor support)
# This feature is still in an alpha-ish state at the moment. The IR protocol as used by the
# remote has been fully decoded, but I'm not certain the feature is activated in the unit.
# Even with the original remote it seems to have no real effect. The temp indicated doesn't
# change what's shown in the pioneer app and the unit doesn't react aside from a beep
# acknowledging the IR command was received.
remote_transmitter:
  pin: # Hardware-dependent
  carrier_duty_percent: 50%

sensor:
  - platform: ...
    id: my_remote_temp_sensor
    on_value:
      then:
        - pioneer_wyt.remote_temp:
            # Requires conversion from fahrenheit to celsius (if needed) at the moment
            temperature: !lambda "return fahrenheit_to_celsius(x);"
            # beeper: false
```

**Steps to use:**

1.  **Add to ESPHome Configuration:** Copy the `external_components` and `climate` sections into your ESPHome YAML file.
2.  **Configure UART:** Specify the `tx_pin`, `rx_pin`, `baud_rate`, and `parity` for your UART connection to the AC unit.
3.  **Optional IR Transmitter:** If you want to use the "I Feel" / "Follow Me" feature, configure the `remote_transmitter` with the appropriate `pin`.
4.  **Compile and Upload:** Use the ESPHome dashboard or CLI to compile and upload the firmware to your ESP device.

## Development Conventions

*   **Language:** The component is primarily implemented in C++ (`wyt_climate.h`, `wyt_climate.cpp`) with a Python component definition (`climate.py`) for ESPHome's code generation.
*   **Communication Protocol:** A custom UART-based protocol is used to communicate with the Pioneer WYT MCU. This involves sending `SetCommand` structures and parsing `StateResponse` structures, with checksum validation.
*   **IR Protocol:** An optional IR remote protocol is implemented for specific features, leveraging `wyt_remote.h` and `esphome/components/remote_base/pioneer_wyt_protocol.h`.
*   **ESPHome Integration:** The component adheres to ESPHome's component development guidelines, utilizing `climate::Climate`, `PollingComponent`, and `uart::UARTDevice` base classes.
*   **Code Structure:** The C++ code defines various enums and unions to represent the byte-level communication with the AC unit, making the protocol more readable and manageable.
*   **Logging:** ESPHome's logging (`ESP_LOGD`, `ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE`) is used for debugging and status reporting.
