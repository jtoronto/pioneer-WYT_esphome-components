# ESPHome Component for Pioneer WYT (Diamante) Minisplit Heatpumps

Tested on a Diamante Ultra, may work with the original Diamante as well

USB dongle pinout:
|Standard USB Pin|Minisplit Usage|Wire Color|
| --- | --- | --- |
|GND|+5V|Black|
|D+|GND|Green|
|D-|TX|White|
|+5V|RX|Red|

The dongle design can be found here: https://github.com/mikesmitty/electronics-projects/tree/main/wyt-dongle

Example config entry:
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

Thanks to squidpickles' work to reverse engineer the communication protocol:  
https://github.com/squidpickles/tuya-serial  
https://github.com/squidpickles/pioneer-uart