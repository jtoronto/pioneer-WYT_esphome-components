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
    components: [ pioneer ]

climate:
  - platform: pioneer
    name: My Heatpump
    beeper: false # Feedback beep when commands are sent
    display: true # Show temperature on the LED display

uart:
  tx_pin: # Hardware-dependent
  rx_pin: # Hardware-dependent
  baud_rate: 9600
  parity: EVEN
```

Thanks to squidpickles' work to reverse engineer the communication protocol:  
https://github.com/squidpickles/tuya-serial  
https://github.com/squidpickles/pioneer-uart