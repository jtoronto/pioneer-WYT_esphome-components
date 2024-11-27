# ESPHome Component for Pioneer WYT (Diamante) Minisplit Heatpumps

Tested on a Diamante Ultra, may work with the original Diamante as well

USB dongle pinout:
|Standard USB Pin|Minisplit Usage|Wire Color|
| --- | --- | --- |
|GND|+5V|Black|
|D+|GND|Green|
|D-|TX|White|
|+5V|RX|Red|

Example config entry:
```yaml
climate:
  - platform: pioneer
    name: My Heatpump
    beeper: false # Feedback beep when commands are sent
    display: true # Show temperature on the LED display
```