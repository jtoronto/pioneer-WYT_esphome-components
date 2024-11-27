#pragma once

// #include "wyt_command.h"
// #include "wyt_response.h"

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/hal.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace pioneer {
namespace wyt {

static const uint8_t WYT_QUERY_COMMAND[8] = {0xBB, 0x00, 0x01, 0x04, 0x02, 0x01, 0x00, 0xBD};

static const uint8_t WYT_HEADER_SIZE = 5;
static const uint8_t WYT_QUERY_COMMAND_SIZE = 8;
static const uint8_t WYT_QUERY_RESPONSE_SIZE = 61;
static const uint8_t WYT_STATE_COMMAND_SIZE = 35;

// FIXME: Cleanup
// Cool: bb 00 01 03 1d 00 00 64 03 55 00 04 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 92
// Heat: bb 00 01 03 1d 00 00 64 01 57 38 0c 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 a2
// Dry:  bb 00 01 03 1d 00 00 64 02 55 02 04 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 91
// Fan:  bb 00 01 03 1d 00 00 64 07 55 00 00 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 92
// Auto: bb 00 01 03 1d 00 00 64 08 55 00 00 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 9d
// Heat: bb 00 01 03 1d 00 00 64 01 57 38 08 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 a6
// Temp change
// ->76: bb 00 01 03 1d 00 00 64 01 57 38 0c 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 a2
// ->77: bb 00 01 03 1d 00 00 64 01 56 38 08 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 a7
// ->78: bb 00 01 03 1d 00 00 64 01 56 38 0c 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 a3
// ->79: bb 00 01 03 1d 00 00 64 01 55 38 08 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 a4
// ->80: bb 00 01 03 1d 00 00 64 01 55 38 0c 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 a0
// Disable up/down flow
//       bb 00 01 03 1d 00 00 64 01 57 00 0c 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 88 92
// Enable up/down flow
//       bb 00 01 03 1d 00 00 64 01 57 38 0c 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 a2
// Disable left/right flow
//       bb 00 01 03 1d 00 00 64 01 57 38 04 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 80 a2
// Enable left/right flow
//       bb 00 01 03 1d 00 00 64 01 57 38 0c 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 a2
// Enable 8* Heater (Freeze Protection)
//       bb 00 01 03 1d 00 00 64 01 5d b8 08 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 2c
// Disable 8* Heater (Freeze Protection)
//       bb 00 01 04 02 01 00 bd bb 00 01 03 1d 00 00 64 01 5d 38 08 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
//       00 00 00 08 88 ac
//       bb 00 01 03 1d 00 00 64 01 5d 38 08 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 ac
// Enable beeper
//       bb 00 01 03 1d 00 00 64 01 5d 38 08 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 ac
// Disable beeper
//       bb 00 01 03 1d 00 00 44 01 5d 38 08 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 8c
// Enable display
//       bb 00 01 03 1d 00 00 44 01 57 38 0c 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 82
// Disable display
//       bb 00 01 03 1d 00 00 04 01 57 38 0c 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 c2
// Fan - turbo
//       bb 00 01 03 1d 00 00 44 41 57 3d 0c 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 c7
// Fan - auto
//       bb 00 01 03 1d 00 00 44 01 57 38 0c 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 88 82
// Fan - mid-low
//       bb 00 01 03 1d 00 00 24 01 56 3e 08 80 00 00 00 00 00 00 00 8c 3a ff 3f d1 1d 4a 93 ab 08 00 00 08 88 21
// Fan - mute
//       bb 00 01 03 1d 00 00 24 81 56 3a 08 80 00 00 00 00 00 00 00 8c 3a ff 3f d1 1d 4a 93 ab 08 00 00 08 88 a5
// GEN mode - enable LV1
//       bb 00 01 03 1d 00 00 6c 01 56 38 0a 80 00 ec 7c 00 00 01 00 0a 00 00 00 25 7e d5 24 3a 23 00 00 08 88 85
// GEN mode - enable LV2
//       bb 00 01 03 1d 00 00 6c 01 56 38 0a 80 00 ec 7c 00 00 02 00 0a 00 00 00 25 7e d5 24 3a 23 00 00 08 88 86
// GEN mode - enable LV3
//       bb 00 01 03 1d 00 00 6c 01 56 38 0a 80 00 ec 7b 00 00 03 00 0a 00 00 00 25 7e d5 24 3a 23 00 00 08 88 80
// GEN mode - disable                                          ## Here
//       bb 00 01 03 1d 00 00 6c 01 56 38 0a 80 00 ec 7b 00 00 00 00 0a 00 00 00 25 7e d5 24 3a 23 00 00 08 88 83

enum class Source : uint8_t {
  Controller = 0x00,
  Appliance = 0x01,
};

enum class Dest : uint8_t {
  Controller = 0x00,
  Appliance = 0x01,
};

enum class Command : uint8_t {
  Set = 0x03,
  Get = 0x04,
};

enum class Mode : uint8_t {
  Cool = 0x01,
  Fan = 0x02,
  Dehumidify = 0x03,
  Heat = 0x04,
  Auto = 0x05,
};

enum class CmdMode : uint8_t {
  Off = 0x00,
  Heat = 0x01,
  Dehumidify = 0x02,
  Cool = 0x03,
  Fan = 0x07,
  Auto = 0x08,
};

enum class FanSpeed : uint8_t {
  Auto = 0x00,
  Low = 0x01,
  MediumLow = 0x04,
  Medium = 0x02,
  MediumHigh = 0x05,
  High = 0x03,
};

enum class CmdFanSpeed : uint8_t {
  Auto = 0x00,
  Low = 0x02,
  MediumLow = 0x06,
  Medium = 0x03,
  MediumHigh = 0x07,
  High = 0x05,
};

enum class SleepMode : uint8_t {
  Off = 0x00,
  Standard = 0x01,
  Elderly = 0x02,
  Child = 0x03,
};

enum class UpDownFlow : uint8_t {
  Auto = 0x00,
  TopFix = 0x01,
  UpperFix = 0x02,
  MiddleFix = 0x03,
  LowerFix = 0x04,
  BottomFix = 0x05,
  UpDownFlow = 0x08,
  UpFlow = 0x10,
  DownFlow = 0x18,
};

enum class LeftRightFlow : uint8_t {
  Auto = 0x00,
  LeftFix = 0x01,
  MiddleLeftFix = 0x02,
  MiddleFix = 0x03,
  MiddleRightFix = 0x04,
  RightFix = 0x05,
  LeftRightFlow = 0x08,
  LeftFlow = 0x10,
  MiddleFlow = 0x18,
  RightFlow = 0x20,
};

enum class IndoorFanSpeed : uint8_t {
  Off = 0x00,
  Low = 0x3c,
  Medium = 0x55,
  High = 0x62,
};

enum class OutdoorStatus : uint8_t {
  Idle = 0x00,
  Running = 0x0a,
};

/** All commands sent to the WYT MCU begin with this header */
union Header {
  struct {
    // 00
    uint8_t magic;
    // 01
    Source source;
    // 02
    Dest dest;
    // 03
    Command command;
    // 04
    uint8_t length;
  } __attribute__((packed));
  uint8_t bytes[WYT_HEADER_SIZE];
};

/** A request to update the MCU state to the one in this object. */
typedef union {
  struct {
    // 00..04
    Header header;
    // 05..06
    uint8_t unknown1[2];
    // 07
    uint8_t unknown3 : 2;
    bool power : 1;
    bool unknown2 : 2;
    bool beeper : 1;
    bool display : 1;
    bool eco : 1;
    // 08
    CmdMode mode : 4;
    bool health : 1;
    bool unknown4 : 1;
    bool strong : 1;
    bool mute : 1;
    // 09
    uint8_t set_temperature_whole;
    // 10
    CmdFanSpeed fan_speed : 3;
    uint8_t vertical_flow : 3;
    bool unknown5 : 1;
    bool freeze_protection : 1;
    // 11
    uint8_t unknown7 : 2;
    bool set_temperature_half : 1;
    uint8_t unknown6 : 5;
    // 12..17
    uint8_t unknown8[6];
    // 18 Reduced output for use with a generator (0: 100%, 1: 30%, 2: 50%, 3: 80%)
    uint8_t gen_mode : 2;
    uint8_t unknown9 : 6;
    // 19
    SleepMode sleep;
    // 20..31
    uint8_t unknown10[12];
    // 32
    UpDownFlow up_down_flow;
    // 33
    LeftRightFlow left_right_flow;
    // 34
    uint8_t checksum;
  } __attribute__((packed));
  uint8_t bytes[WYT_STATE_COMMAND_SIZE];
} SetCommand;

// Holds a response from the WYT MCU describing its current operating state.
typedef union {
  struct {
    // 00
    uint8_t magic;
    // 01
    Source source;
    // 02
    Dest dest;
    // 03
    Command command;
    // 04
    uint8_t command_length;
    // 05..06
    uint8_t unknown1[2];
    // 07
    Mode mode : 3;
    bool unknown2 : 1;
    bool power : 1;
    bool display : 1;
    bool eco : 1;
    bool strong : 1;
    // 08
    uint8_t set_temperature_whole : 4;
    FanSpeed fan_speed : 3;
    bool unknown3 : 1;
    // 09
    bool set_temperature_half : 1;
    bool unknown7 : 1;
    bool health : 1;
    uint8_t unknown6 : 5;
    // 10
    uint8_t unknown10 : 5;
    bool horizontal_flow : 1;
    bool vertical_flow : 1;
    bool unknown9 : 1;
    // 11..16
    uint8_t unknown11[6];
    // 17
    uint8_t indoor_temp_base;
    // 18
    uint8_t unknown12;
    // 19
    SleepMode sleep : 2;
    uint8_t unknown13 : 5;
    bool four_way_valve_on : 1;
    // 20..29
    uint8_t unknown14[10];
    // 30
    uint8_t indoor_heat_exchanger_temp;
    // 31
    uint8_t unknown15;
    // 32
    uint8_t unknown16 : 7;
    bool freeze_protection : 1;
    // 33
    uint8_t unknown17 : 7;
    bool mute : 1;
    // 34
    IndoorFanSpeed indoor_fan_speed;
    // 35
    uint8_t outdoor_temp;
    // 36
    uint8_t condenser_coil_temp;
    // 37
    uint8_t compressor_discharge_temp;
    // 38
    uint8_t compressor_frequency;
    // 39
    uint8_t outdoor_fan_speed;
    // 40
    OutdoorStatus outdoor_unit_status : 4;
    uint8_t unknown19 : 2;
    bool heat_mode : 1;
    bool unknown18 : 1;
    // 41..44
    uint8_t unknown20[4];
    // 45
    uint8_t supply_voltage;
    // 46
    uint8_t current_used_amps;
    // 47
    uint8_t gen_mode;
    // 48..50
    uint8_t unknown21[3];
    // 51
    UpDownFlow up_down_flow;
    // 52
    LeftRightFlow left_right_flow;
    // 53..60
    uint8_t unknown22[8];
  } __attribute__((packed));
  uint8_t bytes[WYT_QUERY_RESPONSE_SIZE];
} Response;

class WytClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;

  // Call triggers based on updated climate states (modes/actions)
  void refresh();
  void validate_target_temperature();

  climate::ClimateAction get_action();
  optional<std::string> get_custom_fan_mode();
  optional<climate::ClimateFanMode> get_fan_mode();
  climate::ClimateMode get_mode();
  climate::ClimateSwingMode get_swing_mode();
  float get_setpoint();
  float get_temperature();

  void set_beeper(bool enable_beeper) { this->enable_beeper_ = enable_beeper; }
  void set_display(bool enable_display) { this->enable_display_ = enable_display; }

 protected:
  // The current state of the climate device
  uint8_t raw_state_[WYT_QUERY_RESPONSE_SIZE];
  wyt::Response state_;
  bool enable_beeper_{false};
  bool enable_display_{true};

  // The new command to send to the WYT MCU
  SetCommand command;

  // climate::ClimateAction prev_action_; // FIXME: Implement this
  optional<std::string> prev_custom_fan_mode_;
  optional<climate::ClimateFanMode> prev_fan_mode_;
  climate::ClimateMode prev_mode_;
  climate::ClimateSwingMode prev_swing_mode_;
  float prev_current_temperature_ = NAN;
  float prev_target_temperature_ = NAN;

  // Override control to change settings of the climate device.
  void control(const climate::ClimateCall &call) override;

  // Return the traits of this climate device.
  climate::ClimateTraits traits() override;

  // Switch the climate device to the given climate action.
  void switch_to_action_(climate::ClimateAction action, bool publish_state = true);
  // void switch_to_supplemental_action_(climate::ClimateAction action);  // FIXME: Cleanup?
  // void trigger_supplemental_action_();                                 // FIXME: Cleanup?

  // Switch the climate device to the given climate fan mode.
  void switch_to_fan_mode_(climate::ClimateFanMode fan_mode, bool publish_state = true);
  void switch_to_custom_fan_mode_(std::string custom_fan_mode, bool publish_state = true);

  // Switch the climate device to the given climate mode.
  void switch_to_mode_(climate::ClimateMode mode, bool publish_state = true);

  // Switch the climate device to the given climate swing mode.
  void switch_to_swing_mode_(climate::ClimateSwingMode swing_mode, bool publish_state = true);

  // Check if the temperature change trigger should be called.
  void switch_to_setpoint_temperature_();
  void set_temperature_(SetCommand &command, const float temp_c);

  // Get the current state of the climate device
  bool query_state_(bool read_only = false);

  Response response_from_bytes(const uint8_t buffer[WYT_QUERY_RESPONSE_SIZE]);
  uint8_t response_checksum(const uint8_t buffer[WYT_QUERY_RESPONSE_SIZE]);

  // Checksum message data with XOR
  uint8_t checksum(const SetCommand &command);
  Header new_header(const Source &source, const Dest &dest, const Command &command, const uint8_t size);
  SetCommand command_from_bytes(const uint8_t buffer[WYT_STATE_COMMAND_SIZE]);
  SetCommand command_from_response(const Response &response);

  // Send a command to the WYT MCU
  void send_command(SetCommand &command);

  // The sensor used for getting the current temperature
  sensor::Sensor *sensor_{nullptr};

  /* FIXME: Implement or cleanup
  // The set of standard preset configurations this thermostat supports (Eg. AWAY, ECO, etc)
  std::map<climate::ClimatePreset, WytClimateTargetTempConfig> preset_config_{};
  // The set of custom preset configurations this thermostat supports (eg. "My Custom Preset")
  std::map<std::string, WytClimateTargetTempConfig> custom_preset_config_{};

  // Default standard preset to use on start up
  climate::ClimatePreset default_preset_{};
  // Default custom preset to use on start up
  std::string default_custom_preset_{};

  // If set to DEFAULT_PRESET then the default preset is always used. When MEMORY prior
  // state will attempt to be restored if possible
  thermostat::OnBootRestoreFrom on_boot_restore_from_{thermostat::OnBootRestoreFrom::MEMORY};
  */
};

}  // namespace wyt
}  // namespace pioneer
}  // namespace esphome