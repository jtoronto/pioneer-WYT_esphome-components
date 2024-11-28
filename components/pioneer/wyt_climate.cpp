#include "wyt_climate.h"
#include "esphome/core/log.h"

#include <cfloat>
#include <cmath>
#include <stdint.h>

namespace esphome {
namespace pioneer {
namespace wyt {

static const char *const TAG = "pioneer.climate";

void WytClimate::setup() {
  this->prev_custom_fan_mode_.reset();
  this->custom_fan_mode.reset();
  this->prev_fan_mode_.reset();
  this->fan_mode.reset();

  if (!this->query_state_()) {
    ESP_LOGE(TAG, "Status query timed out");
    return;
  }
}

bool WytClimate::query_state_(bool read_only) {
  if (!read_only) {
    ESP_LOGV(TAG, "Sending query command");
    this->write_array(WYT_QUERY_COMMAND, WYT_QUERY_COMMAND_SIZE);
    this->flush();
  }

  bool result = this->read_array(this->raw_state_, WYT_QUERY_RESPONSE_SIZE);
  if (!result) {
    return false;
  }
  if (this->raw_state_[0] != 0xBB) {
    // Clear the buffer
    uint8_t byte;
    while (this->available()) {
      this->read_byte(&byte);
    }
    return false;
  }

  uint8_t checksum = this->raw_state_[WYT_QUERY_RESPONSE_SIZE - 1];
  if (checksum != this->response_checksum(this->raw_state_)) {
    ESP_LOGW(TAG, "Checksum mismatch: 0x%x != 0x%x", checksum, this->response_checksum(this->raw_state_));
    return false;
  }

  this->state_ = response_from_bytes(this->raw_state_);

  this->current_temperature = this->get_temperature();
  if (this->prev_current_temperature_ != this->current_temperature) {
    this->prev_current_temperature_ = this->current_temperature;
    this->publish_state();
  }

  return true;
}

void WytClimate::update() {
  if (!this->query_state_()) {
    ESP_LOGE(TAG, "Status query timed out");
  }

  // Set the current mode
  this->mode = this->get_mode();
  // this->action = this->get_action();  // FIXME: Implement this if possible
  this->fan_mode = this->get_fan_mode();
  this->custom_fan_mode = this->get_custom_fan_mode();
  this->swing_mode = this->get_swing_mode();
  this->target_temperature = this->get_setpoint();
  this->current_temperature = this->get_temperature();

  if (this->current_temperature != this->prev_current_temperature_ ||
      this->target_temperature != this->prev_target_temperature_ ||
      this->custom_fan_mode != this->prev_custom_fan_mode_ || this->fan_mode != this->prev_fan_mode_ ||
      this->mode != this->prev_mode_ || this->swing_mode != this->prev_swing_mode_) {
    // Update the reference state
    this->prev_current_temperature_ = this->current_temperature;
    this->prev_target_temperature_ = this->target_temperature;
    this->prev_swing_mode_ = this->swing_mode;
    this->prev_mode_ = this->mode;
    this->prev_fan_mode_ = this->fan_mode;
    this->prev_custom_fan_mode_ = this->custom_fan_mode;
    this->publish_state();
  }
}

void WytClimate::refresh() {
  this->command = this->command_from_response(this->state_);

  this->switch_to_mode_(this->mode, false);
  // this->switch_to_action_(this->compute_action_(), false);  // FIXME: Use OutdoorStatus, four-way valve status, etc.?
  if (this->fan_mode.has_value())
    this->switch_to_fan_mode_(this->fan_mode.value(), false);
  else if (this->custom_fan_mode.has_value())
    this->switch_to_custom_fan_mode_(this->custom_fan_mode.value(), false);
  this->switch_to_swing_mode_(this->swing_mode, false);
  this->validate_target_temperature();
  this->switch_to_setpoint_temperature_();

  this->send_command(this->command);
  this->publish_state();
}

void WytClimate::validate_target_temperature() {
  if (std::isnan(this->target_temperature)) {
    this->target_temperature =
        ((this->get_traits().get_visual_max_temperature() - this->get_traits().get_visual_min_temperature()) / 2) +
        this->get_traits().get_visual_min_temperature();
  } else {
    // target_temperature must be between the visual minimum and the visual maximum
    if (this->target_temperature < this->get_traits().get_visual_min_temperature())
      this->target_temperature = this->get_traits().get_visual_min_temperature();
    if (this->target_temperature > this->get_traits().get_visual_max_temperature())
      this->target_temperature = this->get_traits().get_visual_max_temperature();
  }
}

void WytClimate::control(const climate::ClimateCall &call) {
  /* FIXME: Implement this
  if (call.get_preset().has_value()) {
    // setup_complete_ blocks modifying/resetting the temps immediately after boot
    if (this->setup_complete_) {
      this->change_preset_(*call.get_preset());
    } else {
      this->preset = *call.get_preset();
    }
  }
  */

  if (call.get_mode().has_value())
    this->mode = *call.get_mode();
  if (call.get_fan_mode().has_value())
    this->fan_mode = *call.get_fan_mode();
  if (call.get_custom_fan_mode().has_value())
    this->custom_fan_mode = *call.get_custom_fan_mode();
  if (call.get_swing_mode().has_value())
    this->swing_mode = *call.get_swing_mode();
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
    validate_target_temperature();
  }

  // make any changes happen
  this->refresh();
}

climate::ClimateTraits WytClimate::traits() {
  auto traits = climate::ClimateTraits();
  // traits.set_supports_action(true); // FIXME: Find out if possible
  traits.set_supports_current_temperature(true);

  traits.add_supported_mode(climate::CLIMATE_MODE_AUTO);
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_DRY);
  traits.add_supported_mode(climate::CLIMATE_MODE_FAN_ONLY);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);

  traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_MEDIUM);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_QUIET);
  traits.set_supported_custom_fan_modes({"Medium-Low", "Medium-High", "Turbo"});

  traits.add_supported_swing_mode(climate::CLIMATE_SWING_BOTH);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_HORIZONTAL);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_VERTICAL);

  traits.set_visual_target_temperature_step(1.0);
  traits.set_visual_current_temperature_step(1.0);

  return traits;
}

void WytClimate::switch_to_action_(climate::ClimateAction action, bool publish_state) {
  if (action == this->action) {
    ESP_LOGI(TAG, "Already showing target action %s", climate::climate_action_to_string(action));
    return;
  }

  if ((action == climate::CLIMATE_ACTION_OFF && this->action == climate::CLIMATE_ACTION_IDLE) ||
      (action == climate::CLIMATE_ACTION_IDLE && this->action == climate::CLIMATE_ACTION_OFF)) {
    // switching from OFF to IDLE or vice-versa -- this is only a visual difference.
    // OFF means user manually disabled, IDLE means the temperature is in target range.
    this->action = action;
    if (publish_state)
      this->publish_state();
    return;
  }

  /* FIXME: Cleanup
  switch (action) {
    case climate::CLIMATE_ACTION_OFF:
    case climate::CLIMATE_ACTION_IDLE:
      break;
    case climate::CLIMATE_ACTION_COOLING:
      break;
    case climate::CLIMATE_ACTION_HEATING:
      if (this->heating_action_ready_()) {
        this->start_timer_(thermostat::TIMER_HEATING_ON);
        this->start_timer_(thermostat::TIMER_HEATING_MAX_RUN_TIME);
        if (this->supports_fan_with_heating_) {
          this->start_timer_(thermostat::TIMER_FANNING_ON);
          trig_fan = this->fan_only_action_trigger_;
        }
        this->heating_max_runtime_exceeded_ = false;
        trig = this->heat_action_trigger_;
        ESP_LOGVV(TAG, "Switching to HEATING action");
        action_ready = true;
      }
      break;
    case climate::CLIMATE_ACTION_FAN:
      if (this->fanning_action_ready_()) {
        if (this->supports_fan_only_action_uses_fan_mode_timer_) {
          this->start_timer_(thermostat::TIMER_FAN_MODE);
        } else {
          this->start_timer_(thermostat::TIMER_FANNING_ON);
        }
        trig = this->fan_only_action_trigger_;
        ESP_LOGVV(TAG, "Switching to FAN_ONLY action");
        action_ready = true;
      }
      break;
    case climate::CLIMATE_ACTION_DRYING:
      if (this->drying_action_ready_()) {
        this->start_timer_(thermostat::TIMER_COOLING_ON);
        this->start_timer_(thermostat::TIMER_FANNING_ON);
        trig = this->dry_action_trigger_;
        ESP_LOGVV(TAG, "Switching to DRYING action");
        action_ready = true;
      }
      break;
    default:
      // we cannot report an invalid mode back to HA (even if it asked for one)
      //  and must assume some valid value
      action = climate::CLIMATE_ACTION_OFF;
      // trig = this->idle_action_trigger_;
  }
  */
}

void WytClimate::switch_to_fan_mode_(climate::ClimateFanMode fan_mode, bool publish_state) {
  if (fan_mode == this->prev_fan_mode_) {
    ESP_LOGI(TAG, "Already in target fan mode %s", climate::climate_fan_mode_to_string(fan_mode));
    return;
  }

  this->command.mute = false;
  this->command.strong = false;
  switch (fan_mode) {
    case climate::CLIMATE_FAN_LOW:
      this->command.fan_speed = CmdFanSpeed::Low;
      break;
    case climate::CLIMATE_FAN_MEDIUM:
      this->command.fan_speed = CmdFanSpeed::Medium;
      break;
    case climate::CLIMATE_FAN_HIGH:
      this->command.fan_speed = CmdFanSpeed::High;
      break;
    case climate::CLIMATE_FAN_QUIET:
      this->command.fan_speed = CmdFanSpeed::Low;
      this->command.mute = true;
      break;
    case climate::CLIMATE_FAN_AUTO:
    default:
      // we cannot report an invalid mode back to HA (even if it asked for one)
      //  and must assume some valid value
      this->command.fan_speed = CmdFanSpeed::Auto;
      fan_mode = climate::CLIMATE_FAN_AUTO;
  }

  // Clear any custom fan modes, this is highlander rules
  this->prev_custom_fan_mode_.reset();
  this->custom_fan_mode.reset();

  this->prev_fan_mode_ = this->get_fan_mode();
  this->fan_mode = fan_mode;
  if (publish_state) {
    this->send_command(command);
    this->publish_state();
  }
}

void WytClimate::switch_to_custom_fan_mode_(std::string custom_fan_mode, bool publish_state) {
  if (custom_fan_mode == this->prev_custom_fan_mode_) {
    ESP_LOGI(TAG, "Already in target custom fan mode %s", custom_fan_mode);
    return;
  }

  this->command.mute = false;
  this->command.strong = false;
  if (custom_fan_mode == "Medium-Low") {
    this->command.fan_speed = CmdFanSpeed::MediumLow;
  } else if (custom_fan_mode == "Medium-High") {
    command.fan_speed = CmdFanSpeed::MediumHigh;
  } else {
    // we cannot report an invalid mode back to HA (even if it asked for one)
    //  and must assume some valid value
    command.fan_speed = CmdFanSpeed::High;
    this->command.strong = true;
    custom_fan_mode = "Turbo";
  }

  // Clear any fan modes, this is highlander rules
  this->prev_fan_mode_.reset();
  this->fan_mode.reset();

  this->prev_custom_fan_mode_ = this->get_custom_fan_mode();
  this->custom_fan_mode = custom_fan_mode;
  if (publish_state) {
    this->send_command(command);
    this->publish_state();
  }
}

void WytClimate::switch_to_mode_(climate::ClimateMode mode, bool publish_state) {
  if (mode == this->get_mode()) {
    ESP_LOGI(TAG, "Already in target mode %s", climate::climate_mode_to_string(mode));
    return;
  }

  this->command.power = true;
  switch (mode) {
    case climate::CLIMATE_MODE_OFF:
      this->command.mode = CmdMode::Off;
      this->command.power = false;
      break;
    case climate::CLIMATE_MODE_COOL:
      this->command.mode = CmdMode::Cool;
      break;
    case climate::CLIMATE_MODE_HEAT:
      this->command.mode = CmdMode::Heat;
      break;
    case climate::CLIMATE_MODE_FAN_ONLY:
      this->command.mode = CmdMode::Fan;
      break;
    case climate::CLIMATE_MODE_DRY:
      this->command.mode = CmdMode::Dehumidify;
      break;
    case climate::CLIMATE_MODE_AUTO:
    default:
      // we cannot report an invalid mode back to HA (even if it asked for one)
      //  and must assume some valid value
      this->command.mode = CmdMode::Auto;
      mode = climate::CLIMATE_MODE_AUTO;
  }

  this->prev_mode_ = this->get_mode();
  this->mode = mode;
  if (publish_state) {
    this->send_command(command);
    this->publish_state();
  }
}

void WytClimate::switch_to_swing_mode_(climate::ClimateSwingMode swing_mode, bool publish_state) {
  if (swing_mode == this->get_swing_mode()) {
    ESP_LOGI(TAG, "Already in target swing mode %s", climate::climate_swing_mode_to_string(swing_mode));
    return;
  }

  SetCommand command = this->command_from_response(this->state_);

  switch (swing_mode) {
    case climate::CLIMATE_SWING_BOTH:
      command.left_right_flow = LeftRightFlow::LeftRightFlow;
      command.up_down_flow = UpDownFlow::UpDownFlow;
      break;
    case climate::CLIMATE_SWING_HORIZONTAL:
      command.left_right_flow = LeftRightFlow::LeftRightFlow;
      command.up_down_flow = UpDownFlow::Auto;
      break;
    case climate::CLIMATE_SWING_VERTICAL:
      command.left_right_flow = LeftRightFlow::Auto;
      command.up_down_flow = UpDownFlow::UpDownFlow;
      break;
    case climate::CLIMATE_SWING_OFF:
    default:
      // we cannot report an invalid mode back to HA (even if it asked for one)
      //  and must assume some valid value
      command.left_right_flow = LeftRightFlow::Auto;
      command.up_down_flow = UpDownFlow::Auto;
      swing_mode = climate::CLIMATE_SWING_OFF;
  }
  this->prev_swing_mode_ = this->get_swing_mode();
  this->swing_mode = swing_mode;
  if (publish_state) {
    this->send_command(command);
    this->publish_state();
  }
}

void WytClimate::switch_to_setpoint_temperature_() {
  if (this->prev_target_temperature_ == this->target_temperature) {
    return;  // nothing changed, no reason to trigger
  }
  // save the new temperature so we can compare later
  this->prev_target_temperature_ = this->target_temperature;

  this->set_temperature_(command, this->target_temperature);
}

void WytClimate::dump_config() { LOG_CLIMATE("", "WytClimate", this); }

uint8_t WytClimate::checksum(const SetCommand &command) {
  uint8_t result = 0;
  // Skip the magic byte
  for (size_t i = 0; i < WYT_STATE_COMMAND_SIZE - 1; ++i) {
    result ^= command.bytes[i];
  }
  return result;
}

Header WytClimate::new_header(const Source &source, const Dest &dest, const Command &command, const uint8_t size) {
  Header header;
  header.magic = 0xbb;
  header.source = source;
  header.dest = dest;
  header.command = command;
  header.length = size;
  return header;
}

SetCommand WytClimate::command_from_bytes(const uint8_t buffer[WYT_STATE_COMMAND_SIZE]) {
  SetCommand command;
  memcpy(command.bytes, buffer, WYT_STATE_COMMAND_SIZE);
  return command;
}

SetCommand WytClimate::command_from_response(const Response &response) {
  SetCommand command;
  Header header = this->new_header(Source::Controller, Dest::Appliance, Command::Set, 0x1d);
  command.header = header;
  command.eco = response.eco;
  command.display = this->enable_display_;
  command.beeper = this->enable_beeper_;
  command.gen_mode = 0x0;  // FIXME: Always disable gen_mode for now
  command.power = response.power;
  command.mute = response.mute;
  command.strong = response.strong;
  command.health = response.health;
  switch (response.mode) {
    case Mode::Auto:
      command.mode = CmdMode::Auto;
      break;
    case Mode::Cool:
      command.mode = CmdMode::Cool;
      break;
    case Mode::Dehumidify:
      command.mode = CmdMode::Dehumidify;
      break;
    case Mode::Fan:
      command.mode = CmdMode::Fan;
      break;
    case Mode::Heat:
      command.mode = CmdMode::Heat;
      break;
  }
  command.freeze_protection = response.freeze_protection;
  command.vertical_flow = response.vertical_flow;
  switch (response.fan_speed) {
    case FanSpeed::Auto:
      command.fan_speed = CmdFanSpeed::Auto;
      break;
    case FanSpeed::Low:
      command.fan_speed = CmdFanSpeed::Low;
      break;
    case FanSpeed::MediumLow:
      command.fan_speed = CmdFanSpeed::MediumLow;
      break;
    case FanSpeed::Medium:
      command.fan_speed = CmdFanSpeed::Medium;
      break;
    case FanSpeed::MediumHigh:
      command.fan_speed = CmdFanSpeed::MediumHigh;
      break;
    case FanSpeed::High:
      command.fan_speed = CmdFanSpeed::High;
      break;
  }
  this->set_temperature_(command, this->get_setpoint());
  command.unknown8[0] = 0x80;
  command.sleep = response.sleep;
  command.up_down_flow = response.up_down_flow;
  command.left_right_flow = static_cast<LeftRightFlow>(static_cast<uint8_t>(response.left_right_flow) + 0x80);
  command.checksum = checksum(command);
  return command;
}

void WytClimate::send_command(SetCommand &command) {
  // Make sure left_right_flow has 0x80 added, but not more than once
  command.left_right_flow = static_cast<LeftRightFlow>((static_cast<uint8_t>(command.left_right_flow) % 0x80) + 0x80);
  command.checksum = this->checksum(command);
  this->write_array(command.bytes, WYT_STATE_COMMAND_SIZE);
  this->flush();
}

Response WytClimate::response_from_bytes(const uint8_t buffer[WYT_QUERY_RESPONSE_SIZE]) {
  Response response;
  memcpy(response.bytes, buffer, WYT_QUERY_RESPONSE_SIZE);
  return response;
}

uint8_t WytClimate::response_checksum(const uint8_t buffer[WYT_QUERY_RESPONSE_SIZE]) {
  uint8_t result = 0;
  // Skip the magic byte
  for (size_t i = 0; i < WYT_QUERY_RESPONSE_SIZE - 1; ++i) {
    result ^= buffer[i];
  }
  return result;
}

/* FIXME: Cleanup?
climate::ClimateAction WytClimate::get_action() {}
*/

climate::ClimateMode WytClimate::get_mode() {
  if (!this->state_.power)
    return climate::CLIMATE_MODE_OFF;
  switch (this->state_.mode) {
    case Mode::Auto:
      return climate::CLIMATE_MODE_AUTO;
    case Mode::Cool:
      return climate::CLIMATE_MODE_COOL;
    case Mode::Dehumidify:
      return climate::CLIMATE_MODE_DRY;
    case Mode::Fan:
      return climate::CLIMATE_MODE_FAN_ONLY;
    case Mode::Heat:
      return climate::CLIMATE_MODE_HEAT;
    default:
      return climate::CLIMATE_MODE_OFF;
  }
}

optional<std::string> WytClimate::get_custom_fan_mode() {
  switch (this->state_.fan_speed) {
    case FanSpeed::MediumLow:
      return optional<std::string>("Medium-Low");
    case FanSpeed::MediumHigh:
      return optional<std::string>("Medium-High");
    case FanSpeed::High:
      if (this->state_.strong)
        return optional<std::string>("Turbo");
    default:
      return optional<std::string>();
  }
}

optional<climate::ClimateFanMode> WytClimate::get_fan_mode() {
  switch (this->state_.fan_speed) {
    case FanSpeed::Low:
      if (this->state_.mute)
        return climate::CLIMATE_FAN_QUIET;
      return climate::CLIMATE_FAN_LOW;
    case FanSpeed::Medium:
      return climate::CLIMATE_FAN_MEDIUM;
    case FanSpeed::High:
      return climate::CLIMATE_FAN_HIGH;
    case FanSpeed::Auto:
      return climate::CLIMATE_FAN_AUTO;
    default:
      return optional<climate::ClimateFanMode>();
  }
}

climate::ClimateSwingMode WytClimate::get_swing_mode() {
  bool left_right = (this->state_.left_right_flow == LeftRightFlow::LeftRightFlow);
  bool up_down = (this->state_.up_down_flow == UpDownFlow::UpDownFlow);

  if (left_right && up_down)
    return climate::CLIMATE_SWING_BOTH;
  else if (left_right)
    return climate::CLIMATE_SWING_HORIZONTAL;
  else if (up_down)
    return climate::CLIMATE_SWING_VERTICAL;
  return climate::CLIMATE_SWING_OFF;
}

float WytClimate::get_setpoint() {
  return 16 + this->state_.set_temperature_whole + (this->state_.set_temperature_half ? 0.5 : 0);
}

/*
Calibration points:
Reading, App Temp, Converted, Rounding Range, Calculated Result
0x69: 105, 72F, 22.22C, 21.94 < x < 22.49, 22.0
0x6b: 107, 73F, 22.78C, 22.50 < x < 23.05, 22.8
0x6e: 110, 75F, 23.89C, 23.61 < x < 24.16, 24.0
0x70: 112, 77F, 25.00C, 24.72 < x < 25.27, 24.8
0x73: 115, 79F, 26.11C, 25.83 < x < 26.38, 26.0
*/
float WytClimate::get_temperature() { return this->state_.indoor_temp_base * 0.4 - 20; }

void WytClimate::set_temperature_(SetCommand &command, const float temp_c) {
  // Double before rounding because 77F/24.94°C * 2 = 49.88, which is cast to 49, adding extra setpoint bumps
  float temp_double_float = temp_c * 2;
  uint8_t temp_double = round(temp_double_float);
  uint8_t temp_whole = temp_double / 2;
  bool temp_half = temp_double % 2;
  command.set_temperature_whole = 0x6f - temp_whole;
  command.set_temperature_half = temp_half;
}

}  // namespace wyt
}  // namespace pioneer
}  // namespace esphome