#include "plaato_airlock.h"

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace plaato_airlock {

static const char *TAG = "plaato_airlock.component";

static const uint8_t MPQ4242_CHIP_ID = 0x58;  // MPQ4242 default device id from part id

float PlaatoAirlockComponent::get_setup_priority() const { return setup_priority::DATA; }

void PlaatoAirlockComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PlaatoAirlock...");

  this->set_led_mode(PlaatoAirlockLEDMode::NO_MODE);
}

void PlaatoAirlockComponent::set_led_mode(uint8_t mode) { this->last_error_ = this->write(&mode, 1); }

void PlaatoAirlockComponent::loop() {}

void PlaatoAirlockComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "PlaatoAirlock component:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with PlaatoAirlock failed!");
  }
}

}  // namespace plaato_airlock
}  // namespace esphome