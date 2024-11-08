#include "plaato_airlock_sensor.h"

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace plaato_airlock {

static const char *TAG = "plaato_airlock.sensor";

float PlaatoAirlockSensor::get_setup_priority() const { return setup_priority::DATA; }

void PlaatoAirlockSensor::setup() { ESP_LOGCONFIG(TAG, "Setting up PlaatoAirlockSensor..."); }

void PlaatoAirlockSensor::update() {
  uint32_t bubble_count;
  uint32_t bubble_total;
  uint8_t bytes[9];
  this->last_error_ = this->read(bytes, 9);
  bubble_count = (uint32_t) bytes[4] | (uint32_t) bytes[3] << 8 | (uint32_t) bytes[2] << 16 | (uint32_t) bytes[1] << 24;
  bubble_total = (uint32_t) bytes[8] | (uint32_t) bytes[7] << 8 | (uint32_t) bytes[6] << 16 | (uint32_t) bytes[5] << 24;

  if (this->bubble_count_sensor_ != nullptr) {
    this->bubble_count_sensor_->publish_state(bubble_count);
  }
  if (this->bubble_total_sensor_ != nullptr) {
    this->bubble_total_sensor_->publish_state(bubble_total);
  }
  /* FIXME: implement checking for reset flag
      if (bytes[0]) {
        reset_flag = true;
      } else {
        reset_flag = false;
      }
  */
}

void PlaatoAirlockSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "PlaatoAirlock Sensor:");
  LOG_SENSOR("  ", "BubbleCountSensor", this->bubble_count_sensor_);
  LOG_SENSOR("  ", "BubbleTotalSensor", this->bubble_total_sensor_);
}

}  // namespace plaato_airlock
}  // namespace esphome
