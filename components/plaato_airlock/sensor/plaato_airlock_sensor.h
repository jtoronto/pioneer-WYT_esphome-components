#pragma once

#include "../plaato_airlock.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/defines.h"
#include "esphome/core/component.h"

namespace esphome {
namespace plaato_airlock {

/// This class includes i2c support for the Plaato Airlock.
class PlaatoAirlockSensor : public PollingComponent {
 public:
  PlaatoAirlockSensor(PlaatoAirlockComponent *parent) : parent_(parent) {}

  /** Sets the sensor that will report the number of bubbles seen. */
  void set_bubble_count_sensor(sensor::Sensor *sensor) { this->bubble_count_sensor_ = sensor; }

  /** Sets the sensor that will report the total size of bubbles seen. */
  void set_bubble_total_sensor(sensor::Sensor *sensor) { this->bubble_total_sensor_ = sensor; }

  /** Used by ESPHome framework. */
  void dump_config() override;
  /** Used by ESPHome framework. */
  void setup() override;
  /** Used by ESPHome framework. */
  void update() override;
  /** Used by ESPHome framework. */
  float get_setup_priority() const override;

 protected:
  PlaatoAirlockComponent *parent_;

  sensor::Sensor *bubble_count_sensor_{nullptr};
  sensor::Sensor *bubble_total_sensor_{nullptr};
};

}  // namespace plaato_airlock
}  // namespace esphome