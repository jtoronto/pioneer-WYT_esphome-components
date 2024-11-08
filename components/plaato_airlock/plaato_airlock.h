#pragma once
#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace plaato_airlock {

#define CHECK_BIT(var, pos) (((var) >> (pos)) & 1)

static const uint8_t MPQ4242_REGISTER_CLK_ON = 0x39;

/// FIXME: Update this description for the component
/// This class includes i2c support for the MPQ4242 USB PD controller.
/// The device has 7 configurable PDOs, 2 GPIOs with several functions
/// and is capable of up to 100W charging. This class is for the
/// MPQ4242 configuration.
class PlaatoAirlockComponent : public i2c::I2CDevice, public Component {
 public:
  void loop() override;

  enum PlaatoAirlockLEDMode : uint8_t {
    NO_MODE,
    COUNTING1,
    COUNTING2,
    SLOWBREATHING,
    FASTBREATHING,
    SLOWFLASH,
    FASTFLASH,
    ALLOFF,
    ALLON,
    BOT,
    MID,
    TOP,
    SLOWDOWN,
    FASTDOWN,
    SLOWUP,
    FASTUP,
    BOTSLOWFLASH,
    BOTFASTFLASH,
    MIDSLOWFLASH,
    MIDFASTFLASH,
    TOPSLOWFLASH,
    TOPFASTFLASH,
    BOTONBUBBLE
  };

  void set_led_mode(uint8_t mode);

  /** Used by ESPHome framework. */
  void setup() override;
  /** Used by ESPHome framework. */
  void dump_config() override;
  /** Used by ESPHome framework. */
  float get_setup_priority() const override;

 protected:
  i2c::ErrorCode last_error_;
};

}  // namespace plaato_airlock
}  // namespace esphome