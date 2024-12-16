// #ifdef USE_REMOTE_TRANSMITTER  // FIXME
#pragma once

#include "wyt_climate.h"
#include "esphome/components/remote_base/pioneer_wyt_protocol.h"

namespace esphome {
namespace pioneer {
namespace wyt {

static const uint8_t WYT_REMOTE_HEADER_MAGIC[3] = {0x23, 0xCB, 0x26};
static const uint8_t WYT_REMOTE_HEADER_SIZE = 5;
static const uint8_t WYT_REMOTE_COMMAND_SIZE = 14;

enum class IrCommandType : uint8_t {
  Fan = 0x02,
  General = 0x01,
};

enum class IrMode : uint8_t {
  Heat = 0x01,
  Dry = 0x02,
  Cool = 0x03,
  Fan = 0x07,
  Auto = 0x08,
};

enum class IrAuxMode : uint8_t {
  None = 0x00,
  Sleep = 0x01,
  Dry = 0x02,
  Unknown = 0x03,  // FIXME: fan-medium, fan-medium-low have this set
  Turbo = 0x05,
};

enum class IrFanSpeed : uint8_t {
  Auto = 0x01,
  Low = 0x02,
  MediumLow = 0x03,
  Medium = 0x04,
  MediumHigh = 0x05,
  High = 0x06,
};

enum class IrSwing : uint8_t {
  Off = 0x00,
  On = 0x04,
};

enum class IrUpDownFlow : uint8_t {
  // FIXME: Verify these numbers
  Off = 0x00,  // Confirmed
  TopFix = 0x01,
  UpperFix = 0x02,
  MiddleFix = 0x03,
  LowerFix = 0x04,
  BottomFix = 0x05,
  UpDownFlow = 0x07,  // Confirmed
  UpFlow = 0x10,
  DownFlow = 0x18,
};

enum class IrLeftRightFlow : uint8_t {
  // FIXME: Verify these numbers
  Off = 0x10,  // Confirmed
  LeftFix = 0x01,
  MiddleLeftFix = 0x02,
  MiddleFix = 0x03,
  MiddleRightFix = 0x04,
  RightFix = 0x05,
  LeftRightFlow = 0x11,  // Confirmed
  LeftFlow = 0x00,
  MiddleFlow = 0x18,
  // RightFlow = 0x20, // Too many bits
};

// All commands sent to the WYT MCU begin with this header
union IrHeader {
  struct {
    // 00..02 - 0x23 0xCB 0x26
    uint8_t magic[3];
    // 03 - 0x02 or 0x01
    uint8_t msg_type;
    // 04 - 0x00
    uint8_t unknown;
  } __attribute__((packed));
  uint8_t bytes[WYT_REMOTE_HEADER_SIZE];
};

// Command to set the fan-mode settings of the WYT MCU
typedef union {
  struct {
    // 00..04
    IrHeader header;
    // 05
    // Every command has this bit set
    // Mute mode:    0x60 01100000
    // No mute mode: 0x40 01000000
    uint8_t unknown1 : 5;
    bool mute : 1;
    uint8_t unknown2 : 2;
    // 06
    uint8_t unknown3 : 5;
    IrFanSpeed fan_speed : 3;
    // 07
    // Horizontal swing 0x90 10010000
    // Vertical swing 0x08 00001000
    // The bit widths are assumed based on the spacing, but not confirmed
    uint8_t unknown4 : 2;
    IrSwing vertical_swing : 3;
    IrSwing horizontal_swing : 3;
    // 08
    uint8_t unknown5;
    // 09
    uint8_t unknown6;
    // 10
    uint8_t unknown7;
    // 11
    uint8_t unknown8;
    // 12
    uint8_t unknown9;
    // 13
    uint8_t checksum;
  } __attribute__((packed));
  uint8_t bytes[WYT_REMOTE_COMMAND_SIZE];
} IrFanCommand;

// Command to set the general features of the WYT MCU
typedef union {
  struct {
    // 00..04
    IrHeader header;
    // 05
    // Feel update: 0x04 00000100
    // Power off:   0x20 00100000
    // Power on:    0x24 00100100
    // Timer on:    0x2c 00101100
    // Display off: 0x64 01100100
    // Eco mode:    0xa4 10100100
    uint8_t unknown1 : 2;
    bool power : 1;
    bool timer : 1;
    bool unknown2 : 1;
    bool beeper : 1;
    bool disable_display : 1;
    bool eco : 1;
    // 06
    IrMode mode : 4;  // Dry mode also sets FanLow
    bool health : 1;
    bool unknown3 : 1;  // FIXME: Cleanup? i-feel-update1,3 have only this bit enabled
    bool turbo : 1;
    bool follow_me : 1;
    // 07
    // value: 31 - temperature C
    uint8_t setpoint_whole_number;
    // 08
    IrAuxMode aux_mode : 3;
    IrUpDownFlow up_down_flow : 5;  // FIXME: Verify last two bits
    // 09
    // Timer duration in 10 minute steps
    // Off:  0x00 00
    // 5h:   0x1e 30
    // 5.5h: 0x21 33
    // 6h:   0x24 36
    uint8_t timer_steps;
    // 10
    uint8_t unknown4;
    // 11
    // Remote temp sensor reading in C for follow me mode, otherwise 0x00
    uint8_t remote_temp;
    // 12
    // Horizontal flow?
    // Horizontal swing enabled:  0x8c 10001100
    // Horizontal swing disabled: 0x84 10000100
    // setpoint half digit - 0.0: 0x88 10001000
    // setpoint half digit - 0.5: 0x8c 10001100
    uint8_t unknown5 : 2;
    bool setpoint_half_digit : 1;
    IrLeftRightFlow left_right_flow : 5;  // FIXME: Verify?
    // 13
    uint8_t checksum;
  } __attribute__((packed));
  uint8_t bytes[WYT_REMOTE_COMMAND_SIZE];
} IrGeneralCommand;

using remote_base::RemoteTransmitterBase;

class IrTransmitter {
 public:
  void set_transmitter(remote_base::RemoteTransmitterBase *transmitter) { this->transmitter_ = transmitter; }
  void transmit(remote_base::PioneerWytData &data) {
    data.finalize();
    auto transmit = this->transmitter_->transmit();
    remote_base::PioneerWytProtocol().encode(transmit.get_data(), data);
    transmit.perform();
  }

 protected:
  remote_base::RemoteTransmitterBase *transmitter_{nullptr};
};

}  // namespace wyt
}  // namespace pioneer
}  // namespace esphome

// #endif // FIXME