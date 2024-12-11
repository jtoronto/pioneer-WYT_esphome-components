#include "pioneer_wyt_protocol.h"
#include "esphome/core/log.h"

namespace esphome {
namespace remote_base {

static const char *const TAG = "remote.pioneer_wyt";

constexpr uint32_t HEADER_MARK_US = 3075;
constexpr uint32_t HEADER_SPACE_US = 1600;
constexpr uint32_t BIT_MARK_US = 486;
constexpr uint32_t BIT_ONE_SPACE_US = 1100;
constexpr uint32_t BIT_ZERO_SPACE_US = 333;
/*
constexpr uint32_t HEADER_MARK_US = 3100;
constexpr uint32_t HEADER_SPACE_US = 1650;
constexpr uint32_t BIT_MARK_US = 500;
constexpr uint32_t BIT_ONE_SPACE_US = 1100;
constexpr uint32_t BIT_ZERO_SPACE_US = 350;
*/

constexpr unsigned int PIONEER_WYT_IR_PACKET_BIT_SIZE = 112;

void PioneerWytProtocol::encode(RemoteTransmitData *dst, const PioneerWytData &data) {
  ESP_LOGI(TAG, "Transmit PioneerWyt: %s", format_hex_pretty(data.data).c_str());
  dst->set_carrier_frequency(37850);  // FIXME: Cleanup?
  // dst->set_carrier_frequency(38000);
  dst->reserve(5 + ((data.data.size() + 1) * 2));
  dst->mark(HEADER_MARK_US);
  dst->space(HEADER_SPACE_US);
  dst->mark(BIT_MARK_US);
  uint8_t checksum = 0;
  if (data.data[3] == 0x02)
    checksum = 0x0F;
  for (uint8_t item : data.data) {
    this->encode_byte_(dst, item);
    checksum += item;
  }
  this->encode_byte_(dst, checksum);
}

void PioneerWytProtocol::encode_byte_(RemoteTransmitData *dst, uint8_t item) {
  for (uint8_t b = 0; b < 8; b++) {
    if (item & (1UL << b)) {
      dst->space(BIT_ONE_SPACE_US);
    } else {
      dst->space(BIT_ZERO_SPACE_US);
    }
    dst->mark(BIT_MARK_US);
  }
}

optional<PioneerWytData> PioneerWytProtocol::decode(RemoteReceiveData src) {
  if (!src.expect_item(HEADER_MARK_US, HEADER_SPACE_US)) {
    return {};
  }
  if (!src.expect_mark(BIT_MARK_US)) {
    return {};
  }
  size_t size = src.size() - src.get_index() - 1;
  if (size < PIONEER_WYT_IR_PACKET_BIT_SIZE * 2)
    return {};
  size = PIONEER_WYT_IR_PACKET_BIT_SIZE * 2;
  uint8_t checksum = 0;
  PioneerWytData out;
  while (size > 0) {
    uint8_t data = 0;
    for (uint8_t b = 0; b < 8; b++) {
      if (src.expect_space(BIT_ONE_SPACE_US)) {
        data |= (1UL << b);
      } else if (!src.expect_space(BIT_ZERO_SPACE_US)) {
        return {};
      }
      if (!src.expect_mark(BIT_MARK_US)) {
        return {};
      }
      size -= 2;
    }
    if (size > 0) {
      checksum += (data >> 4) + (data & 0xF);
      out.data.push_back(data);
    } else if (checksum != data) {
      return {};
    }
  }
  return out;
}

void PioneerWytProtocol::dump(const PioneerWytData &data) {
  ESP_LOGI(TAG, "Received PioneerWyt: %s", format_hex_pretty(data.data).c_str());
}

}  // namespace remote_base
}  // namespace esphome
