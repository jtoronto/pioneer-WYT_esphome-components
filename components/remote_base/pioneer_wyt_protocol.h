#pragma once

#include "esphome/core/component.h"
#include "remote_base.h"

namespace esphome {
namespace remote_base {

struct PioneerWytData {
  std::vector<uint8_t> data;

  bool operator==(const PioneerWytData &rhs) const { return data == rhs.data; }
};

class PioneerWytProtocol : public RemoteProtocol<PioneerWytData> {
 public:
  void encode(RemoteTransmitData *dst, const PioneerWytData &data) override;
  optional<PioneerWytData> decode(RemoteReceiveData src) override;
  void dump(const PioneerWytData &data) override;

 protected:
  void encode_byte_(RemoteTransmitData *dst, uint8_t item);
};

DECLARE_REMOTE_PROTOCOL(PioneerWyt)

template<typename... Ts> class PioneerWytAction : public RemoteTransmitterActionBase<Ts...> {
 public:
  TEMPLATABLE_VALUE(std::vector<uint8_t>, code)

  void encode(RemoteTransmitData *dst, Ts... x) override {
    PioneerWytData data{};
    data.data = this->code_.value(x...);
    PioneerWytProtocol().encode(dst, data);
  }
};

}  // namespace remote_base
}  // namespace esphome
