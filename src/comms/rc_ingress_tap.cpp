#include "rc_ingress_tap.hpp"

#include <string.h>

#include "ExVectrNetwork/wire/NetHeader.hpp"
#include "ExVectrPackets/gen/all.hpp"

namespace VCTR {
namespace Comms {

void RcIngressTap::onFrame(const network::FrameView &f, uint8_t linkIndex) {
  using namespace network::wire;
  NetHeader h;
  constexpr size_t kPayloadAt = kNetHeaderLen + kDataHeaderLen;
  if (decodeNetHeader(f.data, f.len, h) && h.type == static_cast<uint8_t>(FrameType::Data) &&
      f.len >= kPayloadAt && (h.dst == vehicleNode_ || h.dst == kBroadcast) &&
      f.data[kNetHeaderLen] == Packets::RcState::kPort) {
    const uint8_t seq = f.data[kNetHeaderLen + 1];
    const size_t payloadLen = f.len - kPayloadAt;
    Packets::RcState rc;
    if (payloadLen != Packets::RcState::kMaxSize || !rc.decode(f.data + kPayloadAt, payloadLen)) {
      table_.onMalformed();
    } else {
      Safety::RcFrame frame;
      frame.flags = rc.flags;
      memcpy(frame.ch, rc.ch, sizeof frame.ch);
      Safety::ControlSourceId id = Safety::ControlSourceId::None;
      if (table_.onRcFrame(h.src, link_, seq, frame, f.rxTimeNs, &id)) mapper_.onAccepted(id, frame, f.rxTimeNs);
    }
  }
  if (next) next->onFrame(f, linkIndex);
}

} // namespace Comms
} // namespace VCTR
