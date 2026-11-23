#include "cmd_ingress_tap.hpp"

#include "ExVectrNetwork/wire/NetHeader.hpp"
#include "ExVectrPackets/gen/all.hpp"

namespace VCTR {
namespace Comms {

void CmdIngressTap::onFrame(const network::FrameView &f, uint8_t linkIndex) {
  using namespace network::wire;
  NetHeader h;
  constexpr size_t kPayloadAt = kNetHeaderLen + kDataHeaderLen;
  if (decodeNetHeader(f.data, f.len, h) && h.type == static_cast<uint8_t>(FrameType::Data) &&
      f.len >= kPayloadAt && h.dst == vehicleNode_) {
    const uint8_t port = f.data[kNetHeaderLen];
    const uint8_t seq = f.data[kNetHeaderLen + 1];
    const size_t payloadLen = f.len - kPayloadAt;
    if (port == Packets::CmdState::kPort) {
      Packets::CmdState cs;
      if (payloadLen != Packets::CmdState::kMaxSize || !cs.decode(f.data + kPayloadAt, payloadLen)) {
        ingress_.onMalformed();
      } else {
        Safety::CmdFrame frame;
        frame.flags = cs.flags;
        frame.session = cs.session;
        frame.go = static_cast<uint8_t>(cs.go);
        frame.profile = cs.profile;
        frame.vehicleMode = static_cast<uint8_t>(cs.vehicle_mode);
        frame.motors = static_cast<uint8_t>(cs.motors);
        frame.resetButton = static_cast<uint8_t>(cs.reset_button);
        frame.failsafe = static_cast<uint8_t>(cs.failsafe);
        const Safety::CommandIngress::Result r = ingress_.onFrame(h.src, link_, seq, frame, f.rxTimeNs);
        if (r.sessionChanged) { // every detector sees the safe value again before the first state of the session
          mapper_.onSessionChanged();
          interlock_.onSessionChanged();
        }
        if (r.accepted) {
          mapper_.onAccepted(frame, f.rxTimeNs);
          interlock_.onCmdState(frame.motors == 1, f.rxTimeNs);
        }
      }
    } else if (port == Packets::RcState::kPort) {
      rcIgnored_++; // the handset stream is not a command stream (owner decision 2026-11-23)
    } else if (port == Packets::Request::kPort && payloadLen >= Packets::Request::kMinSize) {
      Packets::Request rq;
      if (rq.decode(f.data + kPayloadAt, payloadLen)) {
        ingress_.onRequest(rq.session); // counters only; the executor re-checks session, target and action
        Safety::RequestMsg m;
        m.link = static_cast<uint8_t>(link_);
        m.src = h.src;
        m.reqId = rq.req_id;
        m.action = rq.action;
        m.target = rq.target;
        m.argI = rq.arg_i;
        m.argF = rq.arg_f;
        m.session = rq.session;
        requests_.push(m);
      }
    }
  }
  if (next) next->onFrame(f, linkIndex);
}

} // namespace Comms
} // namespace VCTR
