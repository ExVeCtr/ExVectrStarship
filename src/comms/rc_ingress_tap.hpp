#ifndef COMMS_RC_INGRESS_TAP_HPP
#define COMMS_RC_INGRESS_TAP_HPP

/**
 * RcIngressTap (T9b-1, docs/t9b-design.md 3.1.4): sits between one StreamFrameLink and its Node and decodes the
 * RC_STATE frames (port 0x10) of that link itself, so the control source is keyed by the real ingress link (the
 * shared PortMux cannot tell the links apart, RxInfo.ingressLink is 0 for both nodes). Port 0x10 is not
 * subscribed on the PortMux, so its sequence state is never touched by RC.
 *
 * Every frame is forwarded unchanged to the next sink (node presence, radio link rule and counters are
 * unaffected). Only DATA frames addressed to this vehicle or to broadcast count. A payload that is not exactly
 * RcState::kMaxSize bytes or does not decode is a BadLength reject. Accepted states feed the mapper and the motor
 * interlock (T9b-2; the SD channel decoded by RcCommandMapper::decodeTwoPos, only 1 = ENABLED counts as enabled).
 * Pure of platform code, host-tested through VehicleNetwork (test/test_vehicle_network).
 */

#include "ExVectrNetwork/link/FrameLinkI.hpp"

#include "safety/control_source.hpp"
#include "safety/motor_interlock.hpp"
#include "safety/rc_mapper.hpp"

namespace VCTR {
namespace Comms {

class RcIngressTap : public network::FrameSinkI {
public:
  RcIngressTap(Safety::RcLink link, Safety::ControlSourceTable &table, Safety::RcCommandMapper &mapper,
               Safety::MotorInterlock &interlock, uint8_t vehicleNode)
      : link_(link), table_(table), mapper_(mapper), interlock_(interlock), vehicleNode_(vehicleNode) {}

  network::FrameSinkI *next = nullptr;

  void onFrame(const network::FrameView &f, uint8_t linkIndex) override;

private:
  Safety::RcLink link_;
  Safety::ControlSourceTable &table_;
  Safety::RcCommandMapper &mapper_;
  Safety::MotorInterlock &interlock_;
  uint8_t vehicleNode_;
};

} // namespace Comms
} // namespace VCTR

#endif
