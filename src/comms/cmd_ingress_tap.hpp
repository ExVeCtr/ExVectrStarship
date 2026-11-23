#ifndef COMMS_CMD_INGRESS_TAP_HPP
#define COMMS_CMD_INGRESS_TAP_HPP

/**
 * CmdIngressTap (GS-control cutover, docs/gs-control-design.md 4; replaces RcIngressTap): sits between one
 * StreamFrameLink and its Node and decodes the CMD_STATE frames (port 0x13) of that link itself, so the per-link
 * freshness is keyed by the real ingress link (the shared PortMux cannot tell the links apart, RxInfo.ingressLink is
 * 0 for both nodes). Port 0x13 is not subscribed on the PortMux, so its sequence state is never touched by it.
 * Port 0x10 (handset RC_STATE) is NOT interpreted: it is only counted (rcIgnored(), diagnostics) and forwarded.
 *
 * Every frame is forwarded unchanged to the next sink (node presence, radio link rule and counters are
 * unaffected). Only DATA frames addressed to this vehicle count. A CMD_STATE payload that is not exactly
 * CmdState::kMaxSize bytes or whose strict decode fails (any out-of-range field rejects the whole frame) is a
 * BadLength reject. The shared Safety::CommandIngress decides (source node 0, session latch, sequence); on
 * acceptance the tap feeds the mapper and the motor interlock (a session change resets both first). Vehicle-directed
 * REQUESTs (port 0x11, target 1) are checked against the latched session (design Q5) and counted; the vehicle has
 * no REQUEST executor yet (T9b-5), so the result is only recorded in the ingress counters.
 * Both taps run in the comms task, no locking between them. Pure of platform code, host-tested through
 * VehicleNetwork (test/test_vehicle_network).
 */

#include "ExVectrNetwork/link/FrameLinkI.hpp"

#include "safety/command_ingress.hpp"
#include "safety/motor_interlock.hpp"
#include "safety/rc_mapper.hpp"

namespace VCTR {
namespace Comms {

class CmdIngressTap : public network::FrameSinkI {
public:
  CmdIngressTap(Safety::CmdLink link, Safety::CommandIngress &ingress, Safety::RcCommandMapper &mapper,
                Safety::MotorInterlock &interlock, uint8_t vehicleNode)
      : link_(link), ingress_(ingress), mapper_(mapper), interlock_(interlock), vehicleNode_(vehicleNode) {}

  network::FrameSinkI *next = nullptr;

  void onFrame(const network::FrameView &f, uint8_t linkIndex) override;

  /// Handset RC_STATE frames addressed to the vehicle that were seen and ignored (diagnostics only).
  uint32_t rcIgnored() const { return rcIgnored_; }

private:
  Safety::CmdLink link_;
  Safety::CommandIngress &ingress_;
  Safety::RcCommandMapper &mapper_;
  Safety::MotorInterlock &interlock_;
  uint8_t vehicleNode_;
  uint32_t rcIgnored_ = 0;
};

} // namespace Comms
} // namespace VCTR

#endif
