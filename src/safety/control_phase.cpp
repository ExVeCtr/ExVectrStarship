#include "safety/control_phase.hpp"

namespace VCTR {
namespace Safety {

ControlPhase controlPhaseFor(ActiveMission active, MissionModeIn mode, bool missionActuatorsEnabled,
                             bool kickExpired) {
  ControlPhase p;
  bool tvcAllowed = true;
  if (active == ActiveMission::Bellyflop)
    tvcAllowed = kTvcDuringBellyflopTransition && !kickExpired;
  else if (active == ActiveMission::Freefall)
    tvcAllowed = false;
  p.tvcEnabled = tvcAllowed && mode == MissionModeIn::Running && missionActuatorsEnabled;
  // Bellyflop: the kick (TVC) and the flap attitude control are exclusive; freefall: flaps.
  p.bellyflopFlaps = active == ActiveMission::Freefall || (active == ActiveMission::Bellyflop && !p.tvcEnabled);
  if (active == ActiveMission::Bellyflop)
    p.tvcThrustCapN = kKickThrustLimitN;
  return p;
}

ControlPhaseStep ControlPhaseTracker::update(ActiveMission active, MissionModeIn mode, bool missionActuatorsEnabled,
                                             bool missionSwitched, int64_t nowNs) {
  if (active == ActiveMission::Bellyflop) {
    if (missionSwitched) {
      kickStartNs_ = nowNs;
      kickStartValid_ = true;
    }
  } else {
    kickStartValid_ = false;
  }
  const bool kickExpired = active == ActiveMission::Bellyflop &&
                           (!kickStartValid_ || nowNs - kickStartNs_ > kKickTimeLimitNs + kKickGuardMarginNs);
  ControlPhaseStep s;
  s.phase = controlPhaseFor(active, mode, missionActuatorsEnabled, kickExpired);
  s.resetTvc = s.phase.tvcEnabled && (!lastTvcEnabled_ || missionSwitched);
  s.changed = s.phase.tvcEnabled != lastTvcEnabled_;
  lastTvcEnabled_ = s.phase.tvcEnabled;
  return s;
}

} // namespace Safety
} // namespace VCTR
