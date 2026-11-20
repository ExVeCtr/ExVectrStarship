#include "safety/control_phase.hpp"

namespace VCTR {
namespace Safety {

ControlPhase controlPhaseFor(ActiveMission active, MissionModeIn mode, bool missionActuatorsEnabled) {
  ControlPhase p;
  const bool bellyflopSet = (active == ActiveMission::Bellyflop || active == ActiveMission::Freefall);
  const bool tvcAllowed = active == ActiveMission::Bellyflop ? kTvcDuringBellyflopTransition
                                                             : active != ActiveMission::Freefall;
  p.tvcEnabled = tvcAllowed && mode == MissionModeIn::Running && missionActuatorsEnabled;
  p.bellyflopFlaps = bellyflopSet;
  return p;
}

ControlPhaseStep ControlPhaseTracker::update(ActiveMission active, MissionModeIn mode, bool missionActuatorsEnabled,
                                             bool missionSwitched) {
  ControlPhaseStep s;
  s.phase = controlPhaseFor(active, mode, missionActuatorsEnabled);
  s.resetTvc = s.phase.tvcEnabled && (!lastTvcEnabled_ || missionSwitched);
  s.changed = s.phase.tvcEnabled != lastTvcEnabled_;
  lastTvcEnabled_ = s.phase.tvcEnabled;
  return s;
}

} // namespace Safety
} // namespace VCTR
