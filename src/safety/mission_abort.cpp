#include "safety/mission_abort.hpp"

namespace VCTR {
namespace Safety {

AbortAction decideAbort(const AbortInput &in) {
  if (!in.vehicleRunning)
    return AbortAction::Ignore;

  if (in.mode == MissionModeIn::Idle)
    return AbortAction::ResetAndStop; // nothing is running (reset mission, or never begun)

  switch (in.active) {
  case ActiveMission::Rth:
    if (in.mode == MissionModeIn::Running)
      return AbortAction::KeepRth;
    return AbortAction::ResetAndStop; // Finished = landed (or Init/Startup, which RTH never uses)

  case ActiveMission::Waypoint:
    if (in.mode == MissionModeIn::Initialisation || in.mode == MissionModeIn::Startup)
      return AbortAction::ResetAndStop; // countdown, before liftoff
    return AbortAction::StartRth;       // Running (any phase after the countdown) or Finished: in flight

  case ActiveMission::Bellyflop:
  case ActiveMission::Freefall:
  case ActiveMission::Other:
    // Running, or Finished for at most one safety tick before the chain switches on: always in flight.
    return AbortAction::StartRth;
  }
  return AbortAction::StartRth;
}

const char *toString(AbortAction a) {
  switch (a) {
  case AbortAction::Ignore:
    return "ignored (vehicle not running)";
  case AbortAction::ResetAndStop:
    return "on the ground: reset and stop";
  case AbortAction::StartRth:
    return "in flight: start RTH";
  case AbortAction::KeepRth:
    return "RTH already running: keep it";
  }
  return "?";
}

} // namespace Safety
} // namespace VCTR
