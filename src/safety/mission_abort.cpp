#include "safety/mission_abort.hpp"

namespace VCTR {
namespace Safety {

bool groundCheck(float altitude_m, float speed_mps) {
  // NaN compares false: an invalid estimate never counts as "on the ground", so an abort then starts RTH.
  return altitude_m < kGroundAltitudeMax_m && speed_mps < kGroundSpeedMax_mps;
}

AbortAction decideAbort(const AbortInput &in) {
  if (!in.vehicleRunning)
    return AbortAction::Ignore;

  if (in.mode == MissionModeIn::Idle)
    return AbortAction::ResetAndStop; // nothing is running (reset mission, or never begun)

  const bool onGround = groundCheck(in.altitude_m, in.speed_mps);

  switch (in.active) {
  case ActiveMission::Rth:
    if (in.mode == MissionModeIn::Running)
      return AbortAction::KeepRth;
    return AbortAction::ResetAndStop; // Finished = landed (or Init/Startup, which RTH never uses)

  case ActiveMission::Waypoint:
    if (in.mode == MissionModeIn::Initialisation || in.mode == MissionModeIn::Startup)
      return AbortAction::ResetAndStop; // countdown
    return onGround ? AbortAction::ResetAndStop : AbortAction::StartRth;

  case ActiveMission::Bellyflop:
  case ActiveMission::Freefall:
  case ActiveMission::Other:
    return onGround ? AbortAction::ResetAndStop : AbortAction::StartRth;
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
