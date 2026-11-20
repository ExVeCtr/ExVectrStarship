#ifndef SAFETY_MISSION_ABORT_HPP
#define SAFETY_MISSION_ABORT_HPP

/**
 * MissionAbort decision (user decision 2026-11-20).
 *
 * Pure logic: no Arduino, no ExVectrCore, no globals. Host-tested with `pio test -e native` (test/test_mission_abort).
 *
 * Two kinds of stop exist. A FAILSAFE (latched Failure, for example an IMU failure) stops everything and the vehicle
 * falls; it is not handled here and is unchanged. An ABORT (the MissionAbort telecommand) means "stop and return
 * home":
 * - IN FLIGHT: start the RTH mission immediately (hot start from the current state) from any phase: waypoint
 *   hover / ascent, bellyflop transition, freefall. If RTH is already running it keeps running (a restart would
 *   throw away its progress, for example climb back from the landing descent to the translation).
 * - ON THE GROUND: reset and stop. The mission chain is cancelled, the mission goes Idle, motors and actuators off,
 *   the vehicle returns to Ready; nothing restarts.
 *
 * "On the ground" (documented in docs/project/missions.md section 6.2):
 * - the active mission has not left the ground by its phase: the waypoint mission in Idle, Initialisation or
 *   Startup (countdown: the TVC controllers are off and the motors at most at idle, so the vehicle cannot fly), or
 *   any mission that is Idle, or RTH that has finished (landed); or
 * - the ground check holds: estimated altitude above the start point below kGroundAltitudeMax_m AND estimated
 *   speed below kGroundSpeedMax_mps. It is applied to the missions that can only be flying by their phase
 *   (waypoint Running, bellyflop, freefall) to catch a vehicle that never lifted off (for example an abort right at
 *   T0, or a cancelled hover check that chained into bellyflop on the ground).
 * An abort is only accepted while the vehicle mode is Running (a mission was begun); otherwise it is ignored.
 */

#include <stdint.h>

#include "safety/control_phase.hpp"

namespace VCTR {
namespace Safety {

enum class AbortAction : uint8_t {
  Ignore = 0,   ///< vehicle not Running: nothing to abort
  ResetAndStop, ///< on the ground: cancel the chain, reset the mission (Idle), motors and actuators off, -> Ready
  StartRth,     ///< in flight: switch to RTH now (hot start)
  KeepRth,      ///< RTH already running: keep it
};

struct AbortInput {
  bool vehicleRunning = false; ///< vehicle mode Running
  ActiveMission active = ActiveMission::Rth;
  MissionModeIn mode = MissionModeIn::Idle;
  float altitude_m = 0;   ///< estimated altitude above the start point (position Pz, NWU)
  float speed_mps = 0;    ///< magnitude of the estimated velocity
};

constexpr float kGroundAltitudeMax_m = 0.5f;
constexpr float kGroundSpeedMax_mps = 0.5f;

/// Ground check from the estimated state alone (true = on the ground).
bool groundCheck(float altitude_m, float speed_mps);

AbortAction decideAbort(const AbortInput &in);

/// Short text for logs.
const char *toString(AbortAction a);

} // namespace Safety
} // namespace VCTR

#endif
