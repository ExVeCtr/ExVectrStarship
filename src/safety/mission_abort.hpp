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
 * "On the ground" means BEFORE LIFTOFF and is decided by the mission mode/phase alone (user decision 2026-11-20,
 * documented in docs/project/missions.md section 6.2):
 * - the active mission is Idle (nothing running: reset, or never begun);
 * - the waypoint mission in Initialisation or Startup (countdown: the TVC controllers are off and the motors at most
 *   at idle, so the vehicle cannot fly);
 * - RTH that has finished (landed).
 * Everything else is "in flight": the waypoint mission Running (any phase after the countdown, even at 0.3 m and
 * 0.2 m/s right after liftoff) or Finished, bellyflop and freefall in any mode but Idle. There is deliberately NO
 * altitude/speed ground check any more (review 2026-11-20): an estimate-based "on the ground" right after liftoff
 * would cut the motors and drop the vehicle instead of starting RTH. Consequence: an abort right at T0 with the
 * vehicle still on the pad also starts RTH, which climbs to the home point (0, 0, 3 m), descends and shuts down.
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
};

AbortAction decideAbort(const AbortInput &in);

/// Short text for logs.
const char *toString(AbortAction a);

} // namespace Safety
} // namespace VCTR

#endif
