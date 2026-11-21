#ifndef SAFETY_MISSION_CHAIN_HPP
#define SAFETY_MISSION_CHAIN_HPP

/**
 * Mission chain decision (user decision 2026-11-20: a SystemReset resets the mission chain completely, it never
 * continues it).
 *
 * Pure logic: no Arduino, no ExVectrCore, no globals. Host-tested with `pio test -e native`
 * (test/test_mission_chain). VehicleSafetyAndControlTask (main.cpp) owns one instance. Every 10 Hz tick it describes
 * the active mission (ChainInput) and performs the returned step (switchMissionTo / RTH-end shutdown).
 *
 * Root cause this guards against: every mission's resetMission() sets missionEnd() = true (and the Idle tick of the
 * waypoint and RTH missions keeps it true), so "missionEnd()" alone cannot tell "finished, go to the next mission"
 * from "was reset". The old switching code advanced the chain on missionEnd() alone, so a SystemReset (and the
 * MissionAbort path for a mission that is not Running) made the next tick start the next mission:
 * waypoint -> bellyflop -> freefall -> RTH hot start.
 *
 * Rules:
 * - A chain is pending only between an accepted MissionBegin (begin()) and the end of the chain (RTH ended) or a
 *   reset (reset(): SystemReset, or a MissionAbort that resets and stops on the ground).
 * - The chain advances only while it is pending and the active mission has ended without being reset. "Reset"
 *   means ended AND mode Idle (resetMission() sets both); such a mission cancels the pending chain. An Idle mission
 *   that has not ended is not a reset and keeps the chain pending. No mission reports that state today: it came
 *   from MissionBellyflop's old taskInit override (run on the first unpaused tick after beginMission, it overwrote
 *   Running with Idle), which was removed on 2026-11-20 (docs/project/missions.md Known issues). The rule is kept
 *   as defence in depth, so a mission that ever reports Idle while still running would still chain when it ends.
 * - The RTH-end shutdown (RthEnded) is reported whenever the default mission (RTH) is active and ended, pending or
 *   not: it is also the steady state after boot and after a SystemReset (RTH Idle, missionEnd() true), and it only
 *   switches things off.
 * - RTH instead of the next mission (owner decisions 2026-11-21, X6 of the flight executive design applied to the
 *   current chain; redirect() tells which rule fired):
 *   (a) the active mission ended because a waypoint with the cancel flag (the hover check) was not reached in its
 *       time limit: SwitchToDefault (RTH hot start), whatever the next mission is. Before, the chain went on to the
 *       bellyflop and pitched the vehicle over at about 1 m.
 *   (b) the next mission is the bellyflop (powered kick) and the altitude (position estimate, NWU z above the
 *       position reference) is below kMinKickAltitudeM, or not finite: SwitchToDefault. Covers every reason the
 *       waypoint mission can end low, for example the ascent waypoint's time limit at 50 m. The normal flight
 *       reaches the 120 m apogee (waypoint threshold 3 m) and is unchanged.
 */

#include <stdint.h>

namespace VCTR {
namespace Safety {

/// Lowest altitude (m above the position reference, NWU z) at which the chain may start the bellyflop (powered kick).
/// Below it the chain goes to RTH instead (owner decision 2026-11-21). The normal apogee is about 120 m.
constexpr float kMinKickAltitudeM = 90.0f;

/// Why step() returned SwitchToDefault instead of the configured next mission (None: no redirect).
enum class ChainRedirect : uint8_t {
  None = 0,
  CancelledWaypoint, ///< rule (a): a cancel-flag waypoint (hover check) failed
  BelowKickAltitude, ///< rule (b): bellyflop next, but altitude < kMinKickAltitudeM (or not finite)
};

/// What VehicleSafetyAndControlTask does with the chain this tick.
enum class ChainStep : uint8_t {
  None = 0,        ///< nothing
  RthEnded,        ///< default mission (RTH) active and ended: motors off, flaps off, actuator shutdown
  SwitchToDefault, ///< switchMissionTo(default RTH)
  SwitchToNext,    ///< switchMissionTo(active->nextMission())
};

/// The active mission as the safety task sees it this tick.
struct ChainInput {
  bool activeIsDefault = false;     ///< the active mission is the default mission (RTH)
  bool activeIdle = false;          ///< the active mission is in mode Idle (reset or never begun)
  bool activeEnded = false;         ///< active->missionEnd()
  bool nextIsDefaultOrNone = false; ///< active->nextMission() is the default mission or nullptr
  bool nextIsBellyflop = false;     ///< active->nextMission() is the bellyflop (powered kick) mission
  bool activeCancelled = false;     ///< the active mission ended because a cancel-flag waypoint failed (hover check)
  float altitudeM = 0.0f;           ///< position estimate z, m above the position reference (NWU)
};

class MissionChain {
public:
  /// MissionBegin accepted (the first mission of the chain has just been begun).
  void begin() { pending_ = true; }

  /// SystemReset, or a MissionAbort that resets and stops on the ground: no chain step may follow.
  void reset() { pending_ = false; }

  /// True between begin() and the end of the chain or a reset.
  bool pending() const { return pending_; }

  /// Decides this tick's step. Clears pending() when the chain ends (RthEnded) or an Idle mission cancels it.
  ChainStep step(const ChainInput &in);

  /// Why the last step() returned SwitchToDefault instead of the next mission (None otherwise).
  ChainRedirect redirect() const { return redirect_; }

private:
  bool pending_ = false;
  ChainRedirect redirect_ = ChainRedirect::None;
};

/// Short text for logs.
const char *toString(ChainStep s);
const char *toString(ChainRedirect r);

} // namespace Safety
} // namespace VCTR

#endif
