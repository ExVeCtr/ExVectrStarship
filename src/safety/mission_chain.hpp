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
 *   reset (reset(): SystemReset, MissionAbort of a mission that is not Running).
 * - The chain advances only while it is pending and the active mission has ended without being reset, i.e. it is
 *   not in mode Idle. An Idle active mission cancels the pending chain.
 * - The RTH-end shutdown (RthEnded) is reported whenever the default mission (RTH) is active and ended, pending or
 *   not: it is also the steady state after boot and after a SystemReset (RTH Idle, missionEnd() true), and it only
 *   switches things off.
 */

#include <stdint.h>

namespace VCTR {
namespace Safety {

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
};

class MissionChain {
public:
  /// MissionBegin accepted (the first mission of the chain has just been begun).
  void begin() { pending_ = true; }

  /// SystemReset, or MissionAbort of a mission that is not Running: no chain step may follow.
  void reset() { pending_ = false; }

  /// True between begin() and the end of the chain or a reset.
  bool pending() const { return pending_; }

  /// Decides this tick's step. Clears pending() when the chain ends (RthEnded) or an Idle mission cancels it.
  ChainStep step(const ChainInput &in);

private:
  bool pending_ = false;
};

/// Short text for logs.
const char *toString(ChainStep s);

} // namespace Safety
} // namespace VCTR

#endif
