#ifndef SAFETY_CONTROL_PHASE_HPP
#define SAFETY_CONTROL_PHASE_HPP

/**
 * Explicit controller phase table (user decision 2026-11-20).
 *
 * Pure logic: no Arduino, no ExVectrCore, no globals. Host-tested with `pio test -e native` (test/test_control_phase).
 * VehicleSafetyAndControlTask (main.cpp) owns one ControlPhaseTracker and applies its output once per 10 Hz tick,
 * after all mission switches of that tick (telecommands, failure checks, chain), so the same table is applied on
 * every run whatever happened before (no task-init or mode-edge dependence).
 *
 * Two controller sets:
 * - TVC set: ControlPositionStandard + ControlAttitudeTvc (powered flight). Enabled only in the powered phases: the
 *   waypoint mission, RTH (and any other mission) and, since 2026-11-21, the powered bellyflop kick, each in mode
 *   Running with the mission's actuators enabled. Disabled in every other phase, including freefall and the
 *   bellyflop after its kick: a disabled ControlAttitudeTvc publishes the neutral command [0, 0, 0.1 N, 0] (vanes
 *   centred, motors at the idle floor when the TVC actuators are enabled), never a stale deflection. During the
 *   kick the thrust is limited to kKickThrustLimitN (tvcThrustCapN, applied by StarshipTVC).
 * - Bellyflop set: ControlAttitudeFlaps in BellyFlopMode Stabilize (flap attitude control), selected by the flap
 *   mode observer in freefall and in the bellyflop whenever its TVC set is off. The two sets are exclusive in the
 *   bellyflop: during the kick the flaps stay in Ascent (retracted), the TVC controller alone drives the attitude.
 *   In the other phases the observer selects Ascent or, during RTH stabilisation, Upright. The flap controller is
 *   stateless (P + rate), so it needs no reset.
 *
 * Reset rule: the TVC set is reset (integrals, delta-time reference, setpoint hold) whenever it is enabled and was
 * disabled in the previous tick, or the active mission changed while it is enabled (for example waypoint -> RTH on
 * a MissionAbort), so no state collected in an earlier phase is carried into the next one.
 */

#include <stdint.h>

#include "safety/bellyflop_kick.hpp"

namespace VCTR {
namespace Safety {

/// Which mission object is active (main.cpp maps its pointers onto this).
enum class ActiveMission : uint8_t { Rth = 0, Waypoint, Bellyflop, Freefall, Other };

/// Mirror of MissionMode (telemetry.hpp), kept here so the logic stays platform free.
enum class MissionModeIn : uint8_t { Idle = 0, Initialisation, Startup, Running, Finished };

/**
 * TVC set during the bellyflop transition (MissionBellyflop), the powered kick. true = user decision 2026-11-21
 * ("the motors kick the ship into the bellyflop attitude, then go off and the flaps take over"); it replaces the
 * 2026-11-20 decision (false, unpowered pitch-over), under which the ship fell vertically in the simulator. The kick
 * lasts at most kKickTimeLimitNs (safety/bellyflop_kick.hpp). false restores the unpowered transition (flaps in
 * Stabilize for the whole bellyflop).
 */
constexpr bool kTvcDuringBellyflopTransition = true;

/// tvcThrustCapN when no phase-specific cap applies (the motor limit TVC_THRUST_LIMIT_N then limits alone).
constexpr float kNoTvcThrustCapN = 1.0e9f;

struct ControlPhase {
  bool tvcEnabled = false;     ///< TVC set (position + TVC attitude controller) enabled
  bool bellyflopFlaps = false; ///< bellyflop set: flap attitude control in BellyFlopMode Stabilize
  float tvcThrustCapN = kNoTvcThrustCapN; ///< thrust cap for StarshipTVC (kKickThrustLimitN in the bellyflop)
};

/**
 * The table. missionActuatorsEnabled = MissionAbstract::getActuatorsEnabled(). kickExpired = the bellyflop has
 * been active longer than kKickTimeLimitNs + kKickGuardMarginNs (ControlPhaseTracker): TVC set off regardless.
 */
ControlPhase controlPhaseFor(ActiveMission active, MissionModeIn mode, bool missionActuatorsEnabled,
                             bool kickExpired = false);

struct ControlPhaseStep {
  ControlPhase phase;
  bool resetTvc = false; ///< call resetControlState() on the TVC set before enabling it
  bool changed = false;  ///< tvcEnabled differs from the previous tick (for a log line)
};

class ControlPhaseTracker {
public:
  /**
   * missionSwitched = a mission was (re)begun since the last update (switchMissionTo counter changed). nowNs =
   * monotonic time; the kick guard measures the bellyflop's active time from the update in which it was switched
   * to. A bellyflop seen without that switch (cannot happen in main.cpp) counts as expired: TVC set off.
   */
  ControlPhaseStep update(ActiveMission active, MissionModeIn mode, bool missionActuatorsEnabled,
                          bool missionSwitched, int64_t nowNs);

  /// Boot state (SystemReset): TVC set disabled.
  void reset() {
    lastTvcEnabled_ = false;
    kickStartValid_ = false;
  }

  bool tvcEnabled() const { return lastTvcEnabled_; }

private:
  bool lastTvcEnabled_ = false;
  bool kickStartValid_ = false;
  int64_t kickStartNs_ = 0;
};

} // namespace Safety
} // namespace VCTR

#endif
