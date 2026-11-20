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
 *   waypoint mission and RTH (and any other mission) in mode Running with the mission's actuators enabled.
 *   Disabled in every other phase, including the bellyflop transition and freefall: a disabled ControlAttitudeTvc
 *   publishes the neutral command [0, 0, 0.1 N, 0] (vanes centred, motors at the idle floor when the TVC
 *   actuators are enabled), never a stale deflection.
 * - Bellyflop set: ControlAttitudeFlaps in BellyFlopMode Stabilize (flap attitude control), selected by the flap
 *   mode observer during the bellyflop transition and freefall. In the other phases the observer selects Ascent
 *   (flaps retracted) or, during RTH stabilisation, Upright. The flap controller is stateless (P + rate), so it
 *   needs no reset.
 *
 * Reset rule: the TVC set is reset (integrals, delta-time reference, setpoint hold) whenever it is enabled and was
 * disabled in the previous tick, or the active mission changed while it is enabled (for example waypoint -> RTH on
 * a MissionAbort), so no state collected in an earlier phase is carried into the next one.
 */

#include <stdint.h>

namespace VCTR {
namespace Safety {

/// Which mission object is active (main.cpp maps its pointers onto this).
enum class ActiveMission : uint8_t { Rth = 0, Waypoint, Bellyflop, Freefall, Other };

/// Mirror of MissionMode (telemetry.hpp), kept here so the logic stays platform free.
enum class MissionModeIn : uint8_t { Idle = 0, Initialisation, Startup, Running, Finished };

/**
 * TVC set during the bellyflop transition (MissionBellyflop). false = user decision 2026-11-20 ("TVC off during
 * bellyflop and freefall"). Note: MissionBellyflop was written to pitch the vehicle over with the TVC attitude
 * controller (fixed -90 deg pitch setpoint); with this false that command is inert and the pitch-over is unpowered
 * (bench simulation 2026-11-20: tilt 0.2 -> 2.3 deg in the 1 s transition, ended by the time limit). The old
 * firmware's real flights also show TVC thrust 0 during the pitch-over. Changing this one constant restores the
 * powered pitch-over (the behaviour of every bellyflop after the first since boot on earlier firmware).
 */
constexpr bool kTvcDuringBellyflopTransition = false;

struct ControlPhase {
  bool tvcEnabled = false;     ///< TVC set (position + TVC attitude controller) enabled
  bool bellyflopFlaps = false; ///< bellyflop set: flap attitude control in BellyFlopMode Stabilize
};

/// The table. missionActuatorsEnabled = MissionAbstract::getActuatorsEnabled().
ControlPhase controlPhaseFor(ActiveMission active, MissionModeIn mode, bool missionActuatorsEnabled);

struct ControlPhaseStep {
  ControlPhase phase;
  bool resetTvc = false; ///< call resetControlState() on the TVC set before enabling it
  bool changed = false;  ///< tvcEnabled differs from the previous tick (for a log line)
};

class ControlPhaseTracker {
public:
  /// missionSwitched = a mission was (re)begun since the last update (switchMissionTo counter changed).
  ControlPhaseStep update(ActiveMission active, MissionModeIn mode, bool missionActuatorsEnabled,
                          bool missionSwitched);

  /// Boot state (SystemReset): TVC set disabled.
  void reset() { lastTvcEnabled_ = false; }

  bool tvcEnabled() const { return lastTvcEnabled_; }

private:
  bool lastTvcEnabled_ = false;
};

} // namespace Safety
} // namespace VCTR

#endif
