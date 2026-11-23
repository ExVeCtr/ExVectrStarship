#ifndef SAFETY_MOTOR_ENABLE_HPP
#define SAFETY_MOTOR_ENABLE_HPP

/**
 * Mission output "motorsEnable" and the one pure mapping that applies it (feature/mission-motor-enable).
 *
 * Pure logic: no Arduino, no ExVectrCore, no globals. Host-tested with `pio test -e native` (test/test_motor_enable).
 *
 * This is the SOFTWARE-CONTROL enable of the motors (StarshipTVC::enableMotors / enableActuators), never the
 * interlock. The hard gate in StarshipTVC (Safety::evaluateMotorGate) still ANDs the interlock, the forced-off latch,
 * enableMotors and enableActuators, and never looks at the simulation flag (safety.md rule 9).
 *
 * A mission publishes a MotorsRequest. The default is Unchanged: the mapping then reproduces exactly what
 * VehicleSafetyAndControlTask::vehicleShutdownControl did before this feature (test_motor_enable keeps the old truth
 * table as a reference). Only MissionFreefall requests Enable (the pre-spin, on entry to the tilt-back stage).
 * The controller phase table (control_phase.hpp) is independent of the request.
 */

#include <stdint.h>

namespace VCTR {
namespace Safety {

/// What a mission asks for the motor software-control enable.
enum class MotorsRequest : uint8_t {
  Unchanged = 0, ///< no request: enableMotors keeps its state, TVC actuators follow the mission as before
  Enable,        ///< motors enabled (enableMotors = true) and, because the hard gate needs it, the TVC actuators on
  Disable        ///< enableMotors = false
};

struct MotorEnableInput {
  bool shutdown = true;                 ///< vehicleShutdownControl(shutdown): not ready, zeroing, failure, abort path
  bool missionActuatorsEnabled = false; ///< MissionAbstract::getActuatorsEnabled()
  bool freefallActive = false;          ///< the active mission is MissionFreefall (flaps are always enabled there)
  MotorsRequest request = MotorsRequest::Unchanged;
  bool motorsEnabledNow = false;        ///< StarshipTVC::isMotorsEnabled() before this tick
};

struct MotorEnableOutput {
  bool enableMotors = false;      ///< StarshipTVC::enableMotors
  bool tvcActuators = false;      ///< StarshipTVC::enableActuators
  bool flapActuators = false;     ///< StarshipFlaps::enableActuators
  bool simTvc = false;            ///< BodySimulator::enableTVC (never follows a motors request: simulated thrust unchanged)
};

/// The mapping applied every tick. A shutdown forces TVC actuators and flaps off whatever the request is (the motor
/// gate then closes through enableActuators) and leaves enableMotors as it is, exactly as before this feature.
MotorEnableOutput motorEnableFor(const MotorEnableInput &in);

// ---- Freefall pre-spin (motors idle when the tilt-back stage starts, before the stop trigger and the upright flip) ----

/// The freefall tilt-back stage starts this far above the stop trigger altitude (stopDist + stopTarget). From there the
/// flap observer commands the 30 deg tilt-back pitch and no longer steers towards the home position (main.cpp
/// FlapModeObserverTask). The same constant and function decide the pre-spin, so the two cannot drift apart.
constexpr float kFreefallTiltBackMarginM = 20.0f;

/// True from the tilt-back stage on: altitude < stop trigger altitude + kFreefallTiltBackMarginM. A non-finite value
/// never counts (false).
inline bool freefallTiltBackStage(float altitudeM, float stopTriggerAltitudeM) {
  return altitudeM < stopTriggerAltitudeM + kFreefallTiltBackMarginM;
}

/// Latch: Enable once on entry to the tilt-back stage (never before, never twice) until reset() (mission begin / reset).
class FreefallSpinup {
public:
  void reset() { latched_ = false; }
  /// running = mission Running and not ended.
  MotorsRequest update(bool running, float altitudeM, float stopTriggerAltitudeM) {
    if (running && !latched_ && freefallTiltBackStage(altitudeM, stopTriggerAltitudeM))
      latched_ = true;
    return latched_ ? MotorsRequest::Enable : MotorsRequest::Unchanged;
  }
  bool latched() const { return latched_; }

private:
  bool latched_ = false;
};

/// Simulation only (owner decision 2026-11-23): the modelled thrust [X,Y,Z,T] that the body simulator receives is zero
/// while the motor gate output is closed (motors that get no pulses give no thrust and no vane force). gateOpen is
/// MotorGateResult::outputsEnabled, never the simulation flag. Open: unchanged.
inline void gateSimulatedThrust(const float in[4], bool gateOpen, float out[4]) {
  for (int i = 0; i < 4; i++)
    out[i] = gateOpen ? in[i] : 0.0f;
}

} // namespace Safety
} // namespace VCTR

#endif
