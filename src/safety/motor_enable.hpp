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
 * table as a reference). Only MissionFreefall requests Enable (the pre-spin, kMotorSpinupLeadS before its stop trigger).
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

// ---- Freefall pre-spin (motors idle shortly before the stop trigger and the upright flip) ----

/// Request Enable when the predicted time to the stop trigger is at most this (proposal, owner to confirm on bench).
constexpr float kMotorSpinupLeadS = 1.5f;
/// Fallback when the vertical speed is not usable (non-finite, or no braking margin): request Enable this far above
/// the stop altitude target. An unusable altitude never enables.
constexpr float kMotorSpinupMarginM = 10.0f;

struct FreefallSpinupInput {
  bool running = false;     ///< mission Running and not ended
  float altitudeM = 0;      ///< P(5), the quantity of the stop logic
  float vzMps = 0;          ///< V(2), the quantity of the stop logic
  float stopTargetM = 0;    ///< MissionFreefall::getStopTargetAltitude()
  float thrustLimitN = 0;   ///< stop logic thrust
  float massKg = 1;         ///< stop logic mass
};

/// The stop trigger of MissionFreefall: alt < v^2 / (2 (T/m - g)) + stopTarget (same expression).
bool freefallStopTriggered(float altitudeM, float vzMps, float stopTargetM, float thrustLimitN, float massKg);

/// True if the stop trigger holds now or is predicted to hold within kMotorSpinupLeadS (ballistic prediction, no drag:
/// drag only delays the trigger, so the pre-spin comes early, never late). Fallback margin as above.
bool freefallSpinupDue(const FreefallSpinupInput &in);

/// Latch: Enable once inside the lead (never before, never twice) until reset() (mission begin / reset).
class FreefallSpinup {
public:
  void reset() { latched_ = false; }
  MotorsRequest update(const FreefallSpinupInput &in) {
    if (!in.running)
      return latched_ ? MotorsRequest::Enable : MotorsRequest::Unchanged;
    if (!latched_ && freefallSpinupDue(in))
      latched_ = true;
    return latched_ ? MotorsRequest::Enable : MotorsRequest::Unchanged;
  }
  bool latched() const { return latched_; }

private:
  bool latched_ = false;
};

} // namespace Safety
} // namespace VCTR

#endif
