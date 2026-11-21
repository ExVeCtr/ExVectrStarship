#ifndef EXVECTRSTARSHIP_STARSHIPFLAPS_HPP
#define EXVECTRSTARSHIP_STARSHIPFLAPS_HPP

#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrMath/matrix_vector.hpp"

#include "ExVectrControl/starship/control_attitude_flaps.hpp"
#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrActuator/pwm_output.hpp"
#include "ExVectrActuator/servo_control.hpp"
#include "ExVectrHAL/pin_pwm.hpp"

#include "safety/sim_flap_fault.hpp"

namespace VCTR::CTRL {

class StarshipFlaps : public Core::Task_Periodic {
private:
  const float FLAP_TEST_DURATION = 4.0f;
  const float FLAP_MAX_ANGLE = 90 * DEGREES;
  const float FLAP_MIN_ANGLE = 15 * DEGREES;
  const float FLAP_SERVO_RATE_RADPS = 60 * DEGREES / 0.2f; // modelled servo rate, 300 deg/s

  Core::Simple_Subscriber<CTRL::ControlAttitudeFlapSetting> ctrlSubr_;
  Core::Topic_Publisher<CTRL::ControlAttitudeFlapSetting> trueFlapPub_;

  ACTR::PWM_Output servoTLPin_;
  ACTR::PWM_Output servoTRPin_;
  ACTR::PWM_Output servoBLPin_;
  ACTR::PWM_Output servoBRPin_;

  CTRL::ControlAttitudeFlapSetting flapSettings_;

  // Modelled ("true") flap angles [TL, TR, BL, BR] incl. the sim-only faults (Safety::TrueFlapModel). Published on
  // the true flap topic EVERY tick, enabled or not: the one source of the flap angle for the body simulator and the
  // telemetry (redesign 2026-11-21).
  Safety::TrueFlapModel trueFlaps_{FLAP_SERVO_RATE_RADPS};
  int64_t lastUpdateTime_ = 0;

  bool enableActuators_ = true;

  // Simulation-only servo faults (owner-approved 2026-11-21). Only the modelled true angle is affected, never the PWM.
  // Applied immediately, at TrueFlapModel::publishAngles().
  Safety::SimFlapFaults simFaults_;
  const bool *simulationFlag_ = nullptr; // read at apply time; nullptr = never simulation

  enum class ActuatorTestingState { WaitBegin, Testing, Idle };
  ActuatorTestingState actuatorTestState_ = ActuatorTestingState::Idle;

  int64_t actuatorTestStartTime_ = 0;

public:
  /**
   * @brief Constructs a new StarshipFlaps object.
   */
  StarshipFlaps(HAL::PinPWM &flapServoULPin, HAL::PinPWM &flapServoURPin,
                HAL::PinPWM &flapServoLLPin, HAL::PinPWM &flapServoLRPin);

  void setFlapSettingTopic(
      Core::Topic<CTRL::ControlAttitudeFlapSetting> &flapSettingTopic) {
    ctrlSubr_.subscribe(flapSettingTopic);
  }

  void setTrueFlapSettingTopic(
      Core::Topic<CTRL::ControlAttitudeFlapSetting> &trueFlapSettingTopic) {
    trueFlapPub_.subscribe(trueFlapSettingTopic);
  }

  /// The simulation flag the faults are gated by (read every update, not cached).
  void setSimulationFlag(const bool *flag) { simulationFlag_ = flag; }
  bool simulationActive() const { return simulationFlag_ != nullptr && *simulationFlag_; }
  Safety::SimFlapFaults &simFaults() { return simFaults_; }

  void enableActuators(bool enable) { enableActuators_ = enable; }
  bool actuatorsEnabled() const { return enableActuators_; }

  // void taskCheck() override;

  void taskInit() override;

  void taskThread() override;

  void beginActuatorTest(int64_t testOffset);

  bool testingActuators();

private:
  void limitAngle(float &angle) const {
    if (angle > FLAP_MAX_ANGLE) {
      angle = FLAP_MAX_ANGLE;
    } else if (angle < FLAP_MIN_ANGLE) {
      angle = FLAP_MIN_ANGLE;
    }
  }
};

} // namespace VCTR::CTRL

#endif