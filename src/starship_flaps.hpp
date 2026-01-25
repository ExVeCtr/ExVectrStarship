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

namespace VCTR::CTRL {

class ServoAngleSimulator {
private:
  float currentAngleRad_ = 0;
  float targetAngleRad_ = 0;
  float radsLimit = 45 * DEGREES; // max radians per second
public:
  // Default radsLimit is of a typical cheap 9g servo
  ServoAngleSimulator(float radsLimit = 60 * DEGREES / 0.1f);
  void setRadiansPerSecondLimit(float radsPerSecond);
  void setTargetAngle(float angleRad);
  float getCurrentAngle() const;
  void update(int64_t deltaTime);
};

class StarshipFlaps : public Core::Task_Periodic {
private:
  const float FLAP_TEST_DURATION = 4.0f;
  const float FLAP_MAX_ANGLE = 90 * DEGREES;
  const float FLAP_MIN_ANGLE = 15 * DEGREES;

  Core::Simple_Subscriber<CTRL::ControlAttitudeFlapSetting> ctrlSubr_;
  Core::Topic_Publisher<CTRL::ControlAttitudeFlapSetting> trueFlapPub_;

  ACTR::PWM_Output servoTLPin_;
  ACTR::PWM_Output servoTRPin_;
  ACTR::PWM_Output servoBLPin_;
  ACTR::PWM_Output servoBRPin_;

  CTRL::ControlAttitudeFlapSetting flapSettings_;

  ServoAngleSimulator tlFlapSim_;
  ServoAngleSimulator trFlapSim_;
  ServoAngleSimulator blFlapSim_;
  ServoAngleSimulator brFlapSim_;
  int64_t lastUpdateTime_ = 0;

  bool enableActuators_ = true;

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