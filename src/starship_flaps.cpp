#include "starship_flaps.hpp"

#include "ExVectrActuator/pwm_output.hpp"
#include "ExVectrActuator/servo_control.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrDSP/value_covariance.hpp"
#include "ExVectrHAL/pin_pwm.hpp"
#include "ExVectrMath.hpp"

namespace VCTR {
namespace CTRL {

StarshipFlaps::StarshipFlaps(HAL::PinPWM& flapServoULPin,
                             HAL::PinPWM& flapServoURPin,
                             HAL::PinPWM& flapServoLLPin,
                             HAL::PinPWM& flapServoLRPin)
    : Core::Task_Periodic("Starship Flaps", 20 * Core::MILLISECONDS),
      servoTLPin_(flapServoULPin, ACTR::PWM_Output_Protocol::STANDARD),
      servoTRPin_(flapServoURPin, ACTR::PWM_Output_Protocol::STANDARD),
      servoBLPin_(flapServoLLPin, ACTR::PWM_Output_Protocol::STANDARD),
      servoBRPin_(flapServoLRPin, ACTR::PWM_Output_Protocol::STANDARD) {
  Core::getSystemScheduler().addTask(*this);  // Attach to the scheduler
}

void StarshipFlaps::taskInit() {
  servoTLPin_.init();  // Initialize the top left flap servo
  servoTRPin_.init();  // Initialize the top right flap servo
  servoBLPin_.init();  // Initialize the bottom left flap servo
  servoBRPin_.init();  // Initialize the bottom right flap servo

  servoTLPin_.enableOutput(false);
  servoTRPin_.enableOutput(false);
  servoBLPin_.enableOutput(false);
  servoBRPin_.enableOutput(false);

  flapSettings_.flapTLAngle_Rad =
      0.0f;  // Initialize the top left flap angle to 0 radians
  flapSettings_.flapTRAngle_Rad =
      0.0f;  // Initialize the top right flap angle to 0 radians
  flapSettings_.flapBLAngle_Rad =
      0.0f;  // Initialize the bottom left flap angle to 0 radians
  flapSettings_.flapBRAngle_Rad =
      0.0f;  // Initialize the bottom right flap angle to 0 radians
  flapSettings_.enableFlaps =
      false;  // Initialize the actuator enable flag to false
}

void StarshipFlaps::beginActuatorTest(int64_t testOffset) {
  actuatorTestStartTime_ = Core::NOW() + testOffset;
  actuatorTestState_ = ActuatorTestingState::WaitBegin;
  // enableActuators_ = true;
  LOG_MSG("Flap test begin\n");
}

bool StarshipFlaps::testingActuators() {
  return actuatorTestState_ != ActuatorTestingState::Idle;
}

void StarshipFlaps::taskThread() {
  if (ctrlSubr_.isDataNew()) {
    flapSettings_ =
        ctrlSubr_.getItem();  // Get the new flap settings from the subscriber
  }

  auto flapSettings = flapSettings_;

  bool enable =
      enableActuators_ &&
      flapSettings_.enableFlaps;  // Check if the actuators should be enabled

  if (actuatorTestState_ !=
      ActuatorTestingState::Idle) {  // Hijack the control loop to test the
                                     // actuators

    // LOG_MSG("Actuator test state\n");

    bool finished = true;

    if (Core::NOW() - actuatorTestStartTime_ > 0) {
      actuatorTestState_ = ActuatorTestingState::Testing;
      enable = true;
    }

    if (actuatorTestState_ == ActuatorTestingState::WaitBegin) {
      finished = false;
    }

    auto actCalcFunc = [this](float phaseOffset) -> float {
      return (sin(phaseOffset + double(Core::NOW() - actuatorTestStartTime_) /
                                    Core::SECONDS / FLAP_TEST_DURATION * 2 *
                                    3.1415) +
              1) *
             3.14 / 2;  // Calculate the flap angle based on the phase offset
                        // and the current time
    };

    if (Core::NOW() - actuatorTestStartTime_ <
            FLAP_TEST_DURATION * Core::SECONDS &&
        Core::NOW() - actuatorTestStartTime_ > 0) {
      flapSettings.flapBLAngle_Rad =
          actCalcFunc(0);  // Set the top left flap angle
      flapSettings.flapTRAngle_Rad =
          actCalcFunc(3.14 / 4);  // Set the top left flap angle
      flapSettings.flapTLAngle_Rad =
          actCalcFunc(3.14 / 2);  // Set the top right flap angle
      flapSettings.flapBRAngle_Rad =
          actCalcFunc(3.14 * 3 / 4);  // Set the bottom left flap angle
      finished = false;
    }

    if (finished) {
      actuatorTestState_ = ActuatorTestingState::Idle;
    }
  }

  /*enable = true;
  float angle = 45 * DEGREES;
  flapSettings.flapTLAngle_Rad = angle;
  flapSettings.flapTRAngle_Rad = angle;
  flapSettings.flapBLAngle_Rad = angle;
  flapSettings.flapBRAngle_Rad = angle;*/

  servoTLPin_.enableOutput(enable);
  servoTRPin_.enableOutput(enable);
  servoBLPin_.enableOutput(enable);
  servoBRPin_.enableOutput(enable);

  if (enable) {
    float offset = 0;  //- 10 * DEGREES;
    float offsetTL = 0;
    float offsetTR = 0 * DEGREES;
    float offsetBL = 0;
    float offsetBR = 0 * DEGREES;
    limitAngle(flapSettings.flapTLAngle_Rad);
    limitAngle(flapSettings.flapTRAngle_Rad);
    limitAngle(flapSettings.flapBLAngle_Rad);
    limitAngle(flapSettings.flapBRAngle_Rad);
    // LOG_MSG("Flap angles: TL: %f, TR: %f, BL: %f, BR: %f\n",
    // flapSettings.tlAngle, flapSettings.trAngle, flapSettings.blAngle,
    // flapSettings.brAngle); // Log the flap angles
    servoTLPin_.setValue(1 -
                         (flapSettings.flapTLAngle_Rad + offset + offsetTL) /
                             3.1415 * 2);  // Set the top left flap angle
    servoTRPin_.setValue((flapSettings.flapTRAngle_Rad + offset + offsetTR) /
                         3.1415 * 2);  // Set the top right flap angle
    servoBLPin_.setValue((flapSettings.flapBLAngle_Rad + offset + offsetBL) /
                         3.1415 * 2);  // Set the bottom left flap angle
    servoBRPin_.setValue(1 -
                         (flapSettings.flapBRAngle_Rad + offset + offsetBR) /
                             3.1415 * 2);  // Set the bottom right flap angle
                                           /*float setting = 0;
                                           servoTLPin_.setValue(1 - setting); // Set the top left flap angle
                                           servoTRPin_.setValue(setting); // Set the top right flap angle
                                           servoBLPin_.setValue(1 - setting); // Set the bottom left flap angle
                                           servoBRPin_.setValue(setting); // Set the bottom right flap angle*/
  }
}

}  // namespace CTRL
}  // namespace VCTR