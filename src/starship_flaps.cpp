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

namespace VCTR::CTRL /* StarshipFlaps */ {

StarshipFlaps::StarshipFlaps(HAL::PinPWM &flapServoULPin,
                             HAL::PinPWM &flapServoURPin,
                             HAL::PinPWM &flapServoLLPin,
                             HAL::PinPWM &flapServoLRPin)
    : Core::Task_Periodic("Starship Flaps", 20 * Core::MILLISECONDS),
      servoTLPin_(flapServoULPin, ACTR::PWM_Output_Protocol::STANDARD),
      servoTRPin_(flapServoURPin, ACTR::PWM_Output_Protocol::STANDARD),
      servoBLPin_(flapServoLLPin, ACTR::PWM_Output_Protocol::STANDARD),
      servoBRPin_(flapServoLRPin, ACTR::PWM_Output_Protocol::STANDARD) {
  Core::getSystemScheduler().addTask(*this); // Attach to the scheduler
}

void StarshipFlaps::taskInit() {
  servoTLPin_.init(); // Initialize the top left flap servo
  servoTRPin_.init(); // Initialize the top right flap servo
  servoBLPin_.init(); // Initialize the bottom left flap servo
  servoBRPin_.init(); // Initialize the bottom right flap servo

  servoTLPin_.enableOutput(false);
  servoTRPin_.enableOutput(false);
  servoBLPin_.enableOutput(false);
  servoBRPin_.enableOutput(false);

  // Modelled ("true") flap angle: generic 9 g servos, 60 deg per 0.2 s = 300 deg/s (owner 2026-11-21; was
  // 60 deg per 0.3 s). Used for telemetry and as the simulator's flap input.
  trueFlaps_.setRate(FLAP_SERVO_RATE_RADPS);

  flapSettings_.flapTLAngle_Rad =
      0.0f; // Initialize the top left flap angle to 0 radians
  flapSettings_.flapTRAngle_Rad =
      0.0f; // Initialize the top right flap angle to 0 radians
  flapSettings_.flapBLAngle_Rad =
      0.0f; // Initialize the bottom left flap angle to 0 radians
  flapSettings_.flapBRAngle_Rad =
      0.0f; // Initialize the bottom right flap angle to 0 radians
  flapSettings_.enableFlaps =
      false; // Initialize the actuator enable flag to false
}

void StarshipFlaps::beginActuatorTest(int64_t testOffset) {
  actuatorTestStartTime_ = Core::NowNs() + testOffset;
  actuatorTestState_ = ActuatorTestingState::WaitBegin;
  // enableActuators_ = true;
  LOG_MSG("Flap test begin\n");
}

bool StarshipFlaps::testingActuators() {
  return actuatorTestState_ != ActuatorTestingState::Idle;
}

void StarshipFlaps::taskThread() {
  if (ctrlSubr_.isDataNew()) {
    flapSettings_ = ctrlSubr_.getItem();
  }

  auto flapSettings = flapSettings_;

  bool enable = enableActuators_ && flapSettings_.enableFlaps;

  if (actuatorTestState_ != ActuatorTestingState::Idle) {

    bool finished = true;

    if (Core::NowNs() - actuatorTestStartTime_ > 0) {
      actuatorTestState_ = ActuatorTestingState::Testing;
      enable = true;
    }

    if (actuatorTestState_ == ActuatorTestingState::WaitBegin) {
      finished = false;
    }

    auto actCalcFunc = [this](float phaseOffset) -> float {
      return (sin(phaseOffset + double(Core::NowNs() - actuatorTestStartTime_) /
                                    Core::SECONDS / FLAP_TEST_DURATION * 2 *
                                    3.1415) +
              1) *
             3.14 / 2;
    };

    if (Core::NowNs() - actuatorTestStartTime_ <
            FLAP_TEST_DURATION * Core::SECONDS &&
        Core::NowNs() - actuatorTestStartTime_ > 0) {
      flapSettings.flapBLAngle_Rad = actCalcFunc(0);
      flapSettings.flapTRAngle_Rad = actCalcFunc(3.14 / 4);
      flapSettings.flapTLAngle_Rad = actCalcFunc(3.14 / 2);
      flapSettings.flapBRAngle_Rad = actCalcFunc(3.14 * 3 / 4);
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
    float offset = 0;
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
    servoTLPin_.setValue(
        1 - (flapSettings.flapTLAngle_Rad + offset + offsetTL) / 3.1415 * 2);
    servoTRPin_.setValue((flapSettings.flapTRAngle_Rad + offset + offsetTR) /
                         3.1415 * 2);
    servoBLPin_.setValue((flapSettings.flapBLAngle_Rad + offset + offsetBL) /
                         3.1415 * 2);
    servoBRPin_.setValue(
        1 - (flapSettings.flapBRAngle_Rad + offset + offsetBR) / 3.1415 * 2);

    const float targets[4] = {flapSettings.flapTLAngle_Rad, flapSettings.flapTRAngle_Rad,
                              flapSettings.flapBLAngle_Rad, flapSettings.flapBRAngle_Rad};
    trueFlaps_.setTargets(targets);
  }

  // Modelled true flap angle. Powered: the servo model moves toward the clamped command. Not powered: nothing moves
  // (an unpowered servo stays where it is). The faults (simulation only, owner-approved 2026-11-21, safety.md rule 9
  // exception) are applied at ONE point, publishAngles(), and the result is published EVERY tick, so the body
  // simulator and the telemetry show a fault immediately, also with the flaps disabled. The flag is read here at apply
  // time; outside simulation rateFactor is exactly 1.0f and effectiveAngle returns the model angle unchanged.
  const auto newUpdateTime = Core::NowNs();
  const bool simActive = simulationActive();
  if (enable && newUpdateTime - lastUpdateTime_ > 1 * Core::MILLISECONDS) {
    trueFlaps_.advance(newUpdateTime - lastUpdateTime_, simFaults_, simActive);
  }
  if (!enable || newUpdateTime - lastUpdateTime_ > 1 * Core::MILLISECONDS)
    lastUpdateTime_ = newUpdateTime;

  float trueAngle[4];
  trueFlaps_.publishAngles(simFaults_, simActive, trueAngle);
  trueFlapPub_.publish(CTRL::ControlAttitudeFlapSetting{.flapTLAngle_Rad = trueAngle[0],
                                                        .flapTRAngle_Rad = trueAngle[1],
                                                        .flapBLAngle_Rad = trueAngle[2],
                                                        .flapBRAngle_Rad = trueAngle[3],
                                                        .enableFlaps = enableActuators_});
}

} // namespace VCTR::CTRL