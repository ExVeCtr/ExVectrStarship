#include "starship_tvc.hpp"

#include "ExVectrActuator/pwm_output.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrDSP/value_covariance.hpp"
#include "ExVectrMath.hpp"

namespace VCTR {
namespace CTRL {

StarshipTVC::StarshipTVC(HAL::PinPWM &tvcServoPinXP, HAL::PinPWM &tvcServoPinXN,
                         HAL::PinPWM &tvcServoPinYP, HAL::PinPWM &tvcServoPinYN,
                         HAL::PinPWM &motorCW, HAL::PinPWM &motorCCW,
                         float tvcAngleLimit_Rad, float servoAngleLimit_Rad,
                         float tvcThrustLimit_N, float tvcFactor)
    : Core::Task_Periodic("Starship TVC", 20 * Core::MILLISECONDS),
      servoXP_(tvcServoPinXP, ACTR::PWM_Output_Protocol::STANDARD),
      servoXN_(tvcServoPinXN, ACTR::PWM_Output_Protocol::STANDARD),
      servoYP_(tvcServoPinYP, ACTR::PWM_Output_Protocol::STANDARD),
      servoYN_(tvcServoPinYN, ACTR::PWM_Output_Protocol::STANDARD),
      motorCW_(motorCW, ACTR::PWM_Output_Protocol::STANDARD),
      motorCCW_(motorCCW, ACTR::PWM_Output_Protocol::STANDARD) {
  Core::getSystemScheduler().addTask(*this); // Attach to the scheduler
  setRelease(Core::END_OF_TIME);
  tvcAngleLimit_Rad_ = tvcAngleLimit_Rad;
  servoAngleLimit_Rad_ = servoAngleLimit_Rad;
  tvcThrustLimit_N_ = tvcThrustLimit_N;
  tvcFactor_ = tvcFactor;
}

void StarshipTVC::taskCheck() {
  if (ctrlSubr_.isDataNew()) {
    setDeadline(0);
  }
}

void StarshipTVC::taskInit() {
  servoXP_.init();
  servoXN_.init();
  servoYP_.init();
  servoYN_.init();

  motorCW_.init();
  motorCCW_.init();
}

void StarshipTVC::beginActuatorTest(int64_t testOffset) {
  actuatorTestStartTime_ = Core::NowNs() + testOffset;
  actuatorTestState_ = ActuatorTestingState::WaitBegin;
  // enableActuators_ = true;
  LOG_MSG("TVC test begin\n");
}

bool StarshipTVC::testingActuators() {
  return actuatorTestState_ != ActuatorTestingState::Idle;
}

void StarshipTVC::forceMotorsOff() {
  motorsForcedOff_ = true;
  motorCW_.enableOutput(false); // duty 0 (the PWM_Output only writes 0 here; setValue below is never called while closed)
  motorCCW_.enableOutput(false);
  if (actualThrustMag != 0.0f) {
    // Controller stopped publishing: do not leave the last modelled thrust standing (D16).
    actualThrustMag = 0.0f;
    actualTwistAngle = 0.0f;
    tvcActualThrustTopic_.publish({0.0f, 0.0f, 0.0f, 0.0f});
  }
}

void StarshipTVC::taskThread() {
  auto threadStartTime = Core::NowNs();
  // The hard motor gate (T9b-2): one evaluation per run, used for the thrust model and the motor outputs below.
  Safety::MotorGateInput gateIn;
  gateIn.interlockOpen = interlockOpen(threadStartTime);
  gateIn.forcedOff = motorsForcedOff_;
  gateIn.enableMotors = enableMotors_;
  gateIn.enableActuators = enableActuators_;
  gateIn.testIdle = (actuatorTestState_ == ActuatorTestingState::Idle);
  const Safety::MotorGateResult gate = Safety::evaluateMotorGate(gateIn);
  gateOpenLast_ = gateIn.interlockOpen;
  double dTime = (threadStartTime - lastThreadTime_) / double(Core::SECONDS);
  auto tvcSetting = ctrlSubr_.getItem();
  auto tvcTwistForce = tvcSetting(3); // Torque in Z axis (roll torque)
  auto thrustMagnitude = tvcSetting.magnitude(0, 3);
  Math::Vector_F tvcVector = {tvcSetting(0), tvcSetting(1),
                              tvcSetting(2)}; // Force vector in body frame

  // LOG_MSG("TVC setting: %.2f, %.2f, %.2f, %.2f\n", tvcSetting(0),
  // tvcSetting(1), tvcSetting(2), tvcTwistForce);

  bool enableActuators =
      enableActuators_; // copy to local variable so we can enalbe for testing

  const float thrustLimit =
      thrustCap_N_ < tvcThrustLimit_N_ ? thrustCap_N_ : tvcThrustLimit_N_; // phase cap (bellyflop kick)
  if (thrustMagnitude > thrustLimit) {
    thrustMagnitude = thrustLimit; // Limit the thrust magnitude to the thrust limit
    tvcVector = tvcVector.normalize() * thrustLimit; // Scale the vector to the thrust limit
  }

  auto tvcAngle = tvcVector.getAngleTo(Math::Vector<float, 3>({0, 0, 1}));
  auto tvcRotationAxis = (tvcVector.cross(Math::Vector<float, 3>({0, 0, 1})))
                             .normalize(); // Rotation axis is the cross product
                                           // of the vector and the Z-Axis.
  if (tvcAngle > tvcAngleLimit_Rad_) {
    auto tvcRotation =
        Math::Quat_F(tvcRotationAxis, tvcAngle - tvcAngleLimit_Rad_)
            .conjugate();
    tvcVector = tvcRotation.conjugate().rotate(
        tvcVector); // Rotate the vector back to the limit.
  }

  // LOG_MSG("TVC vector: %.2f, %.2f, %.2f |%.1f| Enabled: %s\n", tvcVector(0),
  // tvcVector(1), tvcVector(2), tvcVector.magnitude(), enableActuators ? "true"
  // : "false");

  float twistFactor = 45;
  float twistLimit = 0.5;

  tvcTwistForce = tvcTwistForce * twistFactor / thrustMagnitude;
  if (thrustMagnitude < 0.01) {
    tvcTwistForce = 0;
  }

  // Keep the twist force from saturating the fins
  if (tvcTwistForce > twistLimit)
    tvcTwistForce = twistLimit;
  else if (tvcTwistForce < -twistLimit)
    tvcTwistForce = -twistLimit;

  // Calculate the fin angles. DO NOT use the tvcVector as this is angle limited
  // to simulate the factor applied to the fins.
  auto xAngle =
      atan2(tvcSetting(1), tvcSetting(2)); // angle between vector and Z axis
                                           // with the X axis as rotation axis
  auto yAngle =
      atan2(tvcSetting(0), tvcSetting(2)); // angle between vector and Z axis
                                           // with the Y axis as rotation axis

  if (xAngle > tvcAngleLimit_Rad_)
    xAngle = tvcAngleLimit_Rad_;
  else if (xAngle < -tvcAngleLimit_Rad_)
    xAngle = -tvcAngleLimit_Rad_;

  if (yAngle > tvcAngleLimit_Rad_)
    yAngle = tvcAngleLimit_Rad_;
  else if (yAngle < -tvcAngleLimit_Rad_)
    yAngle = -tvcAngleLimit_Rad_;

  xAngle *= tvcFactor_;
  yAngle *= tvcFactor_;

  if (actuatorTestState_ !=
      ActuatorTestingState::Idle) { // Hijack the control loop to test the
                                    // actuators

    // LOG_MSG("Actuator test state\n");

    enableActuators = true;

    if (Core::NowNs() - actuatorTestStartTime_ > 0) {
      actuatorTestState_ = ActuatorTestingState::Testing;
    }

    if (Core::NowNs() - actuatorTestStartTime_ <
        TVC_TEST_SPIN_TIME * Core::SECONDS) {
      // LOG_MSG("Actuator test spin\n");

      float angleSin =
          sin(double(Core::NowNs() - actuatorTestStartTime_) / Core::SECONDS /
              TVC_TEST_SPIN_TIME * 2 * M_PI * TVC_TEST_SPIN_COUNT) *
          servoAngleLimit_Rad_;
      float angleCos =
          cos(double(Core::NowNs() - actuatorTestStartTime_) / Core::SECONDS /
              TVC_TEST_SPIN_TIME * 2 * M_PI * TVC_TEST_SPIN_COUNT) *
          servoAngleLimit_Rad_;
      xAngle = angleSin;
      yAngle = angleCos;

    } else if (Core::NowNs() - actuatorTestStartTime_ <
               (TVC_TEST_TWIST_TIME + TVC_TEST_SPIN_TIME) * Core::SECONDS) {
      // LOG_MSG("Actuator test twist\n");

      xAngle = 0;
      yAngle = 0;
      tvcTwistForce =
          sin(double(Core::NowNs() - actuatorTestStartTime_) / Core::SECONDS /
              TVC_TEST_TWIST_TIME * 2 * M_PI * TVC_TEST_TWIST_COUNT);

    } else {
      actuatorTestState_ = ActuatorTestingState::Idle;
    }
  }

  // enableActuators = true;
  // xAngle = 0;
  // yAngle = 0;

  if (xAngle > servoAngleLimit_Rad_)
    xAngle = servoAngleLimit_Rad_;
  else if (xAngle < -servoAngleLimit_Rad_)
    xAngle = -servoAngleLimit_Rad_;

  if (yAngle > servoAngleLimit_Rad_)
    yAngle = servoAngleLimit_Rad_;
  else if (yAngle < -servoAngleLimit_Rad_)
    yAngle = -servoAngleLimit_Rad_;

  auto servoXPOut =
      (xAngle + tvcFinOffsetXP_Rad_) / servoAngleLimit_Rad_ + tvcTwistForce;
  auto servoXNOut =
      -(xAngle - tvcFinOffsetXN_Rad_) / servoAngleLimit_Rad_ + tvcTwistForce;
  auto servoYPOut =
      (yAngle + tvcFinOffsetYP_Rad_) / servoAngleLimit_Rad_ + tvcTwistForce;
  auto servoYNOut =
      -(yAngle - tvcFinOffsetYN_Rad_) / servoAngleLimit_Rad_ + tvcTwistForce;

  auto motorCWOut = thrustMagnitude / tvcThrustLimit_N_ * motorPowerLimit_;
  auto motorCCWOut = thrustMagnitude / tvcThrustLimit_N_ * motorPowerLimit_;

  if (motorCWOut > motorPowerLimit_)
    motorCWOut = motorPowerLimit_;
  if (motorCCWOut > motorPowerLimit_)
    motorCCWOut = motorPowerLimit_;

  // Lets update the actual thrust vector with what we expect to get from the
  // motors and servos. This is used for telemetry and simulation purposes.
  Math::Vector<float, 4> actualThrustVector = 0;
  if (gate.thrustModelEnabled) { // D16: no modelled thrust while the gate is closed, as hardware loses thrust
    auto thrustVector = tvcVector.normalize() * (motorCWOut + motorCCWOut) / 2 *
                        tvcThrustLimit_N_;

    // Slew limit of the modelled thrust angle = vane servo rate / servo-to-thrust gain (the servos move tvcFactor_
    // times the thrust angle; review F6: the limit was 600 deg/s in thrust-angle space = 9000 deg/s of servo).
    const float angleRateLimit = TVC_SERVO_RATE_RADPS / tvcFactor_;
    float angleXAxis = atan2(thrustVector(1), thrustVector(2));
    float angleYAxis = atan2(thrustVector(0), thrustVector(2));
    float thrustMag = thrustVector.magnitude();
    if (!(isfinite(angleXAxis) && isfinite(angleYAxis) && isfinite(thrustMag))) {
      // A non-finite TVC command must not latch NaN into the integrated actual angles (it stayed NaN until
      // reboot): hold the previous angles and report no thrust for this tick.
      angleXAxis = actualThrustXAngle;
      angleYAxis = actualThrustYAngle;
      thrustMag = 0.0f;
    }
    float angleXDiff = angleXAxis - actualThrustXAngle;
    float angleYDiff = angleYAxis - actualThrustYAngle;
    if (angleXDiff > angleRateLimit * dTime)
      angleXDiff = angleRateLimit * dTime;
    else if (angleXDiff < -angleRateLimit * dTime)
      angleXDiff = -angleRateLimit * dTime;
    if (angleYDiff > angleRateLimit * dTime)
      angleYDiff = angleRateLimit * dTime;
    else if (angleYDiff < -angleRateLimit * dTime)
      angleYDiff = -angleRateLimit * dTime;
    actualThrustXAngle += angleXDiff;
    actualThrustYAngle += angleYDiff;
    actualThrustMag = thrustMag;
    actualTwistAngle = tvcTwistForce * thrustMagnitude / twistFactor;
    if (!isfinite(actualTwistAngle))
      actualTwistAngle = 0.0f;
  } else {
    actualThrustMag = 0;
    actualTwistAngle = 0; // no stale roll torque after the actuators were disabled (review F13)
  }
  tvcActualThrustTopic_.publish({
      sin(actualThrustYAngle) * actualThrustMag,                           //
      sin(actualThrustXAngle) * actualThrustMag,                           //
      cos(actualThrustXAngle) * cos(actualThrustYAngle) * actualThrustMag, //
      actualTwistAngle                                                     //
  });
  servoXP_.enableOutput(enableActuators);
  servoXN_.enableOutput(enableActuators);
  servoYP_.enableOutput(enableActuators);
  servoYN_.enableOutput(enableActuators);

  motorCW_.enableOutput(gate.outputsEnabled);
  motorCCW_.enableOutput(gate.outputsEnabled);

  // LOG_MSG("Motor CW: %.2f, Motor CCW: %.2f\n", motorCWOut, motorCCWOut);

  if (enableActuators) {
    servoXP_.setValue(servoXPOut * 0.5 + 0.5);
    servoXN_.setValue(servoXNOut * 0.5 + 0.5);
    servoYP_.setValue(servoYPOut * 0.5 + 0.5);
    servoYN_.setValue(servoYNOut * 0.5 + 0.5);
  }

  if (gate.outputsEnabled) { // a closed gate never calls setValue() (PWM_Output would drive the pin again)
    // LOG_MSG("Motor CW: %.2f, Motor CCW: %.2f\n", motorCWOut, motorCCWOut);

    // LOG_MSG("TVC: %.2f, %.2f, %.2f, %.2f\n", tvcSetting(0), tvcSetting(1),
    // tvcSetting(2), tvcSetting(3));

    if (Core::NowNs() - motorEnableTime_ > 2000 * Core::MILLISECONDS ||
        !firstMotorEnable_) {
      // LOG_MSG("Motor enable time: %lld\n", Core::NowNs() - motorEnableTime_);
      firstMotorEnable_ = false;
      float motorIdle = 0.04;
      motorCW_.setValue(motorCWOut + motorIdle);
      motorCCW_.setValue(motorCCWOut + motorIdle);
    } else {
      motorCW_.setValue(0.0);
      motorCCW_.setValue(0.0);
    }

    // motorCW_.setValue(motorCWOut + 0.1);
    // motorCCW_.setValue(motorCCWOut + 0.1);

  } else {
    motorEnableTime_ =
        Core::NowNs(); // Reset the motor enable time to now, so that the motors
                     // are not enabled again until the next control loop.
  }

  lastThreadTime_ = threadStartTime;
}

} // namespace CTRL

} // namespace VCTR
