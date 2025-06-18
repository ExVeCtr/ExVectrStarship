#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrActuator/pwm_output.hpp"

#include "ExVectrCore/print.hpp"

#include "starship_tvc.hpp"


namespace VCTR
{
    namespace CTRL
    {

        
        StarshipTVC::StarshipTVC(HAL::PinPWM& tvcServoPinXP, HAL::PinPWM& tvcServoPinXN, HAL::PinPWM& tvcServoPinYP, HAL::PinPWM& tvcServoPinYN, HAL::PinPWM& motorCW, HAL::PinPWM& motorCCW, float tvcAngleLimit_Rad, float servoAngleLimit_Rad, float tvcThrustLimit_N) :
            Core::Task_Periodic("Starship TVC", 20*Core::MILLISECONDS),
            servoXP_(tvcServoPinXP, ACTR::PWM_Output_Protocol::STANDARD),
            servoXN_(tvcServoPinXN, ACTR::PWM_Output_Protocol::STANDARD),
            servoYP_(tvcServoPinYP, ACTR::PWM_Output_Protocol::STANDARD),
            servoYN_(tvcServoPinYN, ACTR::PWM_Output_Protocol::STANDARD),
            motorCW_(motorCW, ACTR::PWM_Output_Protocol::STANDARD),
            motorCCW_(motorCCW, ACTR::PWM_Output_Protocol::STANDARD)
        {
            Core::getSystemScheduler().addTask(*this); // Attach to the scheduler
            setRelease(Core::END_OF_TIME);
            tvcAngleLimit_Rad_ = tvcAngleLimit_Rad;
            servoAngleLimit_Rad_ = servoAngleLimit_Rad;
            tvcThrustLimit_N_ = tvcThrustLimit_N;
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
            actuatorTestStartTime_ = Core::NOW() + testOffset;
            actuatorTestState_ = ActuatorTestingState::WaitBegin;
            //enableActuators_ = true;
            LOG_MSG("TVC test begin\n");
        }

        bool StarshipTVC::testingActuators() {
            return actuatorTestState_ != ActuatorTestingState::Idle;
        }

        void StarshipTVC::taskThread()
        {
            

            auto tvcSetting = ctrlSubr_.getItem();
            auto tvcTwistForce = tvcSetting(3); //Torque in Z axis (roll torque)
            auto thrustMagnitude = tvcSetting.magnitude(0, 3);
            Math::Vector_F tvcVector = {tvcSetting(0), tvcSetting(1), tvcSetting(2)}; //Force vector in body frame

            //LOG_MSG("TVC setting: %.2f, %.2f, %.2f, %.2f\n", tvcSetting(0), tvcSetting(1), tvcSetting(2), tvcTwistForce);

            bool enableActuators = enableActuators_; //copy to local variable so we can enalbe for testing

            if (thrustMagnitude > tvcThrustLimit_N_) {
                thrustMagnitude = tvcThrustLimit_N_; //Limit the thrust magnitude to the thrust limit
                tvcVector = tvcVector.normalize() * tvcThrustLimit_N_; //Scale the vector to the thrust limit
            }

            auto tvcAngle = tvcVector.getAngleTo(Math::Vector<float, 3>({0, 0, 1}));
            auto tvcRotationAxis = (tvcVector.cross(Math::Vector<float, 3>({0, 0, 1}))).normalize(); //Rotation axis is the cross product of the vector and the Z-Axis.
            if (tvcAngle > tvcAngleLimit_Rad_) {
                auto tvcRotation = Math::Quat_F(tvcRotationAxis, tvcAngle - tvcAngleLimit_Rad_).conjugate();
                tvcVector = tvcRotation.rotate(tvcVector); //Rotate the vector back to the limit.
            }

            //LOG_MSG("TVC vector: %.2f, %.2f, %.2f |%.1f| Enabled: %s\n", tvcVector(0), tvcVector(1), tvcVector(2), tvcVector.magnitude(), enableActuators ? "true" : "false");

            float angleFactor = 19;
            float twistFactor = 45;
            float twistLimit = 0.5;

            tvcTwistForce = tvcTwistForce * twistFactor / thrustMagnitude;
            if (thrustMagnitude < 0.01) {
                tvcTwistForce = 0;
            } 
            
            // Keep the twist force from saturating the fins
            if (tvcTwistForce > twistLimit) tvcTwistForce = twistLimit;
            else if (tvcTwistForce < -twistLimit) tvcTwistForce = -twistLimit;

            // Calculate the fin angles. DO NOT use the tvcVector as this is angle limited to simulate the factor applied to the fins.
            auto xAngle = atan2(tvcSetting(1), tvcSetting(2)); //angle between vector and Z axis with the X axis as rotation axis
            auto yAngle = atan2(tvcSetting(0), tvcSetting(2)); //angle between vector and Z axis with the Y axis as rotation axis

            if (xAngle > tvcAngleLimit_Rad_) xAngle = tvcAngleLimit_Rad_;
            else if (xAngle < -tvcAngleLimit_Rad_) xAngle = -tvcAngleLimit_Rad_;

            if (yAngle > tvcAngleLimit_Rad_) yAngle = tvcAngleLimit_Rad_;
            else if (yAngle < -tvcAngleLimit_Rad_) yAngle = -tvcAngleLimit_Rad_;

            xAngle *= angleFactor;
            yAngle *= angleFactor;

            if (actuatorTestState_ != ActuatorTestingState::Idle) { //Hijack the control loop to test the actuators

                //LOG_MSG("Actuator test state\n");

                enableActuators = true;

                if (Core::NOW() - actuatorTestStartTime_ > 0) {
                    actuatorTestState_ = ActuatorTestingState::Testing;
                }

                if (Core::NOW() - actuatorTestStartTime_ < TVC_TEST_SPIN_TIME * Core::SECONDS) {

                    //LOG_MSG("Actuator test spin\n");
                    
                    float angleSin = sin(double(Core::NOW() - actuatorTestStartTime_) / Core::SECONDS / TVC_TEST_SPIN_TIME * 2 * M_PI * TVC_TEST_SPIN_COUNT) * servoAngleLimit_Rad_;
                    float angleCos = cos(double(Core::NOW() - actuatorTestStartTime_) / Core::SECONDS / TVC_TEST_SPIN_TIME * 2 * M_PI * TVC_TEST_SPIN_COUNT) * servoAngleLimit_Rad_;
                    xAngle = angleSin;
                    yAngle = angleCos;

                } else if (Core::NOW() - actuatorTestStartTime_ < (TVC_TEST_TWIST_TIME + TVC_TEST_SPIN_TIME) * Core::SECONDS) {

                    //LOG_MSG("Actuator test twist\n");

                    xAngle = 0;
                    yAngle = 0;
                    tvcTwistForce = sin(double(Core::NOW() - actuatorTestStartTime_) / Core::SECONDS / TVC_TEST_TWIST_TIME * 2 * M_PI * TVC_TEST_TWIST_COUNT);

                } else {
                    actuatorTestState_ = ActuatorTestingState::Idle;
                }

                
            } 

            //enableActuators = true;
            //xAngle = 0;
            //yAngle = 0;

            if (xAngle > servoAngleLimit_Rad_) xAngle = servoAngleLimit_Rad_;
            else if (xAngle < -servoAngleLimit_Rad_) xAngle = -servoAngleLimit_Rad_;

            if (yAngle > servoAngleLimit_Rad_) yAngle = servoAngleLimit_Rad_;
            else if (yAngle < -servoAngleLimit_Rad_) yAngle = -servoAngleLimit_Rad_;

            auto servoXPOut = (xAngle + tvcFinOffsetXP_Rad_) / servoAngleLimit_Rad_ + tvcTwistForce;
            auto servoXNOut = -(xAngle - tvcFinOffsetXN_Rad_) / servoAngleLimit_Rad_ + tvcTwistForce;
            auto servoYPOut = (yAngle + tvcFinOffsetYP_Rad_) / servoAngleLimit_Rad_ + tvcTwistForce;
            auto servoYNOut = -(yAngle - tvcFinOffsetYN_Rad_) / servoAngleLimit_Rad_ + tvcTwistForce;

            auto motorCWOut = thrustMagnitude / tvcThrustLimit_N_ * motorPowerLimit_;
            auto motorCCWOut = thrustMagnitude / tvcThrustLimit_N_ * motorPowerLimit_;

            if (motorCWOut > motorPowerLimit_) motorCWOut = motorPowerLimit_;
            if (motorCCWOut > motorPowerLimit_) motorCCWOut = motorPowerLimit_;
            
            // Lets update the actual thrust vector with what we expect to get from the motors and servos. This is used for telemetry and simulation purposes.
            Math::Vector<float, 4> tvcActualThrustVector = 0;
            if (enableActuators_ && actuatorTestState_ == ActuatorTestingState::Idle){ // Assume disarm if the motors arte
                auto actualThrustVector = tvcVector.normalize() * (motorCWOut + motorCCWOut)/2 * tvcThrustLimit_N_; 
                tvcActualThrustVector = {
                    actualThrustVector(0),
                    actualThrustVector(1),
                    actualThrustVector(2),
                    tvcTwistForce * thrustMagnitude / twistFactor
                };
            }
            tvcActualThrustTopic_.publish(tvcActualThrustVector); // Publish the actual thrust vector

            servoXP_.enableOutput(enableActuators);
            servoXN_.enableOutput(enableActuators);
            servoYP_.enableOutput(enableActuators);
            servoYN_.enableOutput(enableActuators);

            motorCW_.enableOutput(enableMotors_ && enableActuators_ && actuatorTestState_ == ActuatorTestingState::Idle);
            motorCCW_.enableOutput(enableMotors_ && enableActuators_ && actuatorTestState_ == ActuatorTestingState::Idle);

            //LOG_MSG("Motor CW: %.2f, Motor CCW: %.2f\n", motorCWOut, motorCCWOut);

            if (enableActuators) {

                servoXP_.setValue(servoXPOut * 0.5 + 0.5);
                servoXN_.setValue(servoXNOut * 0.5 + 0.5);
                servoYP_.setValue(servoYPOut * 0.5 + 0.5);
                servoYN_.setValue(servoYNOut * 0.5 + 0.5);

            }

            if (enableMotors_ && enableActuators_ && actuatorTestState_ == ActuatorTestingState::Idle) {

                //LOG_MSG("Motor CW: %.2f, Motor CCW: %.2f\n", motorCWOut, motorCCWOut);

                //LOG_MSG("TVC: %.2f, %.2f, %.2f, %.2f\n", tvcSetting(0), tvcSetting(1), tvcSetting(2), tvcSetting(3));

                if (Core::NOW() - motorEnableTime_ > 2000 * Core::MILLISECONDS || !firstMotorEnable_) {
                    //LOG_MSG("Motor enable time: %lld\n", Core::NOW() - motorEnableTime_);
                    firstMotorEnable_ = false;
                    float motorIdle = 0.04;
                    motorCW_.setValue(motorCWOut + motorIdle);
                    motorCCW_.setValue(motorCCWOut + motorIdle);
                } else {
                    motorCW_.setValue(0.0);
                    motorCCW_.setValue(0.0);
                }

                //motorCW_.setValue(motorCWOut + 0.1);
                //motorCCW_.setValue(motorCCWOut + 0.1);

            } else {
                motorEnableTime_ = Core::NOW(); // Reset the motor enable time to now, so that the motors are not enabled again until the next control loop.
            }



        }

    }

}

