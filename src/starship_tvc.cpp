#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrActuator/pwm_output.hpp"

#include "ExVectrCore/print.hpp"

#include "starship_tvc.hpp"


namespace VCTR
{
    namespace CTRL
    {

        
        StarshipTVC::StarshipTVC(HAL::PinPWM& tvcServoPinXP, HAL::PinPWM& tvcServoPinXN, HAL::PinPWM& tvcServoPinYP, HAL::PinPWM& tvcServoPinYN, HAL::PinPWM& motorCW, HAL::PinPWM& motorCCW, float tvcAngleLimit_Rad, float servoAngleLimit_Rad, float tvcThrustLimit_N) :
            Core::Task_Periodic("Starship TVC", 50*Core::MILLISECONDS),
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

        void StarshipTVC::taskThread()
        {

            auto tvcSetting = ctrlSubr_.getItem();
            auto tvcTwistForce = tvcSetting(3); //Torque in Z axis (roll torque)
            auto thrustMagnitude = tvcSetting.magnitude(0, 3);

            float angleFactor = 5;
            float twistFactor = 45;

            tvcTwistForce = tvcTwistForce * twistFactor / thrustMagnitude;
            if (thrustMagnitude < 0.01) {
                tvcTwistForce = 0;
            } 

            auto xAngle = atan2(tvcSetting(1), tvcSetting(2)) * angleFactor; //angle between vector and Z axis with the X axis as rotation axis
            auto yAngle = atan2(tvcSetting(0), tvcSetting(2)) * angleFactor; //angle between vector and Z axis with the Y axis as rotation axis

            if (xAngle > tvcAngleLimit_Rad_) xAngle = tvcAngleLimit_Rad_;
            else if (xAngle < -tvcAngleLimit_Rad_) xAngle = -tvcAngleLimit_Rad_;

            if (yAngle > tvcAngleLimit_Rad_) yAngle = tvcAngleLimit_Rad_;
            else if (yAngle < -tvcAngleLimit_Rad_) yAngle = -tvcAngleLimit_Rad_;

            auto servoXPOut = (xAngle + tvcFinOffsetXP_Rad_) / tvcAngleLimit_Rad_ + tvcTwistForce;
            auto servoXNOut = -(xAngle - tvcFinOffsetXN_Rad_) / tvcAngleLimit_Rad_ + tvcTwistForce;
            auto servoYPOut = (yAngle + tvcFinOffsetYP_Rad_) / tvcAngleLimit_Rad_ + tvcTwistForce;
            auto servoYNOut = -(yAngle - tvcFinOffsetYN_Rad_) / tvcAngleLimit_Rad_ + tvcTwistForce;

            auto motorCWOut = thrustMagnitude / tvcThrustLimit_N_ * motorPowerLimit_;
            auto motorCCWOut = thrustMagnitude / tvcThrustLimit_N_ * motorPowerLimit_;

            if (motorCWOut > motorPowerLimit_) motorCWOut = motorPowerLimit_;
            if (motorCCWOut > motorPowerLimit_) motorCCWOut = motorPowerLimit_;

            servoXP_.enableOutput(enableActuators_);
            servoXN_.enableOutput(enableActuators_);
            servoYP_.enableOutput(enableActuators_);
            servoYN_.enableOutput(enableActuators_);

            motorCW_.enableOutput(enableMotors_ && enableActuators_);
            motorCCW_.enableOutput(enableMotors_ && enableActuators_);

            //LOG_MSG("Motor CW: %.2f, Motor CCW: %.2f\n", motorCWOut, motorCCWOut);

            if (enableActuators_) {

                servoXP_.setValue(servoXPOut * 0.5 + 0.5);
                servoXN_.setValue(servoXNOut * 0.5 + 0.5);
                servoYP_.setValue(servoYPOut * 0.5 + 0.5);
                servoYN_.setValue(servoYNOut * 0.5 + 0.5);

            }

            if (enableMotors_ && enableActuators_) {

                //LOG_MSG("Motor CW: %.2f, Motor CCW: %.2f\n", motorCWOut, motorCCWOut);

                if (Core::NOW() - motorEnableTime_ > 3000 * Core::MILLISECONDS) {
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

