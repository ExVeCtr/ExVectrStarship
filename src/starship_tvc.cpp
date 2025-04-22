#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrActuator/pwm_output.hpp"

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
            motorCW_(motorCW, ACTR::PWM_Output_Protocol::ONESHOT125),
            motorCCW_(motorCCW, ACTR::PWM_Output_Protocol::ONESHOT125)
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

            if (enableActuators_) {

                servoXP_.enableOutput(true);
                servoXN_.enableOutput(true);
                servoYP_.enableOutput(true);
                servoYN_.enableOutput(true);

                motorCW_.enableOutput(true);
                motorCCW_.enableOutput(true);

                auto tvcSetting = ctrlSubr_.getItem();
                auto tvcTwistForce = tvcSetting(3); //Torque in Z axis (roll torque)
                auto thrustMagnitude = tvcSetting.magnitude(0, 3);

                auto xAngle = atan2(tvcSetting(1), tvcSetting(2)); //angle between vector and Z axis with the X axis as rotation axis
                auto yAngle = atan2(tvcSetting(0), tvcSetting(2)); //angle between vector and Z axis with the Y axis as rotation axis

                if (xAngle > tvcAngleLimit_Rad_) xAngle = tvcAngleLimit_Rad_;
                else if (xAngle < -tvcAngleLimit_Rad_) xAngle = -tvcAngleLimit_Rad_;

                if (yAngle > tvcAngleLimit_Rad_) yAngle = tvcAngleLimit_Rad_;
                else if (yAngle < -tvcAngleLimit_Rad_) yAngle = -tvcAngleLimit_Rad_;

                auto servoXPOut = (xAngle + tvcFinOffsetXP_Rad_) / tvcAngleLimit_Rad_ + tvcTwistForce;
                auto servoXNOut = -(xAngle - tvcFinOffsetXN_Rad_) / tvcAngleLimit_Rad_ + tvcTwistForce;
                auto servoYPOut = (yAngle + tvcFinOffsetYP_Rad_) / tvcAngleLimit_Rad_ + tvcTwistForce;
                auto servoYNOut = -(yAngle - tvcFinOffsetYN_Rad_) / tvcAngleLimit_Rad_ + tvcTwistForce;

                auto motorCWOut = thrustMagnitude / tvcThrustLimit_N_;
                auto motorCCWOut = thrustMagnitude / tvcThrustLimit_N_;

                servoXP_.setValue(servoXPOut * 0.5 + 0.5);
                servoXN_.setValue(servoXNOut * 0.5 + 0.5);
                servoYP_.setValue(servoYPOut * 0.5 + 0.5);
                servoYN_.setValue(servoYNOut * 0.5 + 0.5);

                //servoXP_.setValue(sin(Core::NOWSeconds()) * 0.5 + 0.5);

                motorCW_.setValue(motorCWOut);
                motorCCW_.setValue(motorCCWOut);


            } else {

                servoXP_.enableOutput(false);
                servoXN_.enableOutput(false);
                servoYP_.enableOutput(false);
                servoYN_.enableOutput(false);

                motorCW_.enableOutput(false);
                motorCCW_.enableOutput(false);

            }

        }

    }
}

