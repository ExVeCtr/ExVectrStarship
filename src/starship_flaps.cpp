#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrHAL/pin_pwm.hpp"
#include "ExVectrActuator/pwm_output.hpp"
#include "ExVectrActuator/servo_control.hpp"

#include "starship_flaps.hpp"


namespace VCTR
{
    namespace CTRL
    {

        StarshipFlaps::StarshipFlaps(HAL::PinPWM& flapServoULPin, HAL::PinPWM& flapServoURPin, HAL::PinPWM& flapServoLLPin, HAL::PinPWM& flapServoLRPin) :
            Core::Task_Periodic("Starship Flaps", 20*Core::MILLISECONDS),
            servoTLPin_(flapServoULPin, ACTR::PWM_Output_Protocol::STANDARD),
            servoTRPin_(flapServoURPin, ACTR::PWM_Output_Protocol::STANDARD),
            servoBLPin_(flapServoLLPin, ACTR::PWM_Output_Protocol::STANDARD),
            servoBRPin_(flapServoLRPin, ACTR::PWM_Output_Protocol::STANDARD)
        {
            Core::getSystemScheduler().addTask(*this); // Attach to the scheduler
        }



        void StarshipFlaps::taskInit(){

            servoTLPin_.enableOutput(false);
            servoTRPin_.enableOutput(false);
            servoBLPin_.enableOutput(false);
            servoBRPin_.enableOutput(false);

            flapSettings_.tlAngle = 0.0f; // Initialize the top left flap angle to 0 radians
            flapSettings_.trAngle = 0.0f; // Initialize the top right flap angle to 0 radians
            flapSettings_.blAngle = 0.0f; // Initialize the bottom left flap angle to 0 radians
            flapSettings_.brAngle = 0.0f; // Initialize the bottom right flap angle to 0 radians
            flapSettings_.enableActuators = false; // Initialize the actuator enable flag to false

        }

        void StarshipFlaps::taskThread() {

            if (ctrlSubr_.isDataNew()) {
                flapSettings_ = ctrlSubr_.getItem(); // Get the new flap settings from the subscriber
            }

            bool enable = enableActuators_ && flapSettings_.enableActuators; // Check if the actuators should be enabled

            servoTLPin_.enableOutput(enable);
            servoTRPin_.enableOutput(enable);
            servoBLPin_.enableOutput(enable);
            servoBRPin_.enableOutput(enable);

            if (enable) {
                servoTLPin_.setValue(flapSettings_.tlAngle/3.1415/2); // Set the top left flap angle
                servoTRPin_.setValue(flapSettings_.trAngle/3.1415/2); // Set the top right flap angle
                servoBLPin_.setValue(flapSettings_.blAngle/3.1415/2); // Set the bottom left flap angle
                servoBRPin_.setValue(flapSettings_.brAngle/3.1415/2); // Set the bottom right flap angle
            }


        }

    }
}