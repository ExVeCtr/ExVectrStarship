#ifndef EXVECTRSTARSHIP_STARSHIPFLAPS_HPP
#define EXVECTRSTARSHIP_STARSHIPFLAPS_HPP

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrHAL/pin_pwm.hpp"
#include "ExVectrActuator/pwm_output.hpp"
#include "ExVectrActuator/servo_control.hpp"


namespace VCTR
{
    namespace CTRL
    {

        struct StarshipFlapSettings
        {
            float tlAngle; // Angle of the top left flap in radians
            float trAngle; // Angle of the top right flap in radians
            float blAngle; // Angle of the bottom left flap in radians
            float brAngle; // Angle of the bottom right flap in radians
            bool enableActuators; // Enable or disable the actuators
        };

        /**
         * @brief Simple one dimensional PID controller.
         */
        class StarshipFlaps : public Core::Task_Periodic
        {
        private:

            Core::Simple_Subscriber<StarshipFlapSettings> ctrlSubr_;

            ACTR::PWM_Output servoTLPin_;
            ACTR::PWM_Output servoTRPin_;
            ACTR::PWM_Output servoBLPin_;
            ACTR::PWM_Output servoBRPin_;

            StarshipFlapSettings flapSettings_;

            bool enableActuators_ = true;


        public:
            
            /**
             * @brief Constructs a new StarshipFlaps object.
             */
            StarshipFlaps(HAL::PinPWM& flapServoULPin, HAL::PinPWM& flapServoURPin, HAL::PinPWM& flapServoLLPin, HAL::PinPWM& flapServoLRPin);


            void setFlapSettingTopic(Core::Topic<StarshipFlapSettings>& flapSettingTopic)
            {
                ctrlSubr_.subscribe(flapSettingTopic);
            }

            void enableActuators(bool enable) { enableActuators_ = enable; }

            
            //void taskCheck() override;

            void taskInit() override;

            void taskThread() override;

            
        };

    }
}

#endif // EXVECTRCONTROL_SIMPLE_PID_HPP_