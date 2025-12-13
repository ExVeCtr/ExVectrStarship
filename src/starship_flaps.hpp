#ifndef EXVECTRSTARSHIP_STARSHIPFLAPS_HPP
#define EXVECTRSTARSHIP_STARSHIPFLAPS_HPP

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"

#include "ExVectrDSP/value_covariance.hpp"
#include "ExVectrControl/starship/control_attitude_flaps.hpp"

#include "ExVectrHAL/pin_pwm.hpp"
#include "ExVectrActuator/pwm_output.hpp"
#include "ExVectrActuator/servo_control.hpp"

namespace VCTR
{
    namespace CTRL
    {

        /**
         * @brief Simple one dimensional PID controller.
         */
        class StarshipFlaps : public Core::Task_Periodic
        {
        private:
            const float FLAP_TEST_DURATION = 4.0f;
            const float FLAP_MAX_ANGLE = 90 * DEGREES;
            const float FLAP_MIN_ANGLE = 15 * DEGREES;

            Core::Simple_Subscriber<CTRL::ControlAttitudeFlapSetting> ctrlSubr_;

            ACTR::PWM_Output servoTLPin_;
            ACTR::PWM_Output servoTRPin_;
            ACTR::PWM_Output servoBLPin_;
            ACTR::PWM_Output servoBRPin_;

            CTRL::ControlAttitudeFlapSetting flapSettings_;

            bool enableActuators_ = true;

            enum class ActuatorTestingState
            {
                WaitBegin,
                Testing,
                Idle
            };
            ActuatorTestingState actuatorTestState_ = ActuatorTestingState::Idle;

            int64_t actuatorTestStartTime_ = 0;

        public:
            /**
             * @brief Constructs a new StarshipFlaps object.
             */
            StarshipFlaps(HAL::PinPWM &flapServoULPin, HAL::PinPWM &flapServoURPin, HAL::PinPWM &flapServoLLPin, HAL::PinPWM &flapServoLRPin);

            void setFlapSettingTopic(Core::Topic<CTRL::ControlAttitudeFlapSetting> &flapSettingTopic)
            {
                ctrlSubr_.subscribe(flapSettingTopic);
            }

            const CTRL::ControlAttitudeFlapSetting &getFlapSettings() const { return flapSettings_; }

            void enableActuators(bool enable) { enableActuators_ = enable; }

            // void taskCheck() override;

            void taskInit() override;

            void taskThread() override;

            void beginActuatorTest(int64_t testOffset);

            bool testingActuators();

        private:
            void limitAngle(float &angle) const
            {
                if (angle > FLAP_MAX_ANGLE)
                {
                    angle = FLAP_MAX_ANGLE;
                }
                else if (angle < FLAP_MIN_ANGLE)
                {
                    angle = FLAP_MIN_ANGLE;
                }
            }
        };

    }
}

#endif // EXVECTRCONTROL_SIMPLE_PID_HPP_