#ifndef EXVECTRSTARSHIP_STARSHIPTVC_HPP
#define EXVECTRSTARSHIP_STARSHIPTVC_HPP

#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrMath/matrix_vector.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrHAL/pin_pwm.hpp"
#include "ExVectrActuator/pwm_output.hpp"


namespace VCTR
{
    namespace CTRL
    {   

        /**
         * @brief Takes the TVC settings from rocket control and maps them to the servos and motors.
         */
        class StarshipTVC : public Core::Task_Periodic
        {
        private:

            const float TVC_TEST_SPIN_TIME = 2;
            const int64_t TVC_TEST_SPIN_COUNT = 2;
            const float TVC_TEST_TWIST_TIME = 2;
            const int64_t TVC_TEST_TWIST_COUNT = 2;

            Core::Simple_Subscriber<Math::Vector<float, 4>> ctrlSubr_;

            ACTR::PWM_Output servoXP_;
            ACTR::PWM_Output servoXN_;
            ACTR::PWM_Output servoYP_;
            ACTR::PWM_Output servoYN_;

            ACTR::PWM_Output motorCW_;
            ACTR::PWM_Output motorCCW_;

            bool enableActuators_ = false;
            bool enableMotors_ = false;

            float motorPowerLimit_ = 0.05;

            int64_t motorEnableTime_ = 0;

            float tvcAngleLimit_Rad_;
            float servoAngleLimit_Rad_;
            float tvcThrustLimit_N_;

            float tvcFinOffsetXP_Rad_ = 0.0f;
            float tvcFinOffsetXN_Rad_ = 0.0f;
            float tvcFinOffsetYP_Rad_ = 0.0f;
            float tvcFinOffsetYN_Rad_ = 0.0f;

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
             * @brief Constructs a new StarshipTVC object.
             * @param tvcServoPinXP Pin number for the servo controlling the fin in the positive X direction.
             * @param tvcServoPinXN Pin number for the servo controlling the fin in the negative X direction.
             * @param tvcServoPinYP Pin number for the servo controlling the fin in the positive Y direction.
             * @param tvcServoPinYN Pin number for the servo controlling the fin in the negative Y direction.
             * @param tvcAngleLimit_Rad The theoretical maximum angle in radians for the TVC. Basically if this angle is commanded, then the servos will be at their maximum angle.
             * @param servoAngleLimit_Rad The max angle the servo will reach if commanded to the maximum angle.
             * @param tvcThrustLimit_N The maximum thrust in Newtons for the TVC. This is the maximum thrust that can be commanded to the motors.
             */
            StarshipTVC(HAL::PinPWM& tvcServoPinXP, HAL::PinPWM& tvcServoPinXN, HAL::PinPWM& tvcServoPinYP, HAL::PinPWM& tvcServoPinYN, HAL::PinPWM& motorCW, HAL::PinPWM& motorCCW, float tvcAngleLimit_Rad = 15*3.14/180, float servoAngleLimit_Rad = 45*3.14/180, float tvcThrustLimit_N = 20.0f);


            void setTVCInputTopic(Core::Topic<Math::Vector<float, 4>> &tvcInputTopic) { ctrlSubr_.subscribe(tvcInputTopic); }

            void setTVCFinsOffset(float finOffsetXP_Rad, float finOffsetXN_Rad, float finOffsetYP_Rad, float finOffsetYN_Rad) {
                tvcFinOffsetXP_Rad_ = finOffsetXP_Rad;
                tvcFinOffsetXN_Rad_ = finOffsetXN_Rad;
                tvcFinOffsetYP_Rad_ = finOffsetYP_Rad;
                tvcFinOffsetYN_Rad_ = finOffsetYN_Rad;
            }

            void enableActuators(bool enableActuators) {
                enableActuators_ = enableActuators;
            }

            void motorPowerLimit(float limit) {
                motorPowerLimit_ = limit;
            }

            void enableMotors(bool enable) { enableMotors_ = enable; }
            
            void taskCheck() override;

            void taskInit() override;

            void taskThread() override;

            void beginActuatorTest(int64_t testOffset);

            bool testingActuators();

            
        };

    }
}

#endif // EXVECTRCONTROL_SIMPLE_PID_HPP_