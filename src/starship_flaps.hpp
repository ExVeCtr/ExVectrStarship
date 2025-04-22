#ifndef EXVECTRSTARSHIP_STARSHIPTVC_HPP
#define EXVECTRSTARSHIP_STARSHIPTVC_HPP

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

        /**
         * @brief Simple one dimensional PID controller.
         */
        class StarshipTVC : public Core::Task_Periodic
        {
        private:

            Core::Simple_Subscriber<Math::Vector<float, 4>> ctrlSubr_;

            ACTR::PWM_Output servoTLPin_;
            ACTR::PWM_Output servoTRPin_;
            ACTR::PWM_Output servoBLPin_;
            ACTR::PWM_Output servoBRPin_;


            ACTR::Servo_Control servoTL_;
            ACTR::Servo_Control servoTR_;
            ACTR::Servo_Control servoBL_;
            ACTR::Servo_Control servoBR_;

            bool enableActuators_ = false;

            float tvcAngleLimit_Rad_;
            float servoAngleLimit_Rad_;
            float tvcThrustLimit_N_;


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


            void enableActuators(bool enable) { enableActuators_ = enable; }

            
            void taskCheck() override;

            void taskInit() override;

            void taskThread() override;

            
        };

    }
}

#endif // EXVECTRCONTROL_SIMPLE_PID_HPP_