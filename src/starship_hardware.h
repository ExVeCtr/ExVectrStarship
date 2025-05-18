#ifndef STARSHIP_HARDWARE_H
#define STARSHIP_HARDWARE_H

/**
 * 
 * This defines all connection pins of servos, motors etc and also parameters like max TVC thrust and angle.
 * 
*/

#define TVC_SERVO_PIN_1 1
#define TVC_SERVO_PIN_2 22
#define TVC_SERVO_PIN_3 0
#define TVC_SERVO_PIN_4 23

#define FLAP_SERVO_PIN_UL 25
#define FLAP_SERVO_PIN_UR 24
#define FLAP_SERVO_PIN_DL 28
#define FLAP_SERVO_PIN_DR 29

#define MOTOR_PIN_CW 8
#define MOTOR_PIN_CCW 7

#define NEO_M8Q_SERIALPORT Serial5

#define VEHICLE_MASS_KG 1.05f // Mass of the vehicle in kg
#define TVC_THRUST_LIMIT_N 15.0f // Maximum thrust in Newtons
#define TVC_ANGLE_LIMIT_RAD 8*3.14/180 // Maximum angle in radians



#endif