#ifndef STARSHIP_HARDWARE_H
#define STARSHIP_HARDWARE_H

/**
 * 
 * This defines all connection pins of servos, motors etc and also parameters like max TVC thrust and angle.
 * 
*/

#define TVC_SERVO_PIN_1 0
#define TVC_SERVO_PIN_2 23
#define TVC_SERVO_PIN_3 1
#define TVC_SERVO_PIN_4 22

#define FLAP_SERVO_PIN_UL 25
#define FLAP_SERVO_PIN_UR 24
#define FLAP_SERVO_PIN_DL 29
#define FLAP_SERVO_PIN_DR 28

#define MOTOR_PIN_CW 8
#define MOTOR_PIN_CCW 7

#define NEO_M8Q_SERIALPORT Serial5

#define TVC_SERVO_LIMIT 45*DEG_TO_RAD // Maximum angle of the TVC servos in radians in one direction (Full movement range is double this amount)
#define TVC_ANGLE_FACTOR 15 // What factor to multiply the TVC angle by to achive actual thrust angle.

#define VEHICLE_MASS_KG 1.1f // Mass of the vehicle in kg
#define TVC_THRUST_LIMIT_N 14.0f // Maximum thrust in Newtons
#define TVC_ANGLE_LIMIT_RAD (TVC_SERVO_LIMIT/TVC_ANGLE_FACTOR) //8*3.14/180 // Maximum angle in radians



#endif