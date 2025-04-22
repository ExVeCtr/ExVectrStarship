#ifndef STARSHIP_CONNECTIONS_V_1_0_H
#define STARSHIP_CONNECTIONS_V_1_0_H

/**
 * 
 * This defines all connection pins of servos, motors, sensors etc.
 * This could of course be different from vehicle to vehicle but IMU 
 * and LoRa radios could be the some depending on the board used.
 * 
*/

//#include "boards/board_v_1_0.h" //Add board used by Starship


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



#endif