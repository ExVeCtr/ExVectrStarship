#ifndef BOARD_V_1_0_H
#define BOARD_V_1_0_H

/**
 * 
 * This defines all pins for a board and can be used to
 * easily change between boards and testing.
 * Board can be changed in definitions.h file by changing
 * the BOARD_VERSION_HEADER definition.
 * 
*/

#include "Arduino.h"

#define RGBLED_PIN                  4

#define MPU9250_INT_PIN             2
#define MPU9250_NCS_PIN             3
#define MPU9250_SPIBUS              SPI

#define BME280_NCS_PIN              6
#define BME280_SPIBUS               SPI

#define QMC5883L_INT_PIN            16

#define INPUT_VOLTAGE_ADC_PIN       26
#define INPUT_VOLTAGE_ADC_FACTOR    1.0f


#endif