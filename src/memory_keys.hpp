#ifndef MEMORY_KEYS_HPP
#define MEMORY_KEYS_HPP




#define MEMORY_KEY_GYROCALIB 100 //Gyro calibration data. This is in sensor frame. Use the GYROTRANFORM key to also retrieve the sensor to body transformation matrix.
#define MEMORY_KEY_MAGCALIB 101 //Magnetometer calibration data. This is in sensor frame. Use the MAGTRANFORM key to also retrieve the sensor to body transformation matrix.
#define MEMORY_KEY_ACCCALIB 102 //Accelerometer calibration data. This is in sensor frame. Use the ACCTRANSFORM key to also retrieve the sensor to body transformation matrix.

#define MEMORY_KEY_GYROTRANSFORM 200
#define MEMORY_KEY_MAGTRANSFORM 201
#define MEMORY_KEY_ACCTRANSFORM 202



















#endif // MEMORY_KEYS_HPP