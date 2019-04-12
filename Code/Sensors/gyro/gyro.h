#ifndef GYRO_H
#define GYRO_H

#include "Arduino.h"

// IMU lib
#include "MPU9250.h"

// Iteration step in milli seconds. Used when integrating gyro's angular position
#define TIME_STEP 20

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 gyro(Wire,0x68);
int status;

// Degree variation in Z axis
float degreeZ = 0;

// Reads gyro's angular speed in Z axis and integrates it to get the robot's current angle variation
void updateGyro();

// Gyro beginning function. Checks if initialization went correctly
void startGyro();

// Prints current angle on serial monitor
void printGyro();

#endif // GYRO_H