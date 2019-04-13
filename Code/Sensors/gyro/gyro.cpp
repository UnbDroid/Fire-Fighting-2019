#include "gyro.h"

MPU9250 * gyro = new MPU9250(Wire, 0x68);
float degreeZ = 0;

// Gets gyro's raw readings (rad/s) and integrates them into angles (rad).
// Angles are also converted from rad to degrees. 
void updateGyro() {
  gyro->readSensor();
  degreeZ += (gyro->getGyroZ_rads() * TIME_STEP * 180) / (1000 * PI);
}

// Gyro setup function
void startGyro() {
  gyro->begin();
}

// Prints current angle on serial monitor
void printGyro() {
    updateGyro();
    Serial.print("angle: ");
    Serial.println(degreeZ);
}