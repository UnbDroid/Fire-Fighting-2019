#include "MPU9250.h"

// Time between gyro updates
#define TIME_STEP 0.01

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

float radX    = 0;
float radY    = 0;
float radZ    = 0;
float degreeX = 0; 
float degreeY = 0;
float degreeZ = 0;

unsigned long timer;

// Converts angles from rads to degrees
void GyroRad2Degree() {
  //degreeX = (radX * 180) / PI;
  //degreeY = (radY * 180) / PI;
  degreeZ = (radZ * 180) / PI;
}

// Gets gyro's raw readings (rad/s) and integrates them into angles (rad).
// Angles are also converted from rad to degrees. 
void UpdateGyro() {
  timer = millis();

  IMU.readSensor();
  //radX = IMU.getGyroX_rads();
  //radY = IMU.getGyroY_rads();
  radZ += IMU.getGyroZ_rads() * TIME_STEP;

  GyroRad2Degree();
  delay((TIME_STEP * 1000)- (millis() - timer));
}

// Gyro setup function
void StartGyro() {
  status = IMU.begin();
  if (status < 0)
    Serial.println("IMU initialization unsuccessful");
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  StartGyro();
}

void loop() {
  UpdateGyro();
  Serial.print("Z = ");
  Serial.println(degreeZ);
  delay(100);
}