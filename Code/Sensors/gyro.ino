/*------------------------------------------ Gyroscope Functions ---------------------------------------------*/

// Comments:
// Here we have the Turn function and the angular position calculation.

// Usage:
// To read the current angular position just call UpdateGyro, beware that this function shouldn't be called 
// more than once in 20 milli seconds since it time step is 20.
// The Turn function receives the angle disired. Positive turns right and negative turns left.

// Gyro lib
#include "MPU9250.h"

// Iteration step in milli seconds
#define TIME_STEP 20

// Set the turning speed 
#define Turn_Tension 3

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;
float radZ = 0;

void Turn(float degrees) {
  
  static long now;
  static long last_update = 0;

  float offset = degreeZ;
  
  if (degrees > 0) {
    Move_Right_Motor(-Turn_Tension);
    Move_Left_Motor(Turn_Tension);
    while(degreeZ < (offset + degrees)){
      now = millis();
      if (now - last_update >= TIME_STEP) {
        UpdateGyro();
        last_update = now;
      }
    }

  } else {
    Move_Right_Motor(Turn_Tension);
    Move_Left_Motor(-Turn_Tension);
    while(degreeZ > (offset + degrees)){
      now = millis();
      if (now - last_update >= TIME_STEP) {
        UpdateGyro();
        last_update = now;
      }
    }
  }
  StopMotors();
}

// Gets gyro's raw readings (rad/s) and integrates them into angles (rad).
// Angles are also converted from rad to degrees. 
void UpdateGyro() {
  gyro.readSensor();
  degreeZ += (gyro.getGyroZ_rads() * TIME_STEP * 180) / (1000 * PI);
}


// Gyro setup function
void StartGyro() {
  status = gyro.begin();
  if (status < 0)
    Serial.println("IMU initialization unsuccessful");
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  StartGyro();
}

void loop() {

  int now,last_update = 0;

  while(1){
    now = millis();
    if(now - last_update > 20){
      UpdateGyro();
      Serial.print("Z = ");
      Serial.println(degreeZ);
      last_update = now;
    }
  }
}