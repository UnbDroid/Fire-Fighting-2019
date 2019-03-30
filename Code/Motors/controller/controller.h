#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// Gyro lib
#include "Arduino.h"
#include "MPU9250.h"

class Motor {
private:
  int   a_pin;
  int   b_pin;
  int   pwm_pin;
  bool  direction = FORWARD;
  volatile long encoder = 0;

public:
  const static float  BATTERY_LEVEL;
  const static bool   FORWARD;
  const static bool   BACKWARDS;
  const static bool   LEFT;
  const static bool   RIGHT;

  Motor(int _a_pin, int _b_pin, int _pwm_pin);
  bool getDirection();
  long getEncoder();
  void sumEncoder();
  void subEncoder();
  void move(int voltage, bool side);
  void stop();
};

class Controller {
private:
  // Saturation Value
  const static double SATURATION_VALUE;
  
  // PI Constants
  const static double KP;
  const static double KI;
  const static double KG;

  // Iteration step in milli seconds
  const static int TIME_STEP;
  
  // Encoder counts equivalent to 1 full rotation
  const static int ENC_COUNTS;

  // Constant used when converting a distance in cm to degrees
  const static int WHEEL_RADIUS;

  // Set the turning speed 
  const static int TURN_TENSION;

  static long left_old_enc;
  static long right_old_enc;
  static long left_old_speed;
  static long right_old_speed;
  static long left_speed;   // Left motor speed
  static long right_speed;   // Right motor speed

  float degreeZ = 0;

  MPU9250 * gyro = new MPU9250(Wire, 0x68);

  float dist2Counts(float distance);
  void  resetSpeed();
  void  saturationDetector(float *left_pwr_signal, float *right_pwr_signal);
  void  updateSpeed(float dt);
  void  antiWindUp(float *left_integral,float *right_integral);
  void  updateGyro();

public:
  Motor * left_motor  = new Motor(32, 34, 9);
  Motor * right_motor = new Motor(38, 40, 8);

  void move(float speed, int distance);
  void stop();
  void turn(float angle);
  void Controller::begin();
};

#endif // CONTROLLER_H_