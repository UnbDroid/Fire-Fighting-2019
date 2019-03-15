#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// Gyro lib
#include "../Sensors/MPU9250/MPU9250.h"

class Motor {
private:
  int   a_pin;
  int   b_pin;
  int   pwm_pin;
  bool  direction;
  volatile long encoder;

public:
  const static float  BATTERY_LEVEL = 15.1;
  const static bool   FORWARD       = true;
  const static bool   BACKWARDS     = false;

  Motor(int _a_pin, int _b_pin, int _pwm_pin);
  bool getDirection();
  long getEncoder();
  void sumEncoder();
  void move(int voltage);
  void stop();
};

class Controller {
private:
  // Saturation Value
  const static double SATURATION_VALUE = 14.4;
  
  // PI Constants
  const static double KP = 0.0125;
  const static double KI = 0.015;
  const static double KG = 50;

  // Iteration step in milli seconds
  const static int TIME_STEP = 20;
  
  // Encoder counts equivalent to 1 full rotation
  const static int ENC_COUNTS = 560;

  // Constant used when converting a distance in cm to degrees
  const static int WHEEL_RADIUS = 4;

  static long left_old_enc    = 0;
  static long right_old_enc   = 0;
  static long left_old_speed  = 0;
  static long right_old_speed = 0;
  static long left_speed      = 0;   // Left motor speed
  static long right_speed     = 0;   // Right motor speed

  float degreeZ = 0;

  MPU9250 * gyro = new MPU9250(Wire,0x68);

  float dist2Counts(float distance);
  void  resetSpeed();
  void  saturationDetector(float *left_pwr_signal, float *right_pwr_signal);
  void  updateSpeed(float dt);
  void  antiWindUp(float *left_integral,float *right_integral);
  void  updateGyro();

public:
  Motor * left_motor  = new Motor(32, 34, 9);
  Motor * right_motor = new Motor(39, 40, 8);

  void move(float speed,int distance);
  void stop();
  void turn(float degrees);
  void begin();
};

#endif // CONTROLLER_H_