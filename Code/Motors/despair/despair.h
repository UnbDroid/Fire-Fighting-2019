#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Arduino.h"

class Motor {
  int   a_pin;
  int   b_pin;
  int   pwm_pin;
  bool  direction;
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

#endif // CONTROLLER_H