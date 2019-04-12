#ifndef FLAME_H
#define FLAME_H

#include "Arduino.h"
#include "Servo.h"

class Pump {
private:
  int pin;

public:
  Pump(int _pin);
  void on();
  void off();
};

class FlameSensor {
private: 

  int pin_analog;
  int pin_digital;
  int fire_analog = 1024;
  int fire_digital = 0;
  const static int SAMPLE_NUM = 5;
  

public:
  FlameSensor(int _pin_analog, int _pin_digital);
  void sort(int *sample);
  void update();
  int getAnalog();
  int getDigital();
  void printFlame();
};

class Extinguisher {
private:

	int servo_position;
  int count_fire;

  Pump * pump = new Pump(38);
  Servo servo;
  FlameSensor * sensor = new FlameSensor(A0, 7);

  void findFire();
  void moveServo(int from_position, int to_position);
  void moveServoSearchingFlame(int from_position, int to_position);

public:

  const static int FIRE            = 75; // 75 and bellow are the analog values for when the sensor is facing the flame 
  const static int SERVO_START     = 90;
  const static int SERVO_LEFT_MAX  = 135;
  const static int SERVO_RIGHT_MAX = 45;
  const static int LEFT_SIDE       = 1;
  const static int RIGHT_SIDE      = -1;
  const static int LITTLE_SEARCH   = 10;
  const static int SPREAD_DEGREES  = 10;

  bool fire_exist = false;
  int fire_position = 0;

  void searchFlame(int room_side);
  void extinguishFire();

};

#endif