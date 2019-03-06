#ifndef FLAME_H
#define FLAME_H

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
  int fire_position = 0;
  bool fire_exist = false;

public:
  FlameSensor(int _pin);
  void update();
};

class Extinguisher {
private:

	int servo_position;
  int count_fire

public:

  const static int FIRE            75 // 75 and bellow are the analog values for when the sensor is facing the flame 
  const static int SERVO_START     90
  const static int SERVO_LEFT_MAX  135
  const static int SERVO_RIGHT_MAX 45

  void findFire();
  void movesServo();
  int searchFlame();
  void extinguishFire();

};

#endif