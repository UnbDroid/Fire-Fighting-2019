#ifndef MIC_H
#define MIC_H

#include "Arduino.h"
#include "/home/digo/Documents/DROID/FireFighting/Fire-Fighting-2019/Code/Sensors/led/led.h"

class Mic {
private:
  const static int SAMPLE_NUM = 100;
  
  int pin;
  bool status = INACTIVE;

  void sort(bool *sample);

public:
  const static bool ACTIVE = true;
  const static bool INACTIVE = false;

  LED * led = new LED(29);

  Mic(int _pin);
  void start();
  void update();
  bool getStatus();
  void printStatus();
};

#endif // MIC_H
