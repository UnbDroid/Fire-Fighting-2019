#ifndef MIC_H
#define MIC_H

#include "Arduino.h"

#include "led.h"

class Mic {
private:
  const static int SAMPLE_NUM = 100;
  int pin;
  bool status = INACTIVE;
  LED * led = new LED(29);

  void sort(bool *sample);
  void update();
  bool getStatus();

public:
  const static bool ACTIVE = true;
  const static bool INACTIVE = false;

  Mic(int _pin);
  void start();
  void printStatus();
};

#endif // MIC_H