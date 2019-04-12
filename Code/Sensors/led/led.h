#ifndef LED_H
#define LED_H

#include "Arduino.h"

class LED {
private:
  int pin;

public:
  LED(int _pin);
  void on();
  void off();
};

#endif // LED_H