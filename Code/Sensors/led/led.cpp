#include "led.h"

LED::LED(int _pin) {
  pin = _pin;
  pinMode(pin, OUTPUT);
}

void LED::on() {
  digitalWrite(pin, HIGH);
}

void LED::off() {
  digitalWrite(pin, LOW);
}