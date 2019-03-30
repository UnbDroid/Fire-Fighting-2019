#include "despair.h"

const float  Motor::BATTERY_LEVEL = 15.1;
const bool   Motor::FORWARD       = true;
const bool   Motor::BACKWARDS     = false;
const bool   Motor::LEFT          = true;
const bool   Motor::RIGHT         = false;

Motor::Motor(int _a_pin, int _b_pin, int _pwm_pin) {
  a_pin   = _a_pin;
  b_pin   = _b_pin;
  pwm_pin = _pwm_pin;

  pinMode(a_pin, OUTPUT);
  pinMode(b_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
}

bool Motor::getDirection() {
  return direction;
}

long Motor::getEncoder() {
  return encoder;
}

void Motor::sumEncoder() {
  encoder++;
}

void Motor::subEncoder() {
  encoder--;
}

void Motor::move(int voltage, bool side) {
  int pwm;

  pwm = voltage*255/BATTERY_LEVEL;

  if (pwm >= 0) {
    if(side == LEFT)
      direction = FORWARD;
    else
      direction = BACKWARDS;
    digitalWrite(a_pin, HIGH);
    digitalWrite(b_pin, LOW);
  } else {
    pwm = -pwm;
    if(side == RIGHT)
      direction = FORWARD;
    else
      direction = BACKWARDS;
    digitalWrite(a_pin, LOW);
    digitalWrite(b_pin, HIGH);
  }
  analogWrite(pwm_pin, (byte)round(pwm));
}

void Motor::stop() {
  digitalWrite(a_pin, HIGH);
  digitalWrite(b_pin, HIGH);
  digitalWrite(a_pin, HIGH);
  digitalWrite(b_pin, HIGH);
  digitalWrite(a_pin, HIGH);
  digitalWrite(b_pin, HIGH);
}