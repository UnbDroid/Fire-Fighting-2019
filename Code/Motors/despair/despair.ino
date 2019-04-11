#include "despair.h"

#define L_ENC 18
#define R_ENC 19

Motor * left_motor = new Motor(32, 34, 9);
Motor * right_motor = new Motor(38, 40, 8);

void doLeftEncoder() {
  if(left_motor->getDirection() == Motor::FORWARD)
    left_motor->sumEncoder();
  else
    left_motor->subEncoder();
}

void doRightEncoder() {
  if(right_motor->getDirection() == Motor::FORWARD)
    right_motor->sumEncoder();
  else
    right_motor->subEncoder();
}

void startEncoders() {
  // Left encoder
  pinMode(L_ENC, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENC), doLeftEncoder, CHANGE);
//   // Right encoder
  pinMode(R_ENC, INPUT);
  attachInterrupt(digitalPinToInterrupt(R_ENC), doRightEncoder, CHANGE);
}

void setup() {
  startEncoders();
  Serial.begin(115200);
}

void loop() {
  left_motor->move(10, Motor::LEFT);
  right_motor->move(-10, Motor::RIGHT);
  delay(3000);
  left_motor->stop();
  right_motor->stop();
  Serial.print("Left: ");
  Serial.println(left_motor->getEncoder());
  Serial.print("Right: ");
  Serial.println(right_motor->getEncoder());
  delay(1000);
  left_motor->move(-10, Motor::LEFT);
  right_motor->move(10, Motor::RIGHT);
  delay(3000);
  left_motor->stop();
  right_motor->stop();
  Serial.print("Left: ");
  Serial.println(left_motor->getEncoder());
  Serial.print("Right: ");
  Serial.println(right_motor->getEncoder());
  delay(1000);
}