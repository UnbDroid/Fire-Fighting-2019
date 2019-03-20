#include "controller.h"

#define L_ENC 18
#define R_ENC 19

Controller controller;

void doLeftEncoder() {
  if(controller.left_motor->getDirection() == Motor::FORWARD)
    controller.left_motor->sumEncoder();
  else
    controller.left_motor->subEncoder();
}

void doRightEncoder() {
  if(controller.right_motor->getDirection() == Motor::FORWARD)
    controller.right_motor->sumEncoder();
  else
    controller.right_motor->subEncoder();
}

void startEncoders() {
  // Left encoder
  pinMode(L_ENC, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENC), doLeftEncoder, CHANGE);
  // Right encoder
  pinMode(R_ENC, INPUT);
  attachInterrupt(digitalPinToInterrupt(R_ENC), doRightEncoder, CHANGE);
}

void setup() {
  controller.begin();
  startEncoders();
  Serial.begin(115200);
}

void loop() {
  Serial.print("oi");
  controller.move(300, 25);
  controller.stop();
  controller.turn(90);
  controller.move(300, -25);
  controller.stop();
  controller.turn(-90);
}