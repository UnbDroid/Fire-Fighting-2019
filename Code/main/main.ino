// Trinity College Fire-Fighting Home Robot Contest
// Equipe DROID - University of Brasília, Brasília - DF, Brazil
// Authors: Daniel Bauchspiess, Gabriel Gonçalves, Lucas Faria, Pedro Saman, Rebeca Helen, Vanessa Rodrigues

// This is the main source code for the robot Atrasadinho. It contains the routines and functions responsible for arena navigation and candle extinguishing. All source code can be found and downloaded on https://github.com/UnbDroid/Fire-Fighting-2019/Code

#include "motors.h"
#include "mic.h"
#include "reflection.h"
#include "us.h"
#include "flame.h"

#define A 'a'
#define B 'b'
#define C 'c'
#define NOT_FOUND 'n'

Extinguisher extinguisher;
Reflection color_l(23);
Reflection color_r(25);
Mic mic(12);

char dog_position = NOT_FOUND;

void extinguishMovements(){
  for(int i = 0; i <= 5; i++){
    extinguisher.extinguishFire();
    moveDistance(300, -3); //moves slowly backwards
    extinguisher.extinguishFire();
    moveDistance(300, 6);
    extinguisher.extinguishFire();
    moveDistance(300, -3); //vale a pena verificar se ja está apagada?
  }
  
}

void searchRoom(int room_side) {
  if (room_side == Extinguisher::RIGHT_SIDE) {
    extinguisher.searchFlame(Extinguisher::RIGHT_SIDE);
    if (extinguisher.fire_exist){  //verificar se da pra usar a variavel assim
      //turn on LED
      moveDistance(450, 25); // valor para que todo o robo esteja dentro do quarto TESTAR
      turn(extinguisher.fire_position - 90); // verificar 2
      extinguisher.searchFlame(Extinguisher::RIGHT_SIDE);//search the flame again
      turn(extinguisher.fire_position - 90); // turn to the flame
      //move close to the candle 
      extinguishMovements();
      //Turn of LED 
    }
    
  } else {
    extinguisher.searchFlame(Extinguisher::LEFT_SIDE);
    if (extinguisher.fire_exist) {  //verificar se da pra usar a variavel assim
      //turn on LED
      moveDistance(450, 25); // valor para que todo o robo esteja dentro do quarto TESTAR
      turn(extinguisher.fire_position - 90); // verificar 2
      extinguisher.searchFlame(Extinguisher::LEFT_SIDE);//search the flame again
      turn(extinguisher.fire_position - 90); // turn to the flame
      //move close to the candle 
      extinguishMovements();
      //Turn of LED
    }
  }  
}

void approachLine() {
  moveLeftMotor(7);
  moveRightMotor(7);
  while(color_l.getColor() == Reflection::BLACK && color_r.getColor() == Reflection::BLACK) {
    color_l.update();
    color_r.update();
    if(color_l.getColor() == Reflection::BLACK)
      stopLeftMotor();
    if(color_r.getColor() == Reflection::BLACK)
      stopRightMotor();
  }
  stopMotors();
}

void startToMiddle() {
  resetController();
  while(rightFrontUS() < 20) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(600);
  }
  stopMotors();

  resetController();
  while(rightFrontUS() > 20) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(600);
  }
  stopMotors();
  float distance = counts2Dist((lenc_pos + renc_pos)/2.0);

  moveDistance(600, -10);
}

void middleToRoom3() {
  turn(90);
  resetController();
  while(frontalUS() > 8) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(600);
  }
  stopMotors();
  delay(300);
  turn(90);

  approachLine();
}

void room3ToMiddle() {
  moveDistance(600, -3);
  turn(90);

  resetController();
  while(leftFrontUS() < 18) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(600);
  }
  stopMotors();
  delay(300);
  moveDistance(600, 37);
  delay(200);
  turn(90);
}

void middleToRoom2() {
  resetController();
  while(frontalUS() < 8) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(700);
  }
  stopMotors();
  turn(90);

  approachLine();
}

void room2ToRoom1() {
  moveDistance(600, -3);
  turn(180);
  moveDistance(600, 10);
  approachLine();
}

void room1ToRoom4() {
  resetController();
  while(frontalUS() < 8) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(700);
  }
  stopMotors();
  turn(-90);

  if(dog_position == C)

  resetController();
}

void room4ToStart() {}


void track1() {
  startToMiddle();
  middleToRoom3();
  room3ToMiddle();
  middleToRoom2();
  room2ToRoom1();
  room1ToRoom4();
  room4ToStart();
}

void track2() {
  // startToRoom4();
  // room4ToRoom1();
  // room1ToRoom2();
  // room2ToMiddle();
  // middleToRoom3();
  // room3ToMiddle();
  // middleToStart();
}

void return1() {}
void return2() {}
void return3() {
  resetController();
  while(frontalUS() > 8) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(600);
  }
  stopMotors();
  delay(300);
  turn(-90);

  resetController();
  while(leftFrontUS() < 18) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(600);
  }
  stopMotors();
  moveDistance(600, 37);
  delay(200);
  turn(-90);

  resetController();
  while(frontalUS() > 8) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(600);
  }
  stopMotors();
}
void return4() {}

int chooseTrack() {
  if((leftFrontUS() + leftBackUS())/2 > (rightFrontUS() + rightBackUS())/2)
    return 1;
  return 2;
}

void setup() {
  startMotors();
  extinguisher.start();
  Serial.begin(9600);
}

void loop() {
  // if(chooseTrack() == 1)
  //   track1();
  // else
  //   track2();

  // stopMotors();
  // while(1) {
  //   mic.led->on();
  //   extinguisher.led->off();
  //   delay(500);
  //   mic.led->off();
  //   extinguisher.led->on();
  //   delay(500);
  // }
  moveDistance(600, 30);
  turn(90);
  turn(-90);
  moveDistance(600, -30);
}