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
      extinguisher.led->on();
      moveDistance(450, 25); // valor para que todo o robo esteja dentro do quarto TESTAR
      turn(extinguisher.fire_position - 90); // verificar 2
      extinguisher.searchFlame(Extinguisher::RIGHT_SIDE);//search the flame again
      turn(extinguisher.fire_position - 90); // turn to the flame
      //move close to the candle 
      extinguishMovements();
      extinguisher.led->off();
    }
    
  } else {
    extinguisher.searchFlame(Extinguisher::LEFT_SIDE);
    if (extinguisher.fire_exist) {  //verificar se da pra usar a variavel assim
      extinguisher.led->on();
      moveDistance(450, 25); // valor para que todo o robo esteja dentro do quarto TESTAR
      turn(extinguisher.fire_position - 90); // verificar 2
      extinguisher.searchFlame(Extinguisher::LEFT_SIDE);//search the flame again
      turn(extinguisher.fire_position - 90); // turn to the flame
      //move close to the candle 
      extinguishMovements();
      extinguisher.led->off();
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
  //float distance = counts2Dist((lenc_pos + renc_pos)/2.0);

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
  while(rightFrontUS() < 18) {
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
  while(frontalUS() > 8) {
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

  delay(300);
  resetController();
  while(color_l.getColor() == Reflection::BLACK && color_r.getColor() == Reflection::BLACK){
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(700);
  }
  stopMotors();
  delay(300);
  moveDistance(600, -3);
  approachLine();
  resetController();
  while(frontalUS() < 8) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(700);
  }
  stopMotors();
  float distance = counts2Dist((lenc_pos + renc_pos)/2.0);
  if (distance < 100){
    dog_position = B;
    moveDistance(600, distance - 20);
    turn(-90);
    resetController();
    while(leftFrontUS() > 18) {
      now = millis();
      dt = now - last_update;

      if(dt > TIME_STEP)
        controller(600);
    }
    stopMotors();
    delay(300);
    moveDistance(600, 5);
    resetController();
    while(leftFrontUS() < 18) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(600);
    }
    stopMotors();
    delay(300);
    moveDistance(25);
    turn(90);
    resetController();
    while(FrontUS() < 5) {
      now = millis();
      dt = now - last_update;

      if(dt > TIME_STEP)
        controller(600);
    }
    stopMotors();
    delay(300);

    } else {
      turn(-90);
      resetController();
      while(frontalUS() < 8) {
        now = millis();
        dt = now - last_update;

        if(dt > TIME_STEP)
          controller(450);
      }
      stopMotors();
      delay(300);
    if (dog_position != A){
      resetController();
      while(leftFrontUS() > 20 || leftFrontUS() == 0){
        now = millis();
        dt = now - last_update;

        if(dt > TIME_STEP)
          controller(-450);
      }
      stopMotors();
      delay(300);
      moveDistance(600, -3);
    }
    resetController();
    while(leftFrontUS() < 20){
      now = millis();
      dt = now - last_update;

      if(dt > TIME_STEP)
        controller(-700);
    }
    stopMotors();
    delay(300);
    moveDistance(-5);
    turn(-90);
    approachLine();    
  }
  resetController();
}

void room4ToStart() {}



void track2() {
  // startToRoom4();
  // room4ToRoom1();
  // room1ToRoom2();
  // room2ToMiddle();
  // middleToRoom3();
  // room3ToMiddle();
  // middleToStart();
}

void return1() {
  resetController();
  moveDistance(600, 37);
  stopMotors();
  delay(300);
  turn(90);
  
  resetController();
  while(frontalUS() > 8) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(600);
  }
  stopMotors();
}
void return2() {
  resetController();
  moveDistance(600, 37);
  stopMotors();
  delay(300);
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
  while(rightFrontUS() < 18) {
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
void return4() {
  resetController();
  while(frontalUS() > 8) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(600);
  }
  stopMotors();
  delay(200);
  if(dog_position != 'A'){
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
  else{
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
    
    resetController();
    moveDistance(600, 5);
    while(rightFrontUS() < 18) {
      now = millis();
      dt = now - last_update;

      if(dt > TIME_STEP)
        controller(600);
    }
    stopMotors();
    moveDistance(600, 37);
    delay(200);
    turn(90);
    
    resetController();
    moveDistance(600, 5);
    while(rightFrontUS() < 18) {
     now = millis();
     dt = now - last_update;

     if(dt > TIME_STEP)
      controller(600);
    }
    stopMotors();
    moveDistance(600, 37);
    delay(200);
    turn(90);
    
    resetController();
    while(frontalUS() > 8) {
      now = millis();
      dt = now - last_update;

      if(dt > TIME_STEP)
        controller(600);
    }
    stopMotors();
  }
}


int track1() {
  startToMiddle();
  middleToRoom3();
  searchRoom(Extinguisher::RIGHT_SIDE);
  if(extinguisher.fire_exist){
    return3();
    return 0;
  }
  room3ToMiddle();
  middleToRoom2();
  searchRoom(Extinguisher::RIGHT_SIDE);
  if (extinguisher.fire_exist) {
    return2();
    return 0;
  }  
  room2ToRoom1();
  searchRoom(Extinguisher::LEFT_SIDE);
  if(extinguisher.fire_exist){
    return1();
    return 0;
  }
  room1ToRoom4();
  searchRoom(Extinguisher::LEFT_SIDE);
  room4ToStart();
}

void chooseTrack() {
  if(( ((rightFrontUS() + rightBackUS()) /2) == 0) ||((rightFrontUS() + rightBackUS()) /2) > 20) )
    turn(90);
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
  
  
  // moveDistance(600, 30);
  // turn(90);
  // turn(-90);
  // moveDistance(600, -30);


  mic.gambiarra(150);
  chooseTrack();
  track1();

}
