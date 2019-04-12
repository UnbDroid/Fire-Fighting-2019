#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"

// US lib
#include "NewPing.h"

// US functions return 0 if they read more than 200 cm
#define MAX_DIST 200  

// === Frontal US === //
#define FRONTAL_TRIG      49
#define FRONTAL_ECHO      39
NewPing frontal_us(FRONTAL_TRIG, FRONTAL_ECHO, MAX_DIST);
unsigned int frontalUS();  // Takes 5 sensor readings and returns the median of the distances 

// === Left US' === //
#define LEFT_FRONT_TRIG   51
#define LEFT_FRONT_ECHO   41
NewPing left_front_us(LEFT_FRONT_TRIG, LEFT_FRONT_ECHO, MAX_DIST);
unsigned int leftFrontUS(); // Takes 5 sensor readings and returns the median of the distances

#define LEFT_BACK_TRIG    53
#define LEFT_BACK_ECHO    43
NewPing left_back_us(LEFT_BACK_TRIG, LEFT_BACK_ECHO, MAX_DIST);
unsigned int leftBackUS(); // Takes 5 sensor readings and returns the median of the distances

// === Right US' === //
#define RIGHT_FRONT_TRIG  47
#define RIGHT_FRONT_ECHO  37
NewPing right_front_us(RIGHT_FRONT_TRIG, RIGHT_FRONT_ECHO, MAX_DIST);
unsigned int rightFrontUS(); // Takes 5 sensor readings and returns the median of the distances

#define RIGHT_BACK_TRIG   45
#define RIGHT_BACK_ECHO   35
NewPing right_back_us(RIGHT_BACK_TRIG, RIGHT_BACK_ECHO, MAX_DIST);
unsigned int rightBackUS(); // Takes 5 sensor readings and returns the median of the distances

// Prints all US readings on Serial Monitor. Order: frontal, leftFront, leftBack, rightFront, rightBack
void printUS();

#endif // SENSORS_H