#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"

// US lib
//#include "/home/digo/Documents/DROID/FireFighting/Fire-Fighting-2019/Code/lib/NewPing/NewPing.h"

// US functions return 0 if they read more than 200 cm
#define MAX_DIST 200

// === Frontal US === //
#define FRONTAL_TRIG      49
#define FRONTAL_ECHO      39
unsigned int frontalUS();  // Takes 5 sensor readings and returns the median of the distances 
extern NewPing * frontal_us;

// === Left US' === //
#define LEFT_FRONT_TRIG   51
#define LEFT_FRONT_ECHO   41
unsigned int leftFrontUS(); // Takes 5 sensor readings and returns the median of the distances
extern NewPing * left_front_us;

#define LEFT_BACK_TRIG    53
#define LEFT_BACK_ECHO    43
unsigned int leftBackUS(); // Takes 5 sensor readings and returns the median of the distances
extern NewPing * left_back_us;

// === Right US' === //
#define RIGHT_FRONT_TRIG  47
#define RIGHT_FRONT_ECHO  37
unsigned int rightFrontUS(); // Takes 5 sensor readings and returns the median of the distances
extern NewPing * right_front_us;

#define RIGHT_BACK_TRIG   45
#define RIGHT_BACK_ECHO   35
unsigned int rightBackUS(); // Takes 5 sensor readings and returns the median of the distances
extern NewPing * right_back_us;

// Prints all US readings on Serial Monitor. Order: frontal, leftFront, leftBack, rightFront, rightBack
void printUS();

#endif // SENSORS_H
