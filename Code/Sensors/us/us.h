#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"

// US lib
#include "NewPing.h"

extern NewPing * frontal_us;
extern NewPing * left_front_us;
extern NewPing * left_back_us;
extern NewPing * right_front_us;
extern NewPing * right_back_us;

// US functions return 0 if they read more than 200 cm
#define MAX_DIST 200

// === Frontal US === //
#define FRONTAL_TRIG      53
#define FRONTAL_ECHO      51
unsigned int frontalUS();  // Takes 5 sensor readings and returns the median of the distances 

// === Left US' === //
#define LEFT_FRONT_TRIG   49
#define LEFT_FRONT_ECHO   47
unsigned int leftFrontUS(); // Takes 5 sensor readings and returns the median of the distances

#define LEFT_BACK_TRIG    45
#define LEFT_BACK_ECHO    43
unsigned int leftBackUS(); // Takes 5 sensor readings and returns the median of the distances

// === Right US' === //
#define RIGHT_FRONT_TRIG  41
#define RIGHT_FRONT_ECHO  39
unsigned int rightFrontUS(); // Takes 5 sensor readings and returns the median of the distances

#define RIGHT_BACK_TRIG   35
#define RIGHT_BACK_ECHO   37
unsigned int rightBackUS(); // Takes 5 sensor readings and returns the median of the distances

// Prints all US readings on Serial Monitor. Order: frontal, leftFront, leftBack, rightFront, rightBack
void printUS();

#endif // SENSORS_H