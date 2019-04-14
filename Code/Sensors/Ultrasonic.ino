// US lib
#include "NewPing.h"

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

NewPing * frontal_us = new NewPing(FRONTAL_TRIG, FRONTAL_ECHO, MAX_DIST);
NewPing * left_front_us = new NewPing(LEFT_FRONT_TRIG, LEFT_FRONT_ECHO, MAX_DIST);
NewPing * left_back_us = new NewPing(LEFT_BACK_TRIG, LEFT_BACK_ECHO, MAX_DIST);
NewPing * right_front_us = new NewPing(RIGHT_FRONT_TRIG, RIGHT_FRONT_ECHO, MAX_DIST);
NewPing * right_back_us = new NewPing(RIGHT_BACK_TRIG, RIGHT_BACK_ECHO, MAX_DIST);

unsigned int frontalUS() {
  return frontal_us->convert_cm(frontal_us->ping_median(10)); 
}
unsigned int leftFrontUS() {
  return left_front_us->convert_cm(left_front_us->ping_median(10)); 
}
unsigned int leftBackUS() {
  return left_back_us->convert_cm(left_back_us->ping_median(10)); 
}
unsigned int rightFrontUS() {
  return right_front_us->convert_cm(right_front_us->ping_median(10)); 
}
unsigned int rightBackUS() {
  return right_back_us->convert_cm(right_back_us->ping_median(10)); 
}

void printUS() {
  Serial.print(frontalUS());
  Serial.print("  ");
  Serial.print(leftFrontUS());
  Serial.print("  ");
  Serial.print(leftBackUS());
  Serial.print("  ");
  Serial.print(rightFrontUS());
  Serial.print("  ");
  Serial.println(rightBackUS());  
}

void setup() {
  Serial.begin(9600);
}
 
void loop() {
  printUS();
}