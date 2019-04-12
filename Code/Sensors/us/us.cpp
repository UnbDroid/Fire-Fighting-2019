#include "us.h"

unsigned int frontalUS() {
  return frontal_us.convert_cm(frontal_us.ping_median(10)); 
}
unsigned int leftFrontUS() {
  return left_front_us.convert_cm(left_front_us.ping_median(10)); 
}
unsigned int leftBackUS() {
  return left_back_us.convert_cm(left_back_us.ping_median(10)); 
}
unsigned int rightFrontUS() {
  return right_front_us.convert_cm(right_front_us.ping_median(10)); 
}
unsigned int rightBackUS() {
  return right_back_us.convert_cm(right_back_us.ping_median(10)); 
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