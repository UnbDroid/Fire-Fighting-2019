#include "reflection.h"

// Reflection object declaring. Receives digital pin number for the reflection sensor.
Reflection color_r(33); 
Reflection color_l(31);

void setup() {
    Serial.begin(9600);
}

void loop() {
    color_r.update(); // reads the sensor
    color_l.update(); // reads the sensor
    Serial.print(color_l.getColor()); // prints the read value
    Serial.print(" ");
    Serial.println(color_r.getColor()); // prints the read value
}