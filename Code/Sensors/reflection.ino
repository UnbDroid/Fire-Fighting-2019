#include "reflection/reflection.h"

#define COLOR_PIN 8

// Reflection object declaring. Receives digital pin number for the reflection sensor.
Reflection color_sensor(COLOR_PIN); 

void setup() {
    Serial.begin(9650);
}

void loop() {
    color_sensor.update(); // reads the sensor
    Serial.println(color_sensor.getColor()); // prints the read value
}