#include "flame.h"

Extinguisher extinguisher;

void setup() {
    Serial.begin(9600);
}

void loop(){
    Serial.println("Start!");
    extinguisher.searchFlame();
    Serial.println("End!");
    delay(5000);
}