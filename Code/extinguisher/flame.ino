#include "flame.h"

Extinguisher extinguisher;

void setup() {
    Serial.begin(9650);
}

void loop(){
    extinguisher.searchFlame();
}