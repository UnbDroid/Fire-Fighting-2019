#include "mic.h"

Mic::Mic(int _pin) {
  pin = _pin;
  pinMode(pin, INPUT);
}

// Bubble sort algorythm used to sort the array of samples
void Mic::sort (bool *sample) {
  int k, j;
  bool aux;

  for (k = 1; k < SAMPLE_NUM; k++) {
    for (j = 0; j < SAMPLE_NUM - 1; j++) {
      if (sample[j] > sample[j + 1]) {
        aux          = sample[j];
        sample[j]     = sample[j + 1];
        sample[j + 1] = aux;
      }
    }
  }
}

// Reads the sensor and updates the last seen color. Filters by median.
void Mic::update() {
  bool sample[SAMPLE_NUM]; // array of samples

  // Fills the array
  for(int i = 0; i < SAMPLE_NUM; i++){
    sample[i] = digitalRead(pin);
    Serial.println(sample[i]);
  }
        
  sort(sample);

  // Takes the median of all samples and updates the last color
  status = sample[(SAMPLE_NUM)/2];
}

bool Mic::getStatus() {
  return status;
}

void Mic::start() {
  while(status == INACTIVE)
    this->update();
  led->on();
  delay(1000);
  while(status == ACTIVE)
    this->update();
  led->off();
}

void Mic::printStatus() {
  this->update();
  Serial.println(status);
}

void Mic::gambiarra(int tests){
  int count;
  while(count<tests){
    count = 0;
    for(int i=0;i<=3*tests/2;i++){
      if (digitalRead(pin) == HIGH){
        count++;
      }
      delay(1);
    }
  }
}