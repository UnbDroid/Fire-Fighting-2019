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
  while(status == ACTIVE)
    this->update();
  led->off();
}

void Mic::printStatus() {
  this->update();
  Serial.println(status);
}

void Mic::hear(int threshold){
  //Counters for the moving average
  int count[5];
  count[0] = 0;
  count[1] = 0;
  count[2] = 0;
  count[3] = 0;
  count[4] = 0;

  //timer
  int t;

  //time step
  int tStep;

  int freq;

  //keep hearing untill appears sound with frequency above the threshold
  do{
    //Set Counters
    count[4]=count[3];
    count[3]=count[2];
    count[2]=count[1];
    count[1]=count[0];
    count[0]=0;

    //Count pulses in one period
    t = millis();
    while((millis() - t) <= tStep){
      if(digitalRead(pin) == HIGH){
        count[0] = count[0] + 1;
      }
      delay(1);     //Delay needed to avoid overflow
    }

    //Calculate corresponding frequency for amount counted
    freq = 0;
    for (int i=0;i<5;i++){
      freq = freq + count[i];
    }
    freq = freq * 1000    //convert mHz to Hz
                /tStep;
    
  }while(freq < threshold);
  
  //Indicating that right sound was detected
  led->on();
  delay(1000);
  led->off();

}