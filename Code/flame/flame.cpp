#include "flame.h"

Pump::Pump(int _pin) {
  pin = _pin;
  pinMode(pin, OUTPUT);
}

void Pump::on() {
  digitalWrite(pin, HIGH);
}

void Pump::off() {
  digitalWrite(pin, LOW);
}

FlameSensor::FlameSensor(int _pin_analog, int _pin_digital){
  pin_analog = _pin_analog;
  pin_digital = _pin_digital;

  pinMode(pin_analog, INPUT);
  pinMode(pin_digital, INPUT);
}

void FlameSensor::sort (int *sample) {
    int k, j;
    int aux;

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

void FlameSensor::update() { // Reads the flame sensor and updates the fire variables 
  int sample_analog[SAMPLE_NUM]; // array of samples
  int sample_digital[SAMPLE_NUM];

    // Fills the array
  for(int i = 0; i < SAMPLE_NUM; i++){
     sample_analog[i] = analogRead(pin_analog);
      sample_digital[i] = digitalRead(pin_digital);
  }

  sort(sample_analog);
  sort(sample_digital);

  // Takes the median of all samples and updates the last color
  fire_analog = sample_analog[(SAMPLE_NUM + 1)/2];
  fire_digital = sample_digital[(SAMPLE_NUM + 1)/2];

  Serial.print("Porta analogica: ");
  Serial.println(fire_analog);
  //Serial.print(" Porta digital: ");
  //Serial.println(fire_digital);
}

int FlameSensor::getAnalog() {
  return this->fire_analog;
}

int FlameSensor::getDigital() {
  return this->fire_digital;
}

void FlameSensor::printFlame() {
  this->update();
  Serial.println(this->getAnalog());
}

void Extinguisher::findFire(){ // Checks if there is a fire and does the average of the servo position to know where the flame is
  //int lastfire_analog = fire_analog;
  sensor->update();
  if (sensor->getAnalog() <= FIRE) {   
    Serial.println("Achei fogo");
    fire_exist = true;       // updates the fire existence status (MEDIA PONDERADA A FAZER)   
    count_fire++;          // counts to do the average math
    fire_position += servo_position;  // sum to do the average math
  }
}

void Extinguisher::moveServo(int from_position, int to_position) { // Moves servo from a certain position to another
  if ((from_position < to_position) && (to_position <= SERVO_LEFT_MAX)) {
    for(servo_position = from_position; servo_position <= to_position; servo_position++) {
      servo.write(servo_position);
      delay(15);
    }
  } else if (to_position >= SERVO_RIGHT_MAX) {
    for(servo_position = from_position; servo_position >= to_position; servo_position--) {
      servo.write(servo_position);
      delay(15);
    }
  }
}

void Extinguisher::moveServoSearchingFlame(int from_position, int to_position) {
  
  if ((from_position < to_position) && (to_position <= SERVO_LEFT_MAX)) {
    for(servo_position = from_position; servo_position <= to_position; servo_position++) {
      servo.write(servo_position);
      findFire();
      delay(40);
    }
  } else if (to_position >= SERVO_RIGHT_MAX) {
    for(servo_position = from_position; servo_position >= to_position; servo_position--) {
      servo.write(servo_position);
      findFire();
      delay(40);
    }
  }

}

void Extinguisher::searchFlame(int room_side) { // Goes to one extrem to another looking for fire

  count_fire = 0;
  fire_position = 0;

  if (room_side == LEFT_SIDE){
    moveServoSearchingFlame(SERVO_START, SERVO_START + LITTLE_SEARCH);
    moveServoSearchingFlame(SERVO_START + LITTLE_SEARCH, SERVO_LEFT_MAX);
    moveServoSearchingFlame(SERVO_LEFT_MAX, SERVO_START);
  }

  if (room_side == RIGHT_SIDE){
    moveServoSearchingFlame(SERVO_START, SERVO_START - LITTLE_SEARCH);
    moveServoSearchingFlame(SERVO_START - LITTLE_SEARCH, SERVO_RIGHT_MAX);
    moveServoSearchingFlame(SERVO_RIGHT_MAX, SERVO_START);
  }

  if (fire_exist){
    fire_position = fire_position/count_fire;  //Does the average of the positions where it saw fire
  }
}

void Extinguisher::extinguishFire() {   // Turns servo to where the flame is and right and left to extinguish when the pump is turned on
  
  pump->on();
  moveServo(SERVO_START, SERVO_START + SPREAD_DEGREES);
  moveServo(SERVO_START + SPREAD_DEGREES, SERVO_START - SPREAD_DEGREES);
  moveServo(SERVO_START - SPREAD_DEGREES, SERVO_START);
  pump->off();
  
}