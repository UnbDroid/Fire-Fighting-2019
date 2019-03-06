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

void FlameSensor::update() { // Reads the flame sensor and updates the fire variables 
  //Fazer função para pegar a mediana!!!!
  fire_analog  = analogRead(PIN_AN_F);
  fire_digital = digitalRead(PIN_D7);
  Serial.print("Porta analogica: ");
  Serial.print(fire_analog);
  Serial.print(" Porta digital: ");
  Serial.print(fire_digital);
}

extinguisher fogo;

void findFire(){ // Checks if there is a fire and does the average of the servo position to know where the flame is
  //int lastfire_analog = fire_analog;
  UpdateFire();
  if (fire_analog <= FIRE) {   
    Serial.println("Achei fogo");
    fire_exist = true;       // updates the fire existence status (MEDIA PONDERADA A FAZER)   
    count_fire++;          // counts to do the average math
    fire_position += servo_position;  // sum to do the average math
  }
}

void movesServo(int from_position, int to_position) { // Moves servo from a certain position to another
  if ((from_position < to_position) && (to_position <= SERVO_LEFT_MAX)) {
    for(servo_position = from_position; servo_position <= to_position; servo_position++) {
      servoFront.write(servo_position);
      delay(15);
    }
  } else if (to_position >= SERVO_RIGHT_MAX) {
    for(servo_position = from_position; servo_position >= to_position; servo_position--) {
      servoFront.write(servo_position);
      delay(15);
    }
  }
}

int searchFlame() { // Goes to one extrem to another looking for fire

  count_fire = 0;
  fire_position = 0;
  for(servo_position = SERVO_START; servo_position <= SERVO_LEFT_MAX; servo_position++) {  // Moves servo slowly to left while looks for fire
    servoFront.write(servo_position);
    FindFire();
    delay(40);
  }
  
  for(servo_position = SERVO_LEFT_MAX; servo_position >= SERVO_RIGHT_MAX; servo_position--) {  // Moves servo slowly to right while looks for fire
      servoFront.write(servo_position);
      FindFire();
      delay(40);
  }
  for(servo_position = SERVO_RIGHT_MAX; servo_position <= SERVO_START; servo_position++) { // Moves servo slowly to center while looks for fire
      servoFront.write(servo_position);
      FindFire();
      delay(40);
  }
  if (fire_exist){
    fire_position = fire_position/count_fire;  //Does the average of the positions where it saw fire
  }
}

void extinguishFire() {   // Turns servo to where the flame is and right and left to extinguish when the pump is turned on
  MovesServo(SERVO_START, fire_position);
  //Turn on the pump
  for (int i = 0; i <= 5; i++){
    MovesServo(fire_position, fire_position + 10);
    MovesServo(fire_position + 10, fire_position - 10);
    MovesServo(fire_position - 10, fire_position);
  }
}