/*--------------------------- Motor Driver Usage ---------------------------*/

// Pins used for motors
#define PONTE_H_A 38
#define PONTE_H_B 39
#define MOT_PWM   9

// Encoder constants
#define ENC_A       2     // Pin
#define ENC_COUNTS  560   // Encoder counts equivalent to 1 full rotation
#define ENC_OFFSET  11    // Offset to estabilize motor position

// Possible ways to rotate
#define FORWARD     1
#define BACKWARDS   0

// Constant used when converting a distance in cm to degrees
#define WHEEL_RADIUS 4

volatile long enc_pos = 0;
byte way              = FORWARD;
byte pot              = 127;

// Calculates how many degrees the motor must rotate in order to
// achieve the distance received as argument
float Dist2Degrees(float distance) {
  return (180 * distance) / (PI * WHEEL_RADIUS);
}

// Calculates how many encoder counts are equivalent to a rotation
// which meets the degrees received as argument 
long Degrees2Counts(float degrees) {
  return round((ENC_COUNTS * degrees) / 360) - ENC_OFFSET;
}

// Increments or decrements the encoder counts depending on which way
// the motor is rotating 
void DoEncoder() {
  noInterrupts();
  if(way == FORWARD)
    enc_pos += 1;
  else
    enc_pos -= 1;
  interrupts();
}

// As we are not using a controller to estabilize the motor's rotation
// when it reaches the position we commanded, inercia makes it rotate a little
// further than expected, creating the need of an offset.

// This algorithm helps us get a constant offset everytime the motor is asked to stop
void StopMotor() {
  digitalWrite(PONTE_H_A, HIGH);
  digitalWrite(PONTE_H_B, HIGH);
  digitalWrite(PONTE_H_A, HIGH);
  digitalWrite(PONTE_H_B, HIGH);
  digitalWrite(PONTE_H_A, HIGH);
  digitalWrite(PONTE_H_B, HIGH);
}

// Makes the motor rotate in the specified way until it reaches the distance
// received as argument
void WalkCm(float distance, byte way) {
  long pos = Degrees2Counts(Dist2Degrees(distance));
  enc_pos  = 0;

  if (way == FORWARD) {
    digitalWrite(PONTE_H_A, HIGH);
    digitalWrite(PONTE_H_B, LOW);
    analogWrite(MOT_PWM, pot);  
  } else {
    digitalWrite(PONTE_H_A, LOW);
    digitalWrite(PONTE_H_B, HIGH);
    analogWrite(MOT_PWM, pot);
  }

  while(enc_pos < pos);
  StopMotor();
}

// Encoder setup function
void StartEncoder() {
  pinMode(ENC_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), DoEncoder, CHANGE);
}

// Motor Driver setup function
void StartDriver() {
  pinMode(PONTE_H_A, OUTPUT);
  pinMode(PONTE_H_B, OUTPUT);
  pinMode(MOT_PWM, OUTPUT);
}

void setup() {
  StartDriver();
  StartEncoder();
  Serial.begin(9600);
}

void loop() {
  WalkCm(10, FORWARD);
  delay(1000);
  WalkCm(10, BACKWARDS);
  delay(1000);
}