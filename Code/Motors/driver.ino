// Code which integrates simple motor functions and gyro usage //

// ----------------------- Libraries ------------------------- //

// Gyro lib
#include "MPU9250.h"

// ----------------------- Constants ------------------------- //

// Time between gyro updates (millis)
#define TIME_STEP   20

// Pins used for left motors
#define LA_H_BRIDGE 34
#define LB_H_BRIDGE 35
#define LMOT_PWM 8

// Pins used for right motors
#define RA_H_BRIDGE 38
#define RB_H_BRIDGE 39
#define RMOT_PWM 9

// Pins used for encoders
#define L_ENC 2  // Left motor's encoder
#define R_ENC 3  // Right motor's encoder

// Encoder constants
#define ENC_COUNTS 560  // Encoder counts equivalent to 1 full rotation
#define ENC_OFFSET 11   // Offset to estabilize motor position

// Possible ways to rotate
#define FORWARD 1
#define BACKWARDS 0

// Constant used when converting a distance in cm to degrees
#define WHEEL_RADIUS 4

// --------------------- Global Variables -------------------- //

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 gyro(Wire,0x68);
int status;

// Angular speed variables in each axis
float radX    = 0;
float radY    = 0;
float radZ    = 0;

// Degree variation variables in each axis
float degreeX = 0; 
float degreeY = 0;
float degreeZ = 0;

// Variables used to store time in gyro's angular speed integration
unsigned long now;
unsigned long last_update = 0;

// Motor variables
volatile long lenc_pos  = 0;        // Left encoder's position
volatile long renc_pos  = 0;        // Right encoder's position
byte          l_way     = FORWARD;  // Left motor's rotation way
byte          r_way     = FORWARD;  // Right motor's rotation way
byte          l_pot     = 90;       // Left motor's power
byte          r_pot     = 90;       // Right motor's power

// ------------------------ Functions ------------------------ //

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

// The functions below increment or decrement the encoder counts depending on
// which way the motor is rotating. They use interruptions.

// For left motor's encoder
void DoEncoderL() {
    noInterrupts();
    if (l_way == FORWARD)
        lenc_pos += 1;
    else
        lenc_pos -= 1;
    interrupts();
}

// For right motor's encoder
void DoEncoderR() {
    noInterrupts();
    if (r_way == FORWARD)
        renc_pos += 1;
    else
        renc_pos -= 1;
    interrupts();
}

// As we are not using a controller to estabilize the motor's rotation
// when it reaches the position we commanded, inercia makes it rotate a
// little further than expected, creating the need of an offset.

// This algorithm helps us get a constant offset everytime the motor is
// asked to stop
void StopMotor(byte A_H_BRIDGE, byte B_H_BRIDGE) {
    digitalWrite(A_H_BRIDGE, HIGH);
    digitalWrite(B_H_BRIDGE, HIGH);
}

void StopMotors() {
    digitalWrite(RA_H_BRIDGE, HIGH);
    digitalWrite(RB_H_BRIDGE, HIGH);
    digitalWrite(LA_H_BRIDGE, HIGH);
    digitalWrite(LB_H_BRIDGE, HIGH);
}

// Resets encoders' position variable. Called before using these variables in
// order to prevent an overflow.
void ResetEncs() {
    lenc_pos = 0;
    renc_pos = 0;
}

// The functions below make the motor start rotating in the direction defined
// as FORWARDS.

// For left motor
void OnFwdL(byte pwr) {
    if (pwr >= 0) {
        l_way = FORWARD;
        digitalWrite(LA_H_BRIDGE, HIGH);
        digitalWrite(LB_H_BRIDGE, LOW);
    } else {
        pwr = -pwr;
        l_way = BACKWARDS;
        digitalWrite(LA_H_BRIDGE, LOW);
        digitalWrite(LB_H_BRIDGE, HIGH);
    }
    analogWrite(LMOT_PWM, pwr);
}

// For right motor
void OnFwdR(byte pwr) {
    if (pwr >= 0) {
        r_way = FORWARD;
        digitalWrite(RA_H_BRIDGE, LOW);
        digitalWrite(RB_H_BRIDGE, HIGH);
    } else {
        pwr = -pwr;
        r_way = BACKWARDS;
        digitalWrite(RA_H_BRIDGE, HIGH);
        digitalWrite(RB_H_BRIDGE, LOW);
    }
    analogWrite(RMOT_PWM, pwr);
}

// The functions below make the motor start rotating in the direction defined
// as BACKWARDS.

// For left motor
void OnRevL(byte pwr) { OnFwdL(-pwr); }

// For right motor
void OnRevR(byte pwr) { OnFwdR(-pwr); }

// Makes the motors rotate in the specified way until they reach the distance
// received as argument
void WalkCm(float distance, byte way) {
    // The line below calculates how many encoder counts the motor must rotate
    // to achieve the specified distance
    volatile long pos = Degrees2Counts(Dist2Degrees(distance));

    ResetEncs();
    if (way == FORWARD) {
        OnFwdL(l_pot);
        OnFwdR(r_pot);
    } else {
        OnRevL(l_pot);
        OnRevR(r_pot);
    }

    while (abs(lenc_pos) < pos || abs(renc_pos) < pos) {
        // Serial.print("Left: ");
        // Serial.print(lenc_pos);
        // Serial.print("\tRight: ");
        // Serial.println(renc_pos);
    }

    StopMotor(LA_H_BRIDGE, LB_H_BRIDGE);
    StopMotor(RA_H_BRIDGE, RB_H_BRIDGE);
}

// Gets gyro's raw readings (rad/s) and integrates them into angles.
// Angles are also converted from rad to degrees.
void UpdateGyro() {
  gyro.readSensor();
  degreeZ += (gyro.getGyroZ_rads() * TIME_STEP * 180) / (1000 * PI);
}

// Makes the robot rotate in its own axis the certain amount of
// degrees received as an argument. Rotation direction is controlled
// by the argument's sign.
void Turn(float degrees) {
  float offset = degreeZ;

  if (degrees > 0) {
    OnRevL();
    OnFwdR();
    while(degreeZ < (offset + degrees)){
      now = millis();
      if (now - last_update >= TIME_STEP) {
        UpdateGyro();
        last_update = now;
      }
    }

  } else {
    OnRevR();
    OnFwdL();
    while(degreeZ > (offset + degrees)){
      now = millis();
      if (now - last_update >= TIME_STEP) {
        UpdateGyro();
        last_update = now;
      }
    }
  }

  StopMotor(LA_H_BRIDGE, LB_H_BRIDGE);
  StopMotor(RA_H_BRIDGE, RB_H_BRIDGE);
}

// Encoders' setup function
void StartEncoders() {
    // Left encoder
    pinMode(L_ENC, INPUT);
    attachInterrupt(digitalPinToInterrupt(L_ENC), DoEncoderL, CHANGE);
    // Right encoder
    pinMode(R_ENC, INPUT);
    attachInterrupt(digitalPinToInterrupt(R_ENC), DoEncoderR, CHANGE);
}

// Motor Driver setup function
void StartDriver() {
    // Left motor
    pinMode(LA_H_BRIDGE, OUTPUT);
    pinMode(LB_H_BRIDGE, OUTPUT);
    pinMode(LMOT_PWM, OUTPUT);
    // Right motor
    pinMode(RA_H_BRIDGE, OUTPUT);
    pinMode(RB_H_BRIDGE, OUTPUT);
    pinMode(RMOT_PWM, OUTPUT);
}

// Gyro setup function
void StartGyro() {
  status = gyro.begin();
  if (status < 0)
    Serial.println("gyro initialization unsuccessful");
}

void setup() {
  StartDriver();
  StartEncoders();
  StartGyro();
  Serial.begin(9600);
}

enum {
    SQUARE,
    TRIANGLE
}

#define WAVEFORM SQUARE;

void loop() {
  delay(1000);
  Turn(90);
  delay(1000);
  Turn(-90);
}
