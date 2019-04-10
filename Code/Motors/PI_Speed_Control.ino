/*------------------------------------ Motor Speed Control Functions -----------------------------------------*/

// Comments:
// This code do the speed control for the robot motors using the gyroscope to control it angular position.
// At this time it does not do position control, then the precision is not 100% reliable.

// Usage:
// The function move receives two arguments: speed and distance disired. Positive is forward and
// negative is backwards. 
// The function stopMotors send 11 to both H_Bridge to stop them.

// Gyro lib
#include "MPU9250.h"
// US lib
#include <NewPing.h>

// Frontal US pins
#define FRONTAL_TRIG      49
#define FRONTAL_ECHO      39

// Left US pins 
#define LEFT_FRONT_TRIG   51
#define LEFT_FRONT_ECHO   41
#define LEFT_BACK_TRIG    53
#define LEFT_BACK_ECHO    43

// Right US pins
#define RIGHT_FRONT_TRIG  47
#define RIGHT_FRONT_ECHO  37  
#define RIGHT_BACK_TRIG   45
#define RIGHT_BACK_ECHO   35

#define MAX_DIST 200  // Distance reading functions return 0 if the read value is greater than this one

// Batery Level (important to be the real value)
#define BATTERY_LEVEL 14.8 // The best way to do this is to set a pin to read the actual value from the battery

// Saturation Value
#define SATURATION_VALUE 14.4

// Iteration step in milli seconds
#define TIME_STEP 20

// Set the turning speed 
#define TURN_TENSION 4

// PI Constants
#define KP 0.0125
#define KI 0.015
#define KG 50

// Pins used for left motors
#define LA_H_BRIDGE 32
#define LB_H_BRIDGE 34
#define LMOT_PWM 9

// Pins used for right motors
#define RA_H_BRIDGE 38
#define RB_H_BRIDGE 40
#define RMOT_PWM 8

// Pins used for encoders
#define L_ENC 18  // Left motor's encoder
#define R_ENC 19  // Right motor's encoder

// Encoder constants
#define ENC_COUNTS 560  // Encoder counts equivalent to 1 full rotation

// Possible ways to rotate
#define FORWARD 1
#define BACKWARDS 0

// Constant used when converting a distance in cm to degrees
#define WHEEL_RADIUS 4

NewPing frontal_us(FRONTAL_TRIG, FRONTAL_ECHO, MAX_DIST);
NewPing left_front_us(LEFT_FRONT_TRIG, LEFT_FRONT_ECHO, MAX_DIST);
NewPing left_back_us(LEFT_BACK_TRIG, LEFT_BACK_ECHO, MAX_DIST);
NewPing right_front_us(RIGHT_FRONT_TRIG, RIGHT_FRONT_ECHO, MAX_DIST);
NewPing right_back_us(RIGHT_BACK_TRIG, RIGHT_BACK_ECHO, MAX_DIST);

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 gyro(Wire,0x68);
int status;

// Degree variation variables in each axis
float degreeZ = 0;

volatile long lenc_pos = 0;  // Left encoder's position
volatile long renc_pos = 0;  // Right encoder's position
long left_speed = 0;   // Left motor speed
long right_speed = 0;   // Right motor speed
byte l_way = FORWARD;        // Left motor's rotation way
byte r_way = FORWARD;        // Right motor's rotation way

// Calculates how many degrees the motor must rotate in order to
// achieve the distance received as argument
float dist2Counts(float distance) {
  return ((ENC_COUNTS*distance)/(2*PI*WHEEL_RADIUS));
}

// The functions below increment or decrement the encoder counts depending on
// which way the motor is rotating. They use interruptions.

// For left motor's encoder
void doEncoderL() {
  if (l_way == FORWARD)
    lenc_pos += 1;
  else
    lenc_pos -= 1;
}

// For right motor's encoder
void doEncoderR() {
  if (r_way == FORWARD)
    renc_pos += 1;
  else
    renc_pos -= 1;
}

// Set the both motor H_Bridge to 11 as to stop them
void stopMotors() {
  digitalWrite(LA_H_BRIDGE, HIGH);
  digitalWrite(LB_H_BRIDGE, HIGH);
  digitalWrite(RA_H_BRIDGE, HIGH);
  digitalWrite(RB_H_BRIDGE, HIGH);
}

// Resets encoders' position variable. Called before using these variables in
// order to prevent an overflow.
void resetEncs() {
  lenc_pos = 0;
  renc_pos = 0;
}

void resetSpeed(){
  left_speed = 0;
  right_speed = 0;
}

// The functions below make the motor start with the power disired
// For left motor
void moveLeftMotor(float tension) {
  int pwm;

  pwm = tension*255/BATTERY_LEVEL;

  if (pwm >= 0) {
    l_way = FORWARD;
    digitalWrite(LA_H_BRIDGE, HIGH);
    digitalWrite(LB_H_BRIDGE, LOW);
  } else {
    pwm = -pwm;
    l_way = BACKWARDS;
    digitalWrite(LA_H_BRIDGE, LOW);
    digitalWrite(LB_H_BRIDGE, HIGH);
  }
  analogWrite(LMOT_PWM, (byte)round(pwm));
}

// For right motor
void moveRightMotor(int tension) {
  float pwm;

  pwm = tension*255/BATTERY_LEVEL;
  if (pwm >= 0) {
    r_way = FORWARD;
    digitalWrite(RA_H_BRIDGE, LOW);
    digitalWrite(RB_H_BRIDGE, HIGH);
  } else {
    pwm = -pwm;
    r_way = BACKWARDS;
    digitalWrite(RA_H_BRIDGE, HIGH);
    digitalWrite(RB_H_BRIDGE, LOW);
  }
  analogWrite(RMOT_PWM, (byte)round(pwm));
}

void saturationDetector(float *left_pwr_signal, float *right_pwr_signal){
  
  if(*left_pwr_signal > SATURATION_VALUE){    
    *left_pwr_signal = SATURATION_VALUE;
  }else if(*left_pwr_signal < -SATURATION_VALUE){
    *left_pwr_signal = -SATURATION_VALUE;
  }

  if(*right_pwr_signal > SATURATION_VALUE){    
    *right_pwr_signal = SATURATION_VALUE;
  }else if(*right_pwr_signal < -SATURATION_VALUE){
    *right_pwr_signal = -SATURATION_VALUE;
  }

}

void updateSpeed(float dt){
  
  static long left_old_enc = 0,right_old_enc = 0, left_old_speed = 0, right_old_speed = 0;
  long left_aux,right_aux;
  long local_lenc, local_renc;
  float alpha = 0.9;

  local_lenc = lenc_pos;
  local_renc = renc_pos;

  left_speed = alpha*((local_lenc - left_old_enc)/dt) + (1 - alpha)*(left_old_speed);
  right_speed = alpha*((local_renc - right_old_enc)/dt) + (1 - alpha)*(right_old_speed);
  
  left_old_speed = left_speed;
  right_old_speed = right_speed;
  left_old_enc = local_lenc;
  right_old_enc = local_renc;
}

void antiWindUp(float *left_integral,float *right_integral){

  if(*left_integral > 260){
    *left_integral = 260;
  }else if(*left_integral < -260){
    *left_integral = -260;
  }
  
  if(*right_integral > 260){
    *right_integral = 260;
  }else if(*right_integral < -260){
    *right_integral = -260;
  }

}

void moveDistance(float speed,int distance){
    
  float left_pwr_signal,right_pwr_signal;
  float relative_error = 0,right_error,left_error;
  float left_integral = 0,right_integral = 0;
  long now, lastupdate = 0;
  float dt,pos;
  int initial_position;
  float initial_degree = degreeZ;

  resetSpeed();

  initial_position = abs((lenc_pos + renc_pos)/2);

  pos = dist2Counts(distance);

  while(abs(initial_position - abs((lenc_pos+renc_pos)/2)) < abs(pos)){
      
    now = millis();
    dt = now - lastupdate;

    if(dt > TIME_STEP){
      dt = dt/1000;
      lastupdate = now;
      updateSpeed(dt);
    
      relative_error = KG*(degreeZ - initial_degree);

      left_error = speed - left_speed - relative_error;
      right_error = speed - right_speed + relative_error;
            
      left_integral = left_integral + left_error * dt; 
      right_integral = right_integral + right_error * dt;     

      antiWindUp(&left_integral,&right_integral);

      left_pwr_signal = KP*left_error + KI*left_integral;
      right_pwr_signal = KP*right_error + KI*right_integral;

      saturationDetector(&left_pwr_signal,&right_pwr_signal);

      moveLeftMotor(left_pwr_signal);
      moveRightMotor(right_pwr_signal);
      updateGyro();
    }
  }
  stopMotors();
}

void turn(float angle) {
  
  static long now;
  static long last_update = 0;

  float offset = degreeZ;
  
  if (angle > 0) {
    moveRightMotor(-TURN_TENSION);
    moveLeftMotor(TURN_TENSION);
    while(degreeZ < (offset + angle)){
      now = millis();
      if (now - last_update >= TIME_STEP) {
        updateGyro();
        last_update = now;
      }
    }

  } else {
    moveRightMotor(TURN_TENSION);
    moveLeftMotor(-TURN_TENSION);
    while(degreeZ > (offset + angle)){
      now = millis();
      if (now - last_update >= TIME_STEP) {
        updateGyro();
        last_update = now;
      }
    }
  }
  stopMotors();
}

// Gets gyro's raw readings (rad/s) and integrates them into angles (rad).
// Angles are also converted from rad to degrees. 
void updateGyro() {
  gyro.readSensor();
  degreeZ += (gyro.getGyroZ_rads() * TIME_STEP * 180) / (1000 * PI);
}

// Gyro setup function
void startGyro() {
  status = gyro.begin();
  if (status < 0)
    Serial.println("IMU initialization unsuccessful");
}

// Encoders' setup function
void startEncoders() {
  // Left encoder
  pinMode(L_ENC, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENC), doEncoderL, CHANGE);
  // Right encoder
  pinMode(R_ENC, INPUT);
  attachInterrupt(digitalPinToInterrupt(R_ENC), doEncoderR, CHANGE);
}

// Motor Driver setup function
void startDriver() {
  // Left motor
  pinMode(LA_H_BRIDGE, OUTPUT);
  pinMode(LB_H_BRIDGE, OUTPUT);
  pinMode(LMOT_PWM, OUTPUT);
  // Right motor
  pinMode(RA_H_BRIDGE, OUTPUT);
  pinMode(RB_H_BRIDGE, OUTPUT);
  pinMode(RMOT_PWM, OUTPUT);
}

unsigned int frontalUS() {
  return frontal_us.convert_cm(frontal_us.ping_median(10)); 
}
unsigned int leftFrontUS() {
  return left_front_us.convert_cm(left_front_us.ping_median(10)); 
}
unsigned int leftBackUS() {
  return left_back_us.convert_cm(left_back_us.ping_median(10)); 
}
unsigned int rightFrontUS() {
  return right_front_us.convert_cm(right_front_us.ping_median(10)); 
}
unsigned int rightBackUS() {
  return right_back_us.convert_cm(right_back_us.ping_median(10)); 
}

void printUS() {
  Serial.print(leftBackUS());
  Serial.print("  ");
  Serial.print(leftFrontUS());
  Serial.print("  ");
  Serial.print(rightFrontUS());
  Serial.print("  ");
  Serial.println(rightBackUS());
  Serial.print("  ");
  Serial.println(frontalUS());
}

// void decideStartSide() {
//   if(rightBackUS() <= 13 && leftFrontUS() > 13){
    
//   }
// }

void setup() {
  startDriver();
  startEncoders();
  startGyro();
  Serial.begin(9600);
}

void loop(){
  moveDistance(450,30);
  delay(1000);
  turn(90);
  delay(1000);
  turn(-90);
  delay(1000);
  moveDistance(450,-30);
  delay(1000);
}