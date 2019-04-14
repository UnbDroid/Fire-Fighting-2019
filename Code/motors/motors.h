#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"
#include "gyro.h"

// Neato motor's integrated wheel radius
#define WHEEL_RADIUS 4

// Motor rotation direction
#define FORWARD   1
#define BACKWARDS 0

// Encoder counts equivalent to 1 full rotation
#define ENC_COUNTS 560  

// Batery Level (important to be the real value)
#define BATTERY_LEVEL 14.8 // The best way to do this is to set a pin to read the actual value from the battery

// Neato motors nominal voltage
#define SATURATION_VALUE 14.4

// Motor voltage when the robot is turning 
#define TURN_TENSION 4

// PI mo-tor controller constants
#define KP 0.0125
#define KI 0.015
#define KG 50

// === Left motor === //
#define LA_H_BRIDGE 40
#define LB_H_BRIDGE 38
#define LMOT_PWM    8
#define L_ENC       19
void doEncoderL();                  // increments or decrements encoder position. Called by interruption
void moveLeftMotor(float tension);  // sends received voltage to the motor. positive is forward, negative backwards

// === Right motor === //
#define RA_H_BRIDGE 34
#define RB_H_BRIDGE 32
#define RMOT_PWM    9
#define R_ENC       18
void doEncoderR();                  // increments or decrements encoder position. Called by interruption
void moveRightMotor(float tension); // sends received voltage to the motor. positive is forward, negative backwards

// === Left motor === //
extern volatile long lenc_pos; // encoder's position
extern long left_speed;        // motor speed
extern byte l_way;       // motor's rotation direction
extern float left_pwr_signal;
extern float left_integral;
extern float left_error;

// === Right motor === //
extern volatile long renc_pos; // encoder's position
extern long right_speed;       // motor speed
extern byte r_way;       // motor's rotation direction
extern float right_pwr_signal;
extern float right_integral;
extern float right_error;

// === Controller variables === //
extern float relative_error;
extern long now, last_update;
extern float dt;
extern float initial_degree;

// === General functions === //

// For left motor's encoder
void doEncoderL();

// For right motor's encoder
void doEncoderR();

// The functions below make the motor start with the power disired
// For left motor
void moveLeftMotor(float tension);

// For right motor
void moveRightMotor(float tension);

void stopLeftMotor();

void stopRightMotor();

// Encoders' setup function
void startMotors();

// Calculates how many degrees the motor must rotate in order to achieve the distance received as argument
float dist2Counts(float distance);

float counts2Dist(float counts);

// Resets all PI controller variables.Used everytime controller is used 
void resetController();

// Sets the both motor H_Bridge to 11 to stop them
void stopMotors();

// Detects if calculated motor voltage is above SATURATION_VALUE. Used in PI controller
void saturationDetector(float *left_pwr_signal, float *right_pwr_signal);

// Reads encoder position variation in time and calculates motor speed
void updateSpeed(float dt);

// Confesso que n sei o que faz n rs. @saman dá uma luz
void antiWindUp(float *left_integral, float *right_integral);

void controller(float speed);

// PI motor controller. Makes the robot walk the received distance in the received speed.
// Positive distance is forward, negative is backwards.
void moveDistance(float speed, int distance);

// Makes the robot turn left if angle is negative, right if angle is negative
void turn(float angle);

#endif // MOTORS_H