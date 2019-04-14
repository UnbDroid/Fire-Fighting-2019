#include "motors.h"

// === Left motor === //
volatile long lenc_pos = 0; // encoder's position
long left_speed = 0;        // motor speed
byte l_way = FORWARD;       // motor's rotation direction
float left_pwr_signal;
float left_integral = 0;
float left_error;

// === Right motor === //
volatile long renc_pos = 0; // encoder's position
long right_speed = 0;       // motor speed
byte r_way = FORWARD;       // motor's rotation direction
float right_pwr_signal;
float right_integral = 0;
float right_error;

// === Controller variables === //
float relative_error = 0;
long now, last_update = 0;
float dt;
float initial_degree;

// Calculates how many degrees the motor must rotate in order to achieve the distance received as argument
float dist2Counts(float distance) {
  return ((ENC_COUNTS*distance)/(2*PI*WHEEL_RADIUS));
}

// Calculates which distance was travelled by the robot when the encoders mark the received amount of counts
float counts2Dist(float counts) {
  return ((2*PI*WHEEL_RADIUS*ENC_COUNTS)/counts);
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

// The functions below make the motor start with the power disired
// For left motor
void moveLeftMotor(float tension) {
  int pwm;

  Serial.println(lenc_pos);
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
  if (lenc_pos > 500)
    digitalWrite(27, HIGH);
  else if (lenc_pos < -100)
    digitalWrite(27, HIGH);
}

// For right motor
void moveRightMotor(float tension) {
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

void stopLeftMotor() {
  digitalWrite(LA_H_BRIDGE, HIGH);
  digitalWrite(LB_H_BRIDGE, HIGH);
  digitalWrite(LA_H_BRIDGE, HIGH);
  digitalWrite(LB_H_BRIDGE, HIGH);
}

void stopRightMotor() {
  digitalWrite(RA_H_BRIDGE, HIGH);
  digitalWrite(RB_H_BRIDGE, HIGH);
  digitalWrite(RA_H_BRIDGE, HIGH);
  digitalWrite(RB_H_BRIDGE, HIGH);
}

// Resets all PI controller variables.Used everytime controller is used 
void startMotors() {
  // Left encoder
  pinMode(L_ENC, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENC), doEncoderL, CHANGE);
  // Right encoder
  pinMode(R_ENC, INPUT);
  attachInterrupt(digitalPinToInterrupt(R_ENC), doEncoderR, CHANGE);

  // Left motor
  pinMode(LA_H_BRIDGE, OUTPUT);
  pinMode(LB_H_BRIDGE, OUTPUT);
  pinMode(LMOT_PWM, OUTPUT);
  // Right motor
  pinMode(RA_H_BRIDGE, OUTPUT);
  pinMode(RB_H_BRIDGE, OUTPUT);
  pinMode(RMOT_PWM, OUTPUT);
}

// Set the both motor H_Bridge to 11 as to stop them
void stopMotors() {
  digitalWrite(LA_H_BRIDGE, HIGH);
  digitalWrite(LB_H_BRIDGE, HIGH);
  digitalWrite(RA_H_BRIDGE, HIGH);
  digitalWrite(RB_H_BRIDGE, HIGH);
}

void resetController(){
  left_speed = 0;
  right_speed = 0;
  relative_error = 0;
  left_integral = 0;
  right_integral = 0;
  last_update = 0;
  initial_degree = degreeZ;
  lenc_pos = 0;
  renc_pos = 0;
}

void saturationDetector(float *left_pwr_signal, float *right_pwr_signal) {
  
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

void updateSpeed(float dt) {
  
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

void antiWindUp(float *left_integral,float *right_integral) {

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

void controller(float speed) {
  dt = dt/1000;
  last_update = now;
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

void moveDistance(float speed, int distance) {
  float pos = dist2Counts(distance);
  int initial_position = abs((lenc_pos + renc_pos)/2);

  resetController();

  while(abs(initial_position - abs((lenc_pos+renc_pos)/2)) < abs(pos)) {
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP)
      controller(speed);
  }
  stopMotors();
}

void turn(float angle) {
  
  static long nowT;
  static long last_updateT = 0;

  float offset = degreeZ;
  
  if (angle > 0) {
    moveRightMotor(-TURN_TENSION);
    moveLeftMotor(TURN_TENSION);
    while(degreeZ < (offset + angle)){
      nowT = millis();
      if (nowT - last_updateT >= TIME_STEP) {
        updateGyro();
        last_updateT = nowT;
      }
    }

  } else {
    moveRightMotor(TURN_TENSION);
    moveLeftMotor(-TURN_TENSION);
    while(degreeZ > (offset + angle)){
      nowT = millis();
      if (nowT - last_updateT >= TIME_STEP) {
        updateGyro();
        last_updateT = nowT;
      }
    }
  }
  stopMotors();
}