#include "controller.h"

const float  Motor::BATTERY_LEVEL = 15.1;
const bool   Motor::FORWARD       = true;
const bool   Motor::BACKWARDS     = false;
const bool   Motor::LEFT          = true;
const bool   Motor::RIGHT         = false;

Motor::Motor(int _a_pin, int _b_pin, int _pwm_pin) {
  a_pin   = _a_pin;
  b_pin   = _b_pin;
  pwm_pin = _pwm_pin;

  pinMode(a_pin, OUTPUT);
  pinMode(b_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
}

bool Motor::getDirection() {
  return direction;
}

long Motor::getEncoder() {
  return encoder;
}

void Motor::sumEncoder() {
  encoder++;
}

void Motor::subEncoder() {
  encoder--;
}

void Motor::move(int voltage, bool side) {
  int pwm;

  pwm = voltage*255/BATTERY_LEVEL;

  if (pwm >= 0) {
    if(side == LEFT)
      direction = FORWARD;
    else 
      direction = BACKWARDS;
    digitalWrite(a_pin, HIGH);
    digitalWrite(b_pin, LOW);
  } else {
    pwm = -pwm;
    if(side == RIGHT) 
      direction = FORWARD;
    else
      direction = BACKWARDS;
    digitalWrite(a_pin, LOW);
    digitalWrite(b_pin, HIGH);
  }
  analogWrite(pwm_pin, (byte)round(pwm));
}

void Motor::stop() {
  digitalWrite(a_pin, HIGH);
  digitalWrite(b_pin, HIGH);
}

const double Controller::SATURATION_VALUE  = 14.4;
const double Controller::KP                = 0.0125;
const double Controller::KI                = 0;
const double Controller::KG                = 0;
const int    Controller::TIME_STEP         = 20;
const int    Controller::ENC_COUNTS        = 560;
const int    Controller::WHEEL_RADIUS      = 4;
const int    Controller::TURN_TENSION      = 8;
      long   Controller::left_old_enc      = 0;
      long   Controller::right_old_enc     = 0;
      long   Controller::left_old_speed    = 0;
      long   Controller::right_old_speed   = 0;
      long   Controller::left_speed        = 0;
      long   Controller::right_speed       = 0;

float Controller::dist2Counts(float distance) {
  return ((ENC_COUNTS*distance)/(2*PI*WHEEL_RADIUS));
}

void Controller::resetSpeed() {
  left_speed  = 0;
  right_speed = 0;
}

void Controller::saturationDetector(float *left_pwr_signal, float *right_pwr_signal) {
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

void Controller::updateSpeed(float dt) {
  long left_aux,right_aux;
  long local_lenc, local_renc;
  float alpha = 0.9;

  local_lenc = left_motor->getEncoder();
  local_renc = right_motor->getEncoder();

  left_speed = alpha*((local_lenc - left_old_enc)/dt) + (1 - alpha)*(left_old_speed);
  right_speed = alpha*((local_renc - right_old_enc)/dt) + (1 - alpha)*(right_old_speed);
  
  left_old_speed = left_speed;
  right_old_speed = right_speed;
  left_old_enc = local_lenc;
  right_old_enc = local_renc;
}

void Controller::antiWindUp(float *left_integral,float *right_integral) {
  // Ver com o Saman o que é esse 260
  
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

void Controller::updateGyro() {
  gyro->readSensor();
  degreeZ += (gyro->getGyroZ_rads() * TIME_STEP * 180) / (1000 * PI);
}

void Controller::move(float speed, int distance) {
  float left_pwr_signal,right_pwr_signal;
  float relative_error = 0,right_error,left_error;
  float left_integral = 0,right_integral = 0;
  long  now, lastupdate = 0;
  float dt,pos;
  int   initial_position;
  float initial_degree = degreeZ;

  this->resetSpeed();

  initial_position = abs((left_motor->getEncoder() + right_motor->getEncoder())/2);

  pos = dist2Counts(distance);

  while(abs(initial_position - abs((left_motor->getEncoder() + right_motor->getEncoder())/2)) < abs(pos)){
      
    now = millis();
    dt = now - lastupdate;

    if(dt > TIME_STEP) {
      dt = dt/1000;
      lastupdate = now;
      this->updateSpeed(dt);
    
      relative_error = KG*(degreeZ - initial_degree);

      left_error = speed - left_speed - relative_error;
      right_error = speed - right_speed + relative_error;
            
      left_integral = left_integral + left_error * dt; 
      right_integral = right_integral + right_error * dt;     

      this->antiWindUp(&left_integral, &right_integral);

      left_pwr_signal = KP*left_error + KI*left_integral;
      right_pwr_signal = KP*right_error + KI*right_integral;

      this->saturationDetector(&left_pwr_signal, &right_pwr_signal);

      left_motor->move(left_pwr_signal, Motor::LEFT);
      right_motor->move(-right_pwr_signal, Motor::RIGHT);
      
      this->updateGyro();
      Serial.print("left: ");
      Serial.println(left_speed);
      Serial.print("right: ");
      Serial.println(right_speed);
    }
  }
}

void Controller::stop() {
  left_motor->stop();
  right_motor->stop();
  left_motor->stop();
  right_motor->stop();
  left_motor->stop();
  right_motor->stop();
}

void Controller::turn(float angle) {
  static unsigned long now;
  static unsigned long last_update = 0;

  float offset = degreeZ;
  
  if (angle > 0) {
    right_motor->move(TURN_TENSION, Motor::RIGHT);
    left_motor->move(TURN_TENSION, Motor::LEFT);
    while(degreeZ < (offset + angle)){
      now = millis();
      if (now - last_update >= TIME_STEP) {
        updateGyro();
        last_update = now;
      }
    }

  } else {
    right_motor->move(-TURN_TENSION, Motor::RIGHT);
    left_motor->move(-TURN_TENSION, Motor::LEFT);
    while(degreeZ > (offset + angle)){
      now = millis();
      if (now - last_update >= TIME_STEP) {
        updateGyro();
        last_update = now;
      }
    }
  }
  stop();
}

void Controller::begin() {
  gyro->begin();
}