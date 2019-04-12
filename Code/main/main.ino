3// Trinity College Fire-Fighting Home Robot Contest
// Equipe DROID - University of Brasília, Brasília - DF, Brazil
// Authors: Daniel Bauchspiess, Gabriel Gonçalves, Lucas Faria, Pedro Saman, Rebeca Helen, Vanessa Rodrigues

// This is the main source code for the robot Atrasadinho. It contains the routines and functions responsible for arena navigation and candle extinguishing. All source code can be found and downloaded on https://github.com/UnbDroid/Fire-Fighting-2019/Code

// ------------------------------------------------ Macro Logic -------------------------------------------------- //

#define ROOM3 3
#define ROOM4 4

// --------------------------------------------------- Motors ---------------------------------------------------- //

// Neato motor's integrated wheel radius
#define WHEEL_RADIUS 4

// Motor rotation direction
#define FORWARD   1
#define BACKWARDS 0

// Motor encoder 
#define ENC_COUNTS 560  // Encoder counts equivalent to 1 full rotation

// Batery Level (important to be the real value)
#define BATTERY_LEVEL 14.8 // The best way to do this is to set a pin to read the actual value from the battery

// Neato motors nominal voltage
#define SATURATION_VALUE 14.4

// Motor voltage when the robot is turning 
#define TURN_TENSION 4

// PI motor controller constants
#define KP 0.0125
#define KI 0.015
#define KG 50

// === Left motor === //
#define LA_H_BRIDGE 32
#define LB_H_BRIDGE 34
#define LMOT_PWM    9
#define L_ENC       18  
volatile long lenc_pos = 0; // encoder's position
long left_speed = 0;        // motor speed
byte l_way = FORWARD;       // motor's rotation direction
float left_pwr_signal;
float left_integral = 0;
float left_error;
void doEncoderL();                  // increments or decrements encoder position. Called by interruption
void moveLeftMotor(float tension);  // sends received voltage to the motor. positive is forward, negative backwards

// === Right motor === //
#define RA_H_BRIDGE 38
#define RB_H_BRIDGE 40
#define RMOT_PWM    8
#define R_ENC       19
volatile long renc_pos = 0; // encoder's position
long right_speed = 0;       // motor speed
byte r_way = FORWARD;       // motor's rotation direction
float right_pwr_signal;
float right_integral = 0;
float right_error;
void doEncoderR();                  // increments or decrements encoder position. Called by interruption
void moveRightMotor(float tension); // sends received voltage to the motor. positive is forward, negative backwards

float relative_error = 0;
long now, last_update = 0;
float dt;
float initial_degree;

// === General functions === //

// Encoders' setup function
void startEncoders();



// Motor driver setup function
void startDriver();

// Calculates how many degrees the motor must rotate in order to achieve the distance received as argument
float dist2Counts(float distance);

// Sets the both motor H_Bridge to 11 to stop them
void stopMotors();

// Resets both encoders' position variables. Called before using these variables in order to prevent an overflow
void resetEncs();

// Resets both motors speeds
void resetSpeed();

// Detects if calculated motor voltage is above SATURATION_VALUE. Used in PI controller
void saturationDetector(float *left_pwr_signal, float *right_pwr_signal);

// Reads encoder position variation in time and calculates motor speed
void updateSpeed(float dt);

// Confesso que n sei o que faz n rs. @saman dá uma luz
void antiWindUp(float *left_integral,float *right_integral);

// PI motor controller. Makes the robot walk the received distance in the received speed.
// Positive distance is forward, negative is backwards.
void moveDistance(float speed,int distance);

// Makes the robot turn left if angle is negative, right if angle is negative
void turn(float angle);

// ---------------------------------------------------- Mic ------------------------------------------------------ //

#define MIC     0
#define MIC_LED 1

// --------------------------------------------------------------------------------------------------------------- //


// Calculates how many degrees the motor must rotate in order to achieve the distance received as argument
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

void moveDistance(float speed,int distance) {
    
  float pos;
  int initial_position;

  resetSpeed();

  initial_position = abs((lenc_pos + renc_pos)/2);

  pos = dist2Counts(distance);

  while(abs(initial_position - abs((lenc_pos+renc_pos)/2)) < abs(pos)){
      
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP){
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
  Serial.print(frontalUS());
  Serial.print("  ");
  Serial.print(leftFrontUS());
  Serial.print("  ");
  Serial.print(leftBackUS());
  Serial.print("  ");
  Serial.print(rightFrontUS());
  Serial.print("  ");
  Serial.println(rightBackUS());  
}

void readMic() {
  while(digitalRead(MIC) != HIGH);
  digitalWrite(MIC_LED, HIGH);
}

int startSide() {
  if(rightFrontUS() < 15 && rightBackUS() < 15)
    return ROOM3;
  return ROOM4;
}


void xablau() {
  while(US == Parede)
    move()
  while(US != Parede)
    move()
  calculaDistancia()
  daRe()
  turn()
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

void moveUS(float speed) {
    
  float left_pwr_signal,right_pwr_signal;
  float relative_error = 0,right_error,left_error;
  float left_integral = 0,right_integral = 0;
  long now, last_update = 0;
  float dt,pos;
  int initial_position;
  float initial_degree = degreeZ;

  resetSpeed();

  initial_position = abs((lenc_pos + renc_pos)/2);

  pos = dist2Counts(distance);

  while(abs(initial_position - abs((lenc_pos+renc_pos)/2)) < abs(pos)){
      
    now = millis();
    dt = now - last_update;

    if(dt > TIME_STEP){
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
  }
  stopMotors();
}

void startToMiddle() {

}

void setup() {
  startDriver();
  startEncoders();
  startGyro();
  Serial.begin(9600);
  readMic();
}

void loop(){
  if(startSide() == ROOM3)
    startToRoom3();
}