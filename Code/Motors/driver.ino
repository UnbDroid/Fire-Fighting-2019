/*--------------------------- Motor Driver Usage ---------------------------*/

#define PONTE_H_A 38
#define PONTE_H_B 39

#define MOT_PWM 9

#define ENC_A       2
#define ENC_B       3
#define ENC_COUNTS  4192

#define KP 0.5

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

float counts(float degree){
  return ((ENC_COUNTS * degree) / 360);
}

void doEncoderA() {
  noInterrupts();
  if(digitalRead(ENC_A) == HIGH){
    if(digitalRead(ENC_B) == LOW)
      encoder0Pos +=  1;
    else
      encoder0Pos -=  1;
  }else{
    if(digitalRead(ENC_B) == HIGH)
      encoder0Pos +=  1;
    else
      encoder0Pos -=  1;
  }
  interrupts();
}

void doEncoderB() {
  noInterrupts();
  encoder1Pos += 1;
  interrupts();
}

void startEncoder() {
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  // Encoder1
  attachInterrupt(digitalPinToInterrupt(ENC_A), doEncoderA, CHANGE);
  // Encoder2
  //attachInterrupt(digitalPinToInterrupt(ENC_B), doEncoderB, CHANGE);
}

void startDriver() {
  pinMode(PONTE_H_A, OUTPUT);
  pinMode(PONTE_H_B, OUTPUT);
  pinMode(MOT_PWM, OUTPUT);

  pinMode(ENC_A, OUTPUT);
  pinMode(ENC_B, OUTPUT);
}

void pid(){                                 // Por enquanto só tá implementado o proporcional
  pot = KP * (counts(3600) - encoder0Pos);
  Serial.println(encoder0Pos);
  if(pot > 0){
    digitalWrite(PONTE_H_A, HIGH);
    digitalWrite(PONTE_H_B, LOW);
    if(pot >= 255)
      analogWrite(MOT_PWM, 255);
    else
      analogWrite(MOT_PWM, pot);
  }else if(pot < 0){
    digitalWrite(PONTE_H_A, LOW);
    digitalWrite(PONTE_H_B, HIGH);
    pot = -pot;
    if(pot >= 255)
      analogWrite(MOT_PWM, 255);
    else
      analogWrite(MOT_PWM, pot);
  }else{
    digitalWrite(PONTE_H_A, HIGH);
    digitalWrite(PONTE_H_B, HIGH);
  }
}

void setup() {
  startDriver();
  startEncoder();
  Serial.begin(9600);
}

float pot;

void loop() {
  pid();
}