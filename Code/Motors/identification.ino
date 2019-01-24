/*--------------------------- Motor Driver Usage ---------------------------*/


// Batery Level
#define batery_level 15.6

// Step Size
#define dt 0.002

// Waveform
#define SQUARE 0
#define TRIANGLE 1

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

volatile long lenc_pos = 0;  // Left encoder's position
volatile long renc_pos = 0;  // Right encoder's position
byte l_way = FORWARD;        // Left motor's rotation way
byte r_way = FORWARD;        // Right motor's rotation way
byte l_pot = 127;            // Left motor's power
byte r_pot = 127;            // Right motor's power

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
void OnFwdL(int pwr) {
    int potencia;
    if (pwr >= 0) {
        potencia = pwr;
        l_way = FORWARD;
        digitalWrite(LA_H_BRIDGE, HIGH);
        digitalWrite(LB_H_BRIDGE, LOW);
    } else {
        potencia = -pwr;
        l_way = BACKWARDS;
        digitalWrite(LA_H_BRIDGE, LOW);
        digitalWrite(LB_H_BRIDGE, HIGH);
    }
    analogWrite(LMOT_PWM, potencia);
}

// For right motor
void OnFwdR(int pwr) {
    int potencia;
    if (pwr >= 0) {
        potencia = pwr;
        r_way = FORWARD;
        digitalWrite(RA_H_BRIDGE, LOW);
        digitalWrite(RB_H_BRIDGE, HIGH);
    } else {
        potencia = -pwr;
        r_way = BACKWARDS;
        digitalWrite(RA_H_BRIDGE, HIGH);
        digitalWrite(RB_H_BRIDGE, LOW);
    }
    analogWrite(RMOT_PWM, potencia);
}

// The functions below make the motor start rotating in the direction defined
// as BACKWARDS.

// For left motor
void OnRevL(int pwr) { OnFwdL(-pwr); }

// For right motor
void OnRevR(int pwr) { OnFwdR(-pwr); }

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

void setup() {
    StartDriver();
    StartEncoders();
    Serial.begin(9600);
}

void loop() {
    float now;
    int pwr;
    int WAVEFORM = 1;
    int lastSerial = millis();
    
    Serial.println("Time PWM Left Right");
    delay(3500);
    if(WAVEFORM == SQUARE){
        pwr = 100;
        float lastFlip = millis();
        while (1) {
          now = millis();
          if (now - lastFlip > 3000) {
            lastFlip = now;
            pwr = -pwr;
            OnFwdL(pwr);
            OnFwdR(pwr);
          }
          if (now - lastSerial > 20) {
              Serial.print(now);
              Serial.print(" ");
              Serial.print(pwr);
              Serial.print(" ");
              Serial.print(lenc_pos);
              Serial.print(" ");
              Serial.print(renc_pos);
              Serial.print("\n");
              lastSerial = now;
          }
        }
    }else if( WAVEFORM == TRIANGLE){
        pwr = 0;
        bool dir = false;
        float lastIncrement = millis();
        while (1) {
            OnFwdR(pwr);
            OnFwdL(pwr);
            now = millis();
            if (now - lastIncrement > 15) {
                if (dir) {
                  Serial.println(" ");
                    pwr = pwr + 1;
                } else {
                  Serial.println(" ");
                    pwr = pwr - 1;
                }
                if (abs(pwr) >= 100) {
                    dir = !dir;
                }
                OnFwdR(pwr);
                OnFwdL(pwr);
            }
            if (now - lastSerial > 20) {
                Serial.print(now);
                Serial.print(" ");
                Serial.print(pwr);
                Serial.print(" ");
                Serial.print(lenc_pos);
                Serial.print(" ");
                Serial.print(renc_pos);
                Serial.print("\n");
                lastSerial = now;
            }
        }
    }
}