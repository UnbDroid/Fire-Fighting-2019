/*--------------------------- Motor Driver Usage ---------------------------*/

// Motor constants
#define left_positive_delta 1.0105
#define left_negative_delta -0.8804
#define right_positive_delta 1.0802
#define right_negative_delta -0.7013

// Batery Level
#define batery_level 15.6

// Step Size
#define dt 0.02

// PID Constants
#define KP_L 0.0224
#define KP_R 0.0219
#define KI_L 0.0916
#define KI_R 0.0762
#define KD_L 0.0011
#define KD_R 0.0011
#define KW 10
#define KE 18

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

// Possible ways to rotate
#define FORWARD 1
#define BACKWARDS 0

// Constant used when converting a distance in cm to degrees
#define WHEEL_RADIUS 4

volatile long lenc_pos = 0;  // Left encoder's position
volatile long renc_pos = 0;  // Right encoder's position
byte l_way = FORWARD;        // Left motor's rotation way
byte r_way = FORWARD;        // Right motor's rotation way

// Calculates how many degrees the motor must rotate in order to
// achieve the distance received as argument
float Dist2Degrees(float distance) {
    return (180 * distance) / (PI * WHEEL_RADIUS);
}

// Calculates how many encoder counts are equivalent to a rotation
// which meets the degrees received as argument
long Degrees2Counts(float degrees) {
    return round((ENC_COUNTS * degrees) / 360);
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

// Set the H_Bridge to 11 as to stop the motors
void StopMotor(byte A_H_BRIDGE, byte B_H_BRIDGE) {
    digitalWrite(A_H_BRIDGE, HIGH);
    digitalWrite(B_H_BRIDGE, HIGH);
}

// Resets encoders' position variable. Called before using these variables in
// order to prevent an overflow.
void ResetEncs() {
    lenc_pos = 0;
    renc_pos = 0;
}

// The functions below make the motor start with the power disired
// For left motor
void Move_Left_Motor(float tension) {
    int pwm;

    pwm = tension*255/batery_level;

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
    analogWrite(LMOT_PWM, pwm);
}

// For right motor
void Move_Right_Motor(int tension) {
    int pwm;

    pwm = tension*255/batery_level;
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
    analogWrite(RMOT_PWM, pwm);
}

void Inverse_Function(float *left_pwr_signal,float *right_pwr_signal){
    
    if(*left_pwr_signal != 0){
        if(*left_pwr_signal > 0){
            *left_pwr_signal+= left_positive_delta;
        }else{
            *left_pwr_signal+= left_negative_delta;
        }
    }else *left_pwr_signal = 0;

    if(*right_pwr_signal != 0){
        if(*right_pwr_signal > 0){
            *right_pwr_signal+= right_positive_delta;
        }else{
            *right_pwr_signal+= right_negative_delta;
        }
    }else *right_pwr_signal = 0;

}

void Saturation_Detector(float *left_pwr_signal, float *right_pwr_signal){
    
    if(*left_pwr_signal > 14.4){    
        *left_pwr_signal = 14.4;
    }else if(*left_pwr_signal < -14.4){
        *left_pwr_signal = -14.4;
    }

    if(*right_pwr_signal > 14.4){    
        *right_pwr_signal = 14.4;
    }else if(*right_pwr_signal < -14.4){
        *right_pwr_signal = -14.4;
    }

}

void PID_Control(float distance){
    
    byte left_direction, right_direction;
    float left_error = 0,right_error = 0;
    float left_pwr_signal,right_pwr_signal;
    float left_AntiWindUpSignal = 0,right_AntiWindUpSignal = 0;
    float left_Integral = 0,right_Integral = 0;
    float left_derivative,right_derivative, old_left_error, old_right_error;
    int initial_time,iteration_time;
    int relative_error;

    // The line below calculates how many encoder counts the motor must rotate
    // to achieve the specified distance
    volatile long pos = Degrees2Counts(Dist2Degrees(distance));

    ResetEncs();
    Serial.println(pos);
    Serial.println("L_Error L_derivative L_integral L_pwr L_AWU");
    while(abs(lenc_pos - pos) >= 1 || abs(renc_pos - pos) >= 1){
                
        initial_time = millis();

        // Get the relative error
        relative_error = KE*(lenc_pos - renc_pos);

        // Updates the old error for the derivative calculation
        old_left_error = left_error;
        old_right_error = right_error;

        // Calculates the error for each motor
        left_error = pos - lenc_pos - relative_error;
        right_error= pos - renc_pos + relative_error;

        Serial.print(left_error);
        Serial.print(" ");

        // Calculates the approximate derivative of the error for each motor
        left_derivative = KD_L*(left_error - old_left_error) / dt;
        right_derivative = KD_R*(right_error - old_right_error) / dt;

        Serial.print(left_derivative);
        Serial.print(" ");

        // Calculates the approximate integral of the error for each motor
        left_Integral = (left_Integral + KI_L*left_error + left_AntiWindUpSignal) * dt;
        right_Integral = (right_Integral + KI_R*right_error+ right_AntiWindUpSignal) * dt;

        Serial.print(left_Integral);
        Serial.print(" ");

        left_pwr_signal = (KP_L*left_error + left_Integral + left_derivative);
        right_pwr_signal= (KP_R*right_error + right_Integral + right_derivative);

        Serial.print(left_pwr_signal);
        Serial.print(" ");

        left_AntiWindUpSignal  = -left_pwr_signal;
        right_AntiWindUpSignal = -right_pwr_signal; 

        Inverse_Function(&left_pwr_signal,&right_pwr_signal);

        Saturation_Detector(&left_pwr_signal,&right_pwr_signal);

        right_AntiWindUpSignal = KW*(right_AntiWindUpSignal + right_pwr_signal);
        left_AntiWindUpSignal  = KW*(left_AntiWindUpSignal  + left_pwr_signal );

        Serial.printls(left_AntiWindUpSignal);
        //Serial.print(" ");
        // Serial.print(right_pwr_signal);
        // Serial.print(" ");
        // Serial.print(lenc_pos);
        // Serial.print(" ");
        // Serial.println(renc_pos);

        Move_Left_Motor(left_pwr_signal);
        Move_Right_Motor(right_pwr_signal);

        iteration_time = millis() - initial_time;
        if(iteration_time <20){
           delay(iteration_time); 
        }
    }
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

void loop(){
    Serial.println("Loop Start");
    delay(4000);
    PID_Control(100);
    StopMotor(LA_H_BRIDGE, LB_H_BRIDGE);
    StopMotor(RA_H_BRIDGE, RB_H_BRIDGE);
    PID_Control(-100);
}