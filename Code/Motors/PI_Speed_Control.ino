                /*--------------------------- Motor Driver Usage ---------------------------*/

                // Motor constants
                #define left_negative_delta -1.152981118373276
                #define right_negative_delta -0.965522875816994
                #define left_positive_delta 1.035470972702807
                #define right_positive_delta 0.990034602076125

                // Batery Level
                #define batery_level 15.5

                // Saturation Value
                #define saturation_value 14.4

                // Period in micro seconds
                #define period 20000

                // PI Constants
                #define KP_L 0.0125
                #define KP_R 0.0125
                #define KI_L 0.015//0.0101412558869
                #define KI_R 0.015//0.0101412558869
                #define KE 0

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
                long left_speed = 0;   // Left motor speed
                long right_speed = 0;   // Right motor speed
                byte l_way = FORWARD;        // Left motor's rotation way
                byte r_way = FORWARD;        // Right motor's rotation way

                // Calculates how many degrees the motor must rotate in order to
                // achieve the distance received as argument
                float Dist2Counts(float distance) {
                    return ((ENC_COUNTS*distance)/(2*PI*WHEEL_RADIUS));
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

                void ResetSpeed(){
                    left_speed = 0;
                    right_speed = 0;
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
                    analogWrite(LMOT_PWM, (byte)round(pwm));
                }

                // For right motor
                void Move_Right_Motor(int tension) {
                    float pwm;

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
                    analogWrite(RMOT_PWM, (byte)round(pwm));
                }

                void Inverse_Function(float *left_pwr_signal,float *right_pwr_signal){
                    
                    if(*left_pwr_signal != 0){
                        if(*left_pwr_signal > 0){
                            *left_pwr_signal+= left_positive_delta;
                        }else{
                            *left_pwr_signal+= left_negative_delta;
                        }
                    }

                    if(*right_pwr_signal != 0){
                        if(*right_pwr_signal > 0){
                            *right_pwr_signal+= right_positive_delta;
                        }else{
                            *right_pwr_signal+= right_negative_delta;
                        }
                    }
                }

                void Saturation_Detector(float *left_pwr_signal, float *right_pwr_signal){
                    
                    if(*left_pwr_signal > saturation_value){    
                        *left_pwr_signal = saturation_value;
                    }else if(*left_pwr_signal < -saturation_value){
                        *left_pwr_signal = -saturation_value;
                    }

                    if(*right_pwr_signal > saturation_value){    
                        *right_pwr_signal = saturation_value;
                    }else if(*right_pwr_signal < -saturation_value){
                        *right_pwr_signal = -saturation_value;
                    }

                }

                void update_vel(float dt){
                    static long left_old_enc = 0,right_old_enc = 0, left_old_speed = 0, right_old_speed = 0;
                    long left_aux,right_aux;
                    long local_lenc, local_renc;
                    float alpha = 0.8;

                    local_lenc = lenc_pos;
                    local_renc = renc_pos;

                    left_speed = alpha*((local_lenc - left_old_enc)/dt) + (1 - alpha)*(left_old_speed);
                    right_speed = alpha*((local_renc - right_old_enc)/dt) + (1 - alpha)*(right_old_speed);
                    
                    left_old_speed = left_speed;
                    right_old_speed = right_speed;
                    left_old_enc = local_lenc;
                    right_old_enc = local_renc;
                }

                // void AntiWindUp(float *left_integral,float *right_integral,float *left_pwr_signal, float *right_pwr_signal,float left_error,float right_error){
                    
                //     if(*left_pwr_signal > saturation_value || *left_pwr_signal < -saturation_value){
                //         *left_integral = 0;
                //         *left_pwr_signal = KP_L*left_error;
                //     }
                //     if(*right_pwr_signal > saturation_value || *right_pwr_signal < -saturation_value){
                //         *right_integral = 0;
                //         *right_pwr_signal = KP_R*right_error;
                //     }
                    
                // }

                //Solução porca ;(
                void AntiWindUp(float *left_integral,float *right_integral){

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

                void PI_Speed_Control(float speed,int distancia){
                    
                    float left_pwr_signal,right_pwr_signal;
                    float relative_error = 0,right_error,left_error;
                    float left_integral = 0,right_integral = 0;
                    long now, lastupdate = 0;
                    float dt,pos;
                    
                    ResetSpeed();
                    ResetEncs();

                    pos = Dist2Counts(distancia);
                    Serial.print(pos);
                    Serial.print(" ");
                    Serial.println(speed);

                    while(abs(lenc_pos) < abs(pos)){
                        
                        now = micros();
                        dt = now - lastupdate;
                        if(dt > period){
                            dt = dt/1000000;
                            lastupdate = now;
                            update_vel(dt);

                            Serial.print(left_speed);
                            Serial.print(" ");
                            Serial.println(right_speed);

                            //relative_error = KE*( left_speed - right_speed);
                            
                            left_error = speed - left_speed - relative_error;
                            right_error = speed - right_speed + relative_error;
                            
                            //Serial.print(left_error);
                            //Serial.print(" ");
                            //Serial.println(right_error);
                            
                            left_integral = left_integral + left_error * dt; 
                            right_integral = right_integral + right_error * dt;     

                            //Posição para chamar essa função na solução porca
                            AntiWindUp(&left_integral,&right_integral);

                            left_pwr_signal = KP_L*left_error + KI_L*left_integral;
                            right_pwr_signal = KP_R*right_error + KI_R*right_integral;

                            //AntiWindUp(&left_integral,&right_integral,&left_pwr_signal,&right_pwr_signal,left_error,right_error);

                            //Serial.print(left_integral);
                            //Serial.print(" ");
                            //Serial.println(right_integral);      

                            //Serial.print(left_pwr_signal);
                            //Serial.print(" ");
                            //Serial.println(right_pwr_signal);           
                            //Inverse_Function(&left_pwr_signal,&right_pwr_signal);
                            Saturation_Detector(&left_pwr_signal,&right_pwr_signal);

                            Move_Left_Motor(left_pwr_signal);
                            Move_Right_Motor(right_pwr_signal);
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
                    delay(2000);
                    PI_Speed_Control(460,50);
                    StopMotor(LA_H_BRIDGE, LB_H_BRIDGE);
                    StopMotor(RA_H_BRIDGE, RB_H_BRIDGE);
                    delay(2000);
                    PI_Speed_Control(-460,-50);
                    StopMotor(LA_H_BRIDGE, LB_H_BRIDGE);
                    StopMotor(RA_H_BRIDGE, RB_H_BRIDGE);
                }