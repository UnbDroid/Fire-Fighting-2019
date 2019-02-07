/*--------------------------- Ultrasonic Sensor Reading --------------------------
  ------------ reference: https://playground.arduino.cc/Code/NewPing -------------*/

#include <NewPing.h>

#define PIN_ECHO_1 53
#define PIN_TRIG_1 52
#define PIN_ECHO_2 51
#define PIN_TRIG_2 50
#define PIN_ECHO_3 49
#define PIN_TRIG_3 48
#define PIN_ECHO_4 47
#define PIN_TRIG_4 46
#define PIN_ECHO_5 45
#define PIN_TRIG_5 44 
#define MAX_DIST 200	// Distance reading functions return 0 if the read value is greater than this one
 
NewPing us_frontal(PIN_TRIG_1, PIN_ECHO_1, MAX_DIST);
NewPing us_left_front(PIN_TRIG_1, PIN_ECHO_1, MAX_DIST);
NewPing us_left_back(PIN_TRIG_1, PIN_ECHO_1, MAX_DIST);
NewPing us_right_front(PIN_TRIG_1, PIN_ECHO_1, MAX_DIST);
NewPing us_right_back(PIN_TRIG_1, PIN_ECHO_1, MAX_DIST);

float dist;

void setup() {
  Serial.begin(9600);
}
 
void loop() {
  dist = us_frontal.convert_cm(us.ping_median(5)); // Takes the median of 5 values (more accurate)	
  Serial.print("\nDistancia em cm: ");
  Serial.print(dist);
  delay(500);
}