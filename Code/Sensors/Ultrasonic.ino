/*--------------------------- Ultrasonic Sensor Reading --------------------------
  ------------ reference: https://playground.arduino.cc/Code/NewPing -------------*/

#include <NewPing.h>

#define PIN_TRIG 4
#define PIN_ECHO 5
#define MAX_DIST 200	// Distance reading functions return 0 if the read value is greater than this one
 
NewPing us(PIN_TRIG, PIN_ECHO, MAX_DIST);

float dist;

void setup() {
  Serial.begin(9600);
}
 
void loop() {
  dist = us.convert_cm(us.ping_median(5)); // Takes the median of 5 values (more accurate)	
  Serial.print("\nDistancia em cm: ");
  Serial.print(dist);
  delay(500);
}