#include <NewPing.h>

#define PIN_TRIG 4
#define PIN_ECHO 5
#define MAX_DIST 200
 
NewPing us(PIN_TRIG, PIN_ECHO, MAX_DIST);

float dist;

void setup() {
  Serial.begin(9600);
  Serial.println("Lendo dados do sensor...");
}
 
void loop() {
  dist = us.convert_cm(us.ping_median(5));
  Serial.print("\nDistancia em cm: ");
  Serial.print(dist);
  delay(500);
}