/*--------------------------- Water Pump Usage --------------------------*/

#define PUMP 8
 
void onPump(){
	digitalWrite(PUMP, HIGH);
}

void offPump(){
	digitalWrite(PUMP, LOW);
}

void setup() {
  pinMode(PUMP, OUTPUT);
}
 
void loop() {
	onPump();
	delay(1000);
	offPump();
	delay(3000);
}
