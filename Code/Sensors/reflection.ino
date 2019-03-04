#define ANALOG_REF  A8
#define DIGITAL_REF 8

void setup() {
    pinMode(ANALOG_REF, INPUT);
    pinMode(DIGITAL_REF, INPUT);
    Serial.begin(9650);
}

int analog;
bool digital;

void readColor() {
    analog = analogRead(ANALOG_REF);
    digital = digitalRead(DIGITAL_REF);
    Serial.print(analog);
    Serial.print(" ");
    Serial.println(digital);
}
void loop() {
    readColor();
}