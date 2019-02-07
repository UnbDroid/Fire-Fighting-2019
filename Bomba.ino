int a=1;
const int bombaPin =  5;  
const int Chama = 4;
 
void setup() {
  
  Serial.begin(9600);
  pinMode(bombaPin, OUTPUT);
  pinMode(Chama, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(true){

 while(digitalRead(Chama)== HIGH){
    digitalWrite(bombaPin, HIGH);
    }
    digitalWrite(bombaPin, LOW);
  
}
