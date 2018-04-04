#include <Wire.h>
#include <MPU6050.h>


MPU6050 mpu;

unsigned long timer = 0;
float timeStep = 0.01;

float pitch = 0;
float roll = 0;
float yaw = 0;

 void inicializaGiro() {
    long int t1,t0 = millis();
    while((!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)))// && (t1 - t0) < 3000 )
    {
      delay(500);
    }
      mpu.calibrateGyro();
      mpu.setThreshold(3);
      InicioDoGiro = 1;
}

void atualizaGiro() {
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  //pitch += norm.YAxis * timeStep;
  //roll += norm.XAxis * timeStep;
  yaw +=  norm.ZAxis * timeStep;
  delay((timeStep*1000) - (millis() - timer));
}

void setup() {
  Serial.begin(9600);
}
 
void loop() {
  atualizaGiro();  
  Serial.print("\nYaw: ");
  Serial.print(dist);
  delay(500);
}