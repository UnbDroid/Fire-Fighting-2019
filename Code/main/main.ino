// Trinity College Fire-Fighting Home Robot Contest
// Equipe DROID - University of Brasília, Brasília - DF, Brazil
// Authors: Daniel Bauchspiess, Gabriel Gonçalves, Lucas Faria, Pedro Saman, Rebeca Helen, Vanessa Rodrigues

// This is the main source code for the robot Atrasadinho. It contains the routines and functions responsible for arena navigation and candle extinguishing. All source code can be found and downloaded on https://github.com/UnbDroid/Fire-Fighting-2019/Code

// ------------------------------------------------ Macro Logic -------------------------------------------------- //

#define ROOM3 3
#define ROOM4 4


// int startSide() {
//   if(rightFrontUS() < 15 && rightBackUS() < 15)
//     return ROOM3;
//   return ROOM4;
// }

void setup() {
  startDriver();
  startEncoders();
  startGyro();
  Serial.begin(9600);
  readMic();
}

void loop(){
  if(startSide() == ROOM3)
    startToRoom3();
}