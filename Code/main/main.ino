// Trinity College Fire-Fighting Home Robot Contest
// Equipe DROID - University of Brasília, Brasília - DF, Brazil
// Authors: Daniel Bauchspiess, Gabriel Gonçalves, Lucas Faria, Pedro Saman, Rebeca Helen, Vanessa Rodrigues

// This is the main source code for the robot Atrasadinho. It contains the routines and functions responsible for arena navigation and candle extinguishing. All source code can be found and downloaded on https://github.com/UnbDroid/Fire-Fighting-2019/Code

void extinguishMovements(){
  for(int i = 0; i <= 5; i++){
    extinguisher.extinguishFire();
    moveDistance(300, -3); //moves slowly backwards
    extinguisher.extinguishFire();
    moveDistance(300, 6);
    extinguisher.extinguishFire();
    moveDistance(300, -3); //vale a pena verificar se ja está apagada?
  }
  
}

void searchRoom(int room_number ,int room_side){
  if ((room_number == 1) || ((room_number == 2) && (room_side == extinguisher.RIGHT_SIDE)) || (room_number == 3) || (room_number == 4)){
    extinguisher.searchFlame(extinguisher.RIGHT_SIDE);
    if (extinguisher.fire_exist){  //verificar se da pra usar a variavel assim
      //turn on LED
      moveDistance(450, 25); // valor para que todo o robo esteja dentro do quarto TESTAR
      turn(extinguisher.fire_position - 90); // verificar 2
      extinguisher.searchFlame(extinguisher.RIGHT_SIDE);//search the flame again
      turn(extinguisher.fire_position - 90); // turn to the flame
      //move close to the candle 
      extinguishMovements();
      //Turn of LED 
    }
  } else {
    extinguisher.searchFlame(extinguisher.LEFT_SIDE);
    if (extinguisher.fire_exist){  //verificar se da pra usar a variavel assim
      //turn on LED
      moveDistance(450, 25); // valor para que todo o robo esteja dentro do quarto TESTAR
      turn(extinguisher.fire_position - 90); // verificar 2
      extinguisher.searchFlame(extinguisher.LEFT_SIDE);//search the flame again
      turn(extinguisher.fire_position - 90); // turn to the flame
      //move close to the candle 
      extinguishMovements();
      //Turn of LED
    }
  }  
}

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