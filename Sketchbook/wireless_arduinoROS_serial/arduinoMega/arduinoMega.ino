#include <Encoder.h>
#include <Sabertooth.h>

float r_motorSpeed = 0.0;
float l_motorSpeed = 0.0;

Encoder leftEnc(2, 3);
Encoder rightEnc(20, 21);
Sabertooth saberTooth(128, Serial2);

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // serial comm. with nodeMCU
  Serial2.begin(9600); // serial comm. with motor driver
  
  leftEnc.write(0);
  rightEnc.write(0);
}

void loop() {
  Serial1.print('l');
  Serial1.print(leftEnc.read());
  Serial1.print('r');
  Serial1.print(rightEnc.read());
  
  while(!Serial1.available()){
    //wait
  }
  
  char identifier = Serial1.read();
  if(identifier == 'r'){ r_motorSpeed = Serial1.parseFloat(); }
  else if(identifier == 'l'){ l_motorSpeed = Serial1.parseFloat(); }
  else{ Serial1.read(); }

  identifier = Serial1.read();
  if(identifier == 'r'){ r_motorSpeed = Serial1.parseFloat(); }
  else if(identifier == 'l'){ l_motorSpeed = Serial1.parseFloat(); }
  else{ Serial1.read(); }
  
  Serial.println("-----------");
  Serial.print("R : ");Serial.println(r_motorSpeed);
  Serial.print("L : ");Serial.println(l_motorSpeed);
  Serial.println("-----------");

  saberTooth.motor(1,r_motorSpeed);
  saberTooth.motor(2,l_motorSpeed);
}
