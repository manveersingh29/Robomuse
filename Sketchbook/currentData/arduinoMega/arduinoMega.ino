#include <Encoder.h>
#include <Sabertooth.h>

const int l_currentReader = A0;
const int  r_currentReader = A1;

float r_motorSpeed = 0.0;
float l_motorSpeed = 0.0;

Encoder leftEnc(2, 3);
Encoder rightEnc(20, 21);
Sabertooth saberTooth(128, Serial2);

void sendEncoderData(){
  Serial1.print(leftEnc.read());
  Serial1.print(rightEnc.read());  
}

void sendCurrentData(){
  Serial1.print(analogRead(l_currentReader));
  Serial1.print(analogRead(r_currentReader));   
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // serial comm. with nodeMCU
  Serial2.begin(9600); // serial comm. with motor driver
  
  leftEnc.write(0);
  rightEnc.write(0);
}

void loop() {
  
  while(!Serial1.available()){
    Serial1.print('s');
    /* encoder feedback */
    sendEncoderData();
    /* current feedback */
    sendCurrentData();
    delay(10);
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
  
  Serial.print('s');
  /* encoder feedback */
  sendEncoderData();
  /* current feedback */
  sendCurrentData();
}
