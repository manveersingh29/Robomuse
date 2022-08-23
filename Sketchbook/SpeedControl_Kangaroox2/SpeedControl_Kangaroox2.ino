#include <Kangaroo.h>

KangarooSerial  K(Serial2);
KangarooChannel K1(K, '1');
KangarooChannel K2(K, '2');

void setup(){
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  
  Serial.begin(38400);
  Serial2.begin(38400);

  K1.start();
  K1.home().wait();
  K2.start();
  K2.home().wait();
  Serial.println("I am working");
}

int baseSpeed = 100;
float factor = 1.01638;

void loop(){
  delay(1000);
  
  K1.s(baseSpeed);
  K2.s(baseSpeed*factor);
  
  while(true){
    
//    long enc_val1 = K1.getP().value();
//    long enc_val2 = K2.getP().value();
//    Serial.print(-1*enc_val1);Serial.print(",");Serial.println(-1*enc_val2);
    
//    long vel1 = K1.getS().value();
//    long vel2 = K2.getS().value();
//    Serial.print(baseSpeed);Serial.print(",");Serial.print(vel1);Serial.print(",");Serial.println(vel2);

    float analogReading = analogRead(A0);
    float analogReading1 = analogRead(A1);
    float voltage = (analogReading/1024.0)*5000; // mVolts
    float voltage1 = (analogReading1/1024.0)*5000;
    float current = (voltage-2500)/66; // 30 Amps module
    float current1 = (voltage1-2500)/66;
    Serial.print(15);Serial.print(",");
    Serial.print(current);Serial.print(",");Serial.print(current1);
    Serial.print(",");Serial.println(-15);
    delay(5);
  }
}
