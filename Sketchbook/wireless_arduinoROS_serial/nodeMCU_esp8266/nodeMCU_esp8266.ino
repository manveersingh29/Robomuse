#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

/*  ESP AP Config.  */
#ifndef APSSID
#define APSSID "ESP_AP"
#define APPSK  "ROSserial@nodeMCU"
#endif

const char *ssid = APSSID;
const char *password = APPSK;

/*  ROS  */
IPAddress server(192,168,4,2);
const uint16_t serverPort = 11411;

ros::NodeHandle nh;

void messageCb_rmotor(const std_msgs::Float32& motorSpeed){
  Serial.print('r');
  Serial.print(motorSpeed.data);
}

void messageCb_lmotor(const std_msgs::Float32& motorSpeed){
  Serial.print('l');
  Serial.print(motorSpeed.data);
}

ros::Subscriber<std_msgs::Float32> sub_rmotor("rwheel_vtarget", messageCb_rmotor);
ros::Subscriber<std_msgs::Float32> sub_lmotor("lwheel_vtarget", messageCb_lmotor);
std_msgs::Int32 leftEncVal;
std_msgs::Int32 rightEncVal;
ros::Publisher pub_leftEnc("leftEncData", &leftEncVal);
ros::Publisher pub_rightEnc("rightEncData", &rightEncVal);

void setup() {
  ESP.eraseConfig();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  delay(1000);

  Serial.begin(9600);
  
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub_rmotor);
  nh.subscribe(sub_lmotor);
  nh.advertise(pub_leftEnc);
  nh.advertise(pub_rightEnc);
}

void loop() {
  if(nh.connected()){
    char identifier = Serial.read();
    if(identifier == 'r'){ rightEncVal.data = Serial.parseInt(); pub_rightEnc.publish(&rightEncVal);}
    else if(identifier == 'l'){ leftEncVal.data = Serial.parseInt(); pub_leftEnc.publish(&leftEncVal);}
    else{ Serial.read(); }
    
    identifier = Serial.read();
    if(identifier == 'r'){ rightEncVal.data = Serial.parseInt(); pub_rightEnc.publish(&rightEncVal);}
    else if(identifier == 'l'){ leftEncVal.data = Serial.parseInt(); pub_leftEnc.publish(&leftEncVal);}
    else{ Serial.read(); }
  }
  else {
    // Don't publish unless connected.
  }
  nh.spinOnce();
  delay(10);
}
