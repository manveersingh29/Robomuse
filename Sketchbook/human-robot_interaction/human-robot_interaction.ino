#include <ros.h>
#include <Kangaroo.h>
#include <std_msgs/Float32.h>

#define sensor_1 A8
#define sensor_2 A9

float wheelSizeFactor = 1.01638; // Left wheel is smaller
float vel_to_cmd = 64.77456; // conversion factor for Kangaroo x2

KangarooSerial  K(Serial2);
KangarooChannel K_leftMotor(K, '1');
KangarooChannel K_rightMotor(K, '2');

ros::NodeHandle nh;

std_msgs::Float32 current_lmotor;
std_msgs::Float32 current_rmotor;

ros::Publisher left_motor_current("current_lmotor", &current_lmotor);
ros::Publisher right_motor_current("current_rmotor", &current_rmotor);

float lmotor_cmd_old = 0, rmotor_cmd_old = 0;
float lmotor_cmd_new = 0, rmotor_cmd_new = 0;

void cb_lmotor_cmd(const std_msgs::Float32& motor_cmd){
  lmotor_cmd_new = motor_cmd.data;
  if(lmotor_cmd_new <=33){
    if(lmotor_cmd_new != lmotor_cmd_old){
      lmotor_cmd_old = lmotor_cmd_new * vel_to_cmd * wheelSizeFactor;
      K_leftMotor.s(lmotor_cmd_old);
    }
  }else{
    lmotor_cmd_old = 33 * vel_to_cmd * wheelSizeFactor;
    K_leftMotor.s(lmotor_cmd_old);
  }
}

void cb_rmotor_cmd(const std_msgs::Float32& motor_cmd){
  rmotor_cmd_new = motor_cmd.data;
  if(rmotor_cmd_new <=33){
    if(rmotor_cmd_new != rmotor_cmd_old){
      rmotor_cmd_old = rmotor_cmd_new * vel_to_cmd;
      K_rightMotor.s(rmotor_cmd_old);
    }
  }else{
    rmotor_cmd_old = 33 * vel_to_cmd;
    K_rightMotor.s(rmotor_cmd_old);
  }
}

ros::Subscriber<std_msgs::Float32> left_motor_commands("lmotor_cmd", &cb_lmotor_cmd);
ros::Subscriber<std_msgs::Float32> right_motor_commands("rmotor_cmd", &cb_rmotor_cmd);

void setup() {
  pinMode(sensor_1,INPUT);
  pinMode(sensor_2,INPUT);
  
  Serial2.begin(38400);
  
  K_leftMotor.start();
  K_leftMotor.home().wait();
  K_rightMotor.start();
  K_rightMotor.home().wait();

  nh.getHardware()->setBaud(38400);

  nh.initNode();
  nh.advertise(left_motor_current);
  nh.advertise(right_motor_current);
  nh.subscribe(left_motor_commands);
  nh.subscribe(right_motor_commands);
  while(!nh.connected()){
    nh.spinOnce();
  }
}

float current_1 = 0, current_2 = 0;

void loop() {
  senseCurrent(&current_1, &current_2);
  current_lmotor.data = current_1;
  current_rmotor.data = current_2;
  left_motor_current.publish(&current_lmotor);
  right_motor_current.publish(&current_rmotor);
  nh.spinOnce();
}


void senseCurrent(float* current_1, float* current_2){
  float accumulatedAnalogReading_1 = 0;
  float accumulatedAnalogReading_2 = 0;
  for(int i=0;i<5;i++){
    accumulatedAnalogReading_1 += analogRead(sensor_1);
    accumulatedAnalogReading_2 += analogRead(sensor_2);
    delay(2);
  }
  float analogReading_1 = accumulatedAnalogReading_1/5;
  float analogReading_2 = accumulatedAnalogReading_2/5;
  float voltage_1 = (analogReading_1 / 1024.0) * 5000; // mVolts
  float voltage_2 = (analogReading_2 / 1024.0) * 5000;
  *current_1 = (voltage_1 - 2500) / 66; // 30 Amps module
  *current_2 = (voltage_2 - 2500) / 66;
  return;
}
