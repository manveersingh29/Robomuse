#include <ros.h>
#include <Kangaroo.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

//LED PINS
#define RED_LED 2
#define GREEN_LED 3
#define BLUE_LED 4

//CURRENT SENSOR PINS
#define sensor_1 A4
#define sensor_2 A7

// Independent mode channels on Kangaroo are, by default, '1' and '2'.
KangarooSerial  K(Serial2);
KangarooChannel K_left(K, '2');
KangarooChannel K_right(K, '1');

//Right wheel dia = 124.21mm
//Left wheel dia = 122.03mm 
//Encoder resolution = 525ppr
//Multipying factor for correcting the robot's trajectory(Ratio of the wheel dia's)   
float factor =  1.01786446;
//Multiplying factor to provide the right commands to Kangaroo(Angular vel(rad/s) changed to the corrected motor commands) //1333.33 motor commands = 1m/s 
float vel_to_cmd=(82.8065);  

long millis1=0;
long left_enc_old=0;
long left_enc_new;
long right_enc_old=0;
long right_enc_new;
float right_vel;
float left_vel;

float left_motor_old=0;
float right_motor_old=0;
float left_motor_new=0;
float right_motor_new=0;

float current_1 = 0, current_2 = 0;

//*************************ROS SETUP*************************//

//ROS Node Handle
ros::NodeHandle  nh;

//ROS message variables (current) 
std_msgs::Float32 current_lmotor;
std_msgs::Float32 current_rmotor;

//ROS Publishers for sending current readings 
ros::Publisher left_motor_current("current_lmotor", &current_lmotor);
ros::Publisher right_motor_current("current_rmotor", &current_rmotor);

//Callback function for left motor
void cb_left_motor ( const std_msgs::Float32& l_motor_msg){                   //Publishes command to the motor only if the value recieved is different from the previous one      
  left_motor_new=float(l_motor_msg.data);   
  if(left_motor_new < 33)                                                      //The value shouldn't exceed 33rad/s(2m/s){Added extra safety} 
  {
    if(left_motor_new!=left_motor_old)
    {
      left_motor_old=left_motor_new*vel_to_cmd*factor;                         //Left motor provided with extra velocity(smaller wheel diameter)
      if(left_motor_old != 0){
        K_left.s(left_motor_old, 300);
      }else{
        K_left.powerDown();           
      }
    }      
  }
}

   
//Callback function for right motor  
void cb_right_motor( const std_msgs::Float32& r_motor_msg){                   //Publishes command to the motor only if the value recieved is different from the previous one 
  right_motor_new=r_motor_msg.data;
  if(right_motor_new < 33)                                                    //The value shouldn't exceed 33rad/s(2m/s){Added extra safety} 
  {
    if(right_motor_new!=right_motor_old)
    {
      right_motor_old=right_motor_new*vel_to_cmd;
      if(right_motor_old != 0){
        K_right.s(right_motor_old, 300);
      }else{
        K_right.powerDown();
      }
    }     
  }
}   

//Callback function for led's 
//  void cb_rgb_flag( const std_msgs::Int16& flag_msg)
//  {
//    if(flag_msg.data == 1){
//      digitalWrite(GREEN_LED,HIGH);
//      digitalWrite(BLUE_LED,LOW);
//      digitalWrite(RED_LED,LOW);      
//    }
//    if(flag_msg.data == 0)
//    {
//      digitalWrite(RED_LED,HIGH);
//      digitalWrite(GREEN_LED,LOW);
//      digitalWrite(BLUE_LED,HIGH); 
//    }
//  }

//ROS Subscribers for motor commands 
ros::Subscriber<std_msgs::Float32> l_motor("lmotor_cmd", &cb_left_motor);
ros::Subscriber<std_msgs::Float32> r_motor("rmotor_cmd", &cb_right_motor);

//ROS Subscriber for RGB LED 
//  ros::Subscriber<std_msgs::Int16> led("led_flag", &cb_rgb_flag );


//ROS messages for Data Variables (Encoders) 
std_msgs::Int16 l_msg;
std_msgs::Int16 r_msg;

//ROS Publishers for Encoder_ticks (generated through Kangaroo)
ros::Publisher left_encoder("lwheel", &l_msg);
ros::Publisher right_encoder("rwheel", &r_msg);

//ROS messages for Data Variables (Motors) 
std_msgs::Float32 right_vel_msg;
std_msgs::Float32 left_vel_msg;

//ROS Publishers for Wheel Velocities (generated through Kangaroo)
ros::Publisher left_velocity("lwheel_vel", &left_vel_msg);
ros::Publisher right_velocity("rwheel_vel", &right_vel_msg);

//*****************************END**********************************//


void setup()
{
  //Initialisation of the LED's 
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  //Initialisation of the Current Sensors  
  pinMode(sensor_1,INPUT);
  pinMode(sensor_2,INPUT);

  //LED's are low-level triggered
  digitalWrite(RED_LED,HIGH);
  digitalWrite(GREEN_LED,HIGH);
  digitalWrite(BLUE_LED,HIGH);  

//**********Kangaroo Setup**********//  
  Serial2.begin(38400);
  K_left.start();
  K_left.home().wait();  
  K_right.start();
  K_right.home().wait();

//***********ROS INITIALISATION**************//
  
  //Baud rate for rosserial communication 
  nh.getHardware()->setBaud(38400);
  
  
  //Initialisation Publishers & Subscribers   
  nh.initNode();
  nh.advertise(left_encoder);
  nh.advertise(right_encoder);
  nh.advertise(left_velocity);
  nh.advertise(right_velocity);
  nh.advertise(left_motor_current);
  nh.advertise(right_motor_current);
  nh.subscribe(l_motor);
  nh.subscribe(r_motor);
//nh.subscribe(led);

  //Keep ROS spinning if rosserial not connected 
  while (!nh.connected())
  { 
    nh.spinOnce();
  }

//  digitalWrite(RED_LED,HIGH);
//  digitalWrite(GREEN_LED,LOW);
//  digitalWrite(BLUE_LED,HIGH);  

  millis1=millis();

//***********************END***************************//  
}

void loop()
{
  
  senseCurrent(&current_1, &current_2);
  current_lmotor.data = -1*current_1;                      
  current_rmotor.data = current_2;
  //Current data publish 
  left_motor_current.publish(&current_lmotor);
  right_motor_current.publish(&current_rmotor);
  
  //Condition applied to Publish Data at 10Hz
  if(millis()-millis1>100){
    millis1+=100;
    //Calculating wheel encoder ticks and velocities(through Kangaroo Functions)     
    left_enc_new=(long(K_left.getP().value())/factor);
    right_enc_new=long(K_right.getP().value());
    left_vel=((K_left.getS().value())/factor);
    right_vel=(K_right.getS().value());

  //Change of LED colour during Robot Movement  
//    if(left_vel == 0 && right_vel == 0)
//    {      
//      digitalWrite(RED_LED,HIGH);
//      digitalWrite(GREEN_LED,LOW);
//      digitalWrite(BLUE_LED,HIGH); 
//    }
//    else
//    {      
//      digitalWrite(RED_LED,HIGH);
//      digitalWrite(GREEN_LED,LOW);
//      digitalWrite(BLUE_LED,LOW); 
//    }

    //Wheel encoder ticks and velocities publish 
    left_vel_msg.data=float(left_vel); 
    right_vel_msg.data=float(right_vel);    
    r_msg.data=(right_enc_new-right_enc_old);
    l_msg.data=(left_enc_new-left_enc_old);
    left_enc_old=left_enc_new;
    right_enc_old=right_enc_new;
    left_encoder.publish( &l_msg );
    right_encoder.publish( &r_msg );
    left_velocity.publish( &left_vel_msg);
    right_velocity.publish( &right_vel_msg);   
  }
  nh.spinOnce();   
}

void senseCurrent(float* current_1, float* current_2){
  float accumulatedAnalogReading_1 = 0;
  float accumulatedAnalogReading_2 = 0;
  //Average of the first 5 readings for each current sensor 
  for(int i=0;i<5;i++){
    accumulatedAnalogReading_1 += analogRead(sensor_1);
    accumulatedAnalogReading_2 += analogRead(sensor_2);
    delay(2);
  }
  float analogReading_1 = accumulatedAnalogReading_1/5;
  float analogReading_2 = accumulatedAnalogReading_2/5;
  //The PWM signal is converted into millivolts(ratio*max_voltage)
  float voltage_1 = (analogReading_1 / 1024.0) * 5000; // mVolts
  float voltage_2 = (analogReading_2 / 1024.0) * 5000;
  //Calculating current readings (30Amp current sensors used)
  *current_1 = (voltage_1 - 2500) / 66; // 30 Amps module
  *current_2 = (voltage_2 - 2500) / 66;
  return;
}
