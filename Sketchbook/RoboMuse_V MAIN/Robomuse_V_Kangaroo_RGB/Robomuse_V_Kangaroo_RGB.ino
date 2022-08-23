#include <ros.h>
#include <Kangaroo.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#define GREEN_LED 3
#define RED_LED 4
#define BLUE_LED 5

// Independent mode channels on Kangaroo are, by default, '1' and '2'.
KangarooSerial  K(Serial2);
KangarooChannel K_left(K, '1');
KangarooChannel K_right(K, '2');

//Multipying factor for correcting the robot's trajectory(Difference in robot wheels)   
float factor = 1.01638;
//Multiplying factor to provide the right commands to Kangaroo(Angular vel(rad/s) changed to the corrected motor commands (1044.33*radius)) 
float vel_to_cmd=(64.77456);  

long millis1=0;
long left_enc_old=0;
long left_enc_new;
long right_enc_old=0;
long right_enc_new;

float left_motor_old=0;
float right_motor_old=0;
float left_motor_new=0;
float right_motor_new=0;

//*************************ROS SETUP*************************//
  ros::NodeHandle  nh;
  
  //Callback function for left motor
  void cb_left_motor ( const std_msgs::Float32& l_motor_msg){                   //Publishes command to the motor only if the value recieved is different from the previous one      
    left_motor_new=float(l_motor_msg.data);   
    if(left_motor_new < 33)                                                      //The value should'nt exceed 33rad/s(2m/s)
      {
        if(left_motor_new!=left_motor_old)
        {
        left_motor_old=left_motor_new*vel_to_cmd*factor;                         //Left motor provided with extra velocity(smaller diameter)
        K_left.s(left_motor_old);           
        }      
      }
  }
     
  //Callback function for right motor 
  void cb_right_motor( const std_msgs::Float32& r_motor_msg){                   //Publishes command to the motor only if the value recieved is different from the previous one 
    right_motor_new=r_motor_msg.data;
    if(right_motor_new < 33)                                                     //The value should'nt exceed 33rad/s(2m/s)
    {
      if(right_motor_new!=right_motor_old)
      {
      right_motor_old=right_motor_new*vel_to_cmd;
      K_right.s(right_motor_old);
      }     
    }
  }   

  void cb_rgb_flag( const std_msgs::Int16& flag_msg)
  {
    if(flag_msg.data == 1){
      pinMode(GREEN_LED,HIGH);
      pinMode(BLUE_LED,LOW);
      pinMode(RED_LED,LOW);      
    }
    if(flag_msg.data == 0)
    {
      pinMode(RED_LED,HIGH);
      pinMode(GREEN_LED,LOW);
      pinMode(BLUE_LED,HIGH); 
    }
  }

  //Motor Command Subscribers
  ros::Subscriber<std_msgs::Float32> l_motor("lmotor_cmd", &cb_left_motor);
  ros::Subscriber<std_msgs::Float32> r_motor("rmotor_cmd", &cb_right_motor);

  //RGB LED subscriber
  ros::Subscriber<std_msgs::Int16> led("led_flag", &cb_rgb_flag );

//**************************END**************************//
  
  //Encoder Data Variables
  std_msgs::Int16 l_msg;
  std_msgs::Int16 r_msg;
  
  //Encoder Data Publishers
  ros::Publisher left_encoder("lwheel", &l_msg);
  ros::Publisher right_encoder("rwheel", &r_msg);


void setup()
{
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
 
  pinMode(RED_LED,LOW);
  pinMode(GREEN_LED,HIGH);
  //Opposite behaviour of blue(low level triggered)
  pinMode(BLUE_LED,HIGH);  

//**********Kangaroo Setup**********//  
  Serial2.begin(38400);
  K_left.start();
  K_left.home().wait();  
  K_right.start();
  K_right.home().wait();

//***********ROS INITIALISATION**************//  
  nh.getHardware()->setBaud(38400);
  
  //Publishers & Subscribers Initialise  
  nh.initNode();
  nh.advertise(left_encoder);
  nh.advertise(right_encoder);
  nh.subscribe(l_motor);
  nh.subscribe(r_motor);
  nh.subscribe(led);
  while (!nh.connected())
  {
    nh.spinOnce();
  }
//*******************************************//
  millis1=millis();
  
}

void loop()
{
  
  //Condition applied to Publish Data at 10Hz
  if(millis()-millis1>100){
    millis1+=100;    
    left_enc_new=(long(K_left.getP().value())/factor);
    right_enc_new=long(K_right.getP().value());
    r_msg.data=(right_enc_new-right_enc_old);
    l_msg.data=(left_enc_new-left_enc_old);
    left_enc_old=left_enc_new;
    right_enc_old=right_enc_new;
    left_encoder.publish( &l_msg );
    right_encoder.publish( &r_msg );
  }
  nh.spinOnce();   
}
