#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include "SparkFun_BNO080_Arduino_Library.h"

BNO080 myIMU;

//////////ROS SETUP/////////////////////////

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);
float time1,time2,dt;
////////////////////////////////////////////

void setup()
{
  //Serial.begin(230400);
  //Serial.println();
  //Serial.println("BNO080 Read Example");

  Wire.begin();

  if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  time1 = millis();
  myIMU.enableGameRotationVector(100); //Send data update every 50ms
  myIMU.enableAccelerometer(100); //Send data update every 50ms //decrease this rate for the sake of more baud rate
  //Serial.println(F("Rotation vector enabled"));
  //Serial.println(F("Output in form i, j, k, real, accuracy"));
  
  imu_msg.header.frame_id="imu";
  imu_msg.orientation.x=0.0;
  imu_msg.orientation.y=0.0;
  imu_msg.orientation.z=0.0;
  imu_msg.orientation.w=0.0;
  
  imu_msg.angular_velocity.x=0.0;
  imu_msg.angular_velocity.y=0.0;
  imu_msg.angular_velocity.z=0.0;
  
                                        
  imu_msg.linear_acceleration.x=0.0;
  imu_msg.linear_acceleration.y=0.0;
  imu_msg.linear_acceleration.z=0.0;

  nh.getHardware()->setBaud(74880);      //Set according to the ros nodes rate(Hz).
  nh.initNode();
  nh.advertise(imu_pub);
  while (!nh.connected())
  {
    nh.spinOnce();
  }
 
}

void loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {

    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
    float x = myIMU.getAccelX();
    float y = myIMU.getAccelY();
    float z = -(myIMU.getAccelZ());
    time2 = millis();
    dt = time2 - time1; 
    //Serial.println(dt);
    if(dt > 10){
      //Serial.println(dt);
      imu_msg.header.stamp=nh.now();
      imu_msg.orientation.x=quatI;
      imu_msg.orientation.y=quatJ;
      imu_msg.orientation.z=quatK;
      imu_msg.orientation.w=quatReal;
      imu_msg.linear_acceleration.x=x;
      imu_msg.linear_acceleration.y=y;
      imu_msg.linear_acceleration.z=z;
      imu_pub.publish( &imu_msg );
      nh.spinOnce();
      time1 = time2;
      }
  }
}
