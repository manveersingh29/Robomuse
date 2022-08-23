#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
class Sensor_Status
{
public:
  Sensor_Status()
  {
    //Subscriber for left Encoder
    encoder_left_sub_=n_.subscribe("/lwheel", 10, &Sensor_Status::encoder_left_callback, this);
    //Subscriber for right Encoder
    encoder_right_sub_=n_.subscribe("/rwheel", 10, &Sensor_Status::encoder_right_callback, this);
    //Subscriber for Hokuyo
    laser_front_sub_=n_.subscribe("/scan2", 10, &Sensor_Status::laser_front_callback, this);
    //Subscriber for RpLidar
    laser_back_sub_=n_.subscribe("/scan_filtered", 10, &Sensor_Status::laser_back_callback, this);
    //Subscriber IMU
    imu_sub_ = n_.subscribe("/imu", 10, &Sensor_Status::imu_callback, this);
    //Subscriber  cmd_vel
    cmd_vel_sub_=n_.subscribe("/robomuse_diff/cmd_vel",10, &Sensor_Status::cmd_vel_callback, this);

    //Publisher led_flag
    led_flag_pub_=n_.advertise<std_msgs::Int16>("led_flag",1);

  }

  //ed_flag Publisher
  void led_flag_publisher()
  {
    if(cmd_vel_flag ==1)
    {
      led_flag_value.data = cmd_vel_flag;
    }
    else if (cmd_vel_flag == 0)
    {
      led_flag_value.data = cmd_vel_flag;
    }
    if(led_flag_new != led_flag_old)
    {
      led_flag_new = led_flag_old;
      led_flag_pub_.publish(led_flag_value);
    }
  }

  //cmd_vel callback
  void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg)
  {
    if (cmd_vel_msg.linear.x != 0.0 || cmd_vel_msg.angular.z != 0)
    {
      cmd_vel_flag = 1;
      led_flag_old = 1;
    }
    else
    {
      cmd_vel_flag = 0;
      led_flag_old = 0;
    }
  }

  //LeftWheel callback
  void encoder_left_callback(const std_msgs::Int16& left_enc_msg)
  {
    encoder_left_flag = 1;
  }

  //RightWheel callback
  void encoder_right_callback(const std_msgs::Int16& right_enc_msg)
  {
    encoder_right_flag = 1;
  }

  //FrontLaser callback
  void laser_front_callback(const sensor_msgs::LaserScan& front_laser_msg)
  {
    laser_front_flag = 1;
  }

  //BackLaser callback
  void laser_back_callback(const sensor_msgs::LaserScan& back_laser_msg)
  {
    laser_back_flag = 1;
  }

  //Imu callback
  void imu_callback(const geometry_msgs::Vector3& imu_msg)
  {
    imu_flag  = 1;
  }

  void sensor_info(){
    if(laser_back_flag==1 & laser_front_flag==1 & encoder_right_flag==1 & encoder_left_flag==1)
    {
      ROS_INFO("All sensors are working");
    }
    if(laser_front_flag == 0)
    {
      ROS_INFO("Front LIDAR Sensor not working");
    }
    if(laser_back_flag == 0)
    {
      ROS_INFO("Back LIDAR Sensor not working");
    }
    if(encoder_right_flag == 0)
    {
      ROS_INFO("Right Encoder not working");
    }
    if(encoder_left_flag == 0)
    {
      ROS_INFO("Left Encoder not working");
    }
    laser_back_flag = 0;
    laser_front_flag = 0;
    encoder_right_flag = 0;
    encoder_left_flag = 0;
    imu_flag = 0;
  }

private:
  //ROS Initialisation
  ros::NodeHandle n_;
  ros::Publisher led_flag_pub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber encoder_left_sub_;
  ros::Subscriber encoder_right_sub_;
  ros::Subscriber laser_front_sub_;
  ros::Subscriber laser_back_sub_;
  ros::Subscriber cmd_vel_sub_;

  //ROS Message Variables
  sensor_msgs::Imu imu_msg;
  sensor_msgs::LaserScan front_laser;
  sensor_msgs::LaserScan back_laser;
  std_msgs::Int16 encoder_right;
  std_msgs::Int16 encoder_left;
  std_msgs::Int16 led_flag_value;

  //Flags for Sensor Callbacks
  int encoder_left_flag = 0;
  int encoder_right_flag = 0;
  int laser_front_flag = 0;
  int laser_back_flag = 0;
  int imu_flag = 0;
  int led_flag_old = 0;
  int led_flag_new = 0;
  int cmd_vel_flag = 0;


};//End of class Sensor_Status

int main(int argc, char **argv)
{
  //Initiate ROS Node
  ros::init(argc, argv, "sensor_status");

  //Initialise ROS Time
  ros::Time::init();
  ros::Rate rate(10);
  ros::AsyncSpinner spinner(10);
  spinner.start();

  //Object of class Sensor_Status
  Sensor_Status Status_Object;
  while(ros::ok()){
    Status_Object.sensor_info();
    Status_Object.led_flag_publisher();
    //ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
