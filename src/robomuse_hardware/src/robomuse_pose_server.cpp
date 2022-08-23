#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include "robomuse_hardware/manual_pose.h"


class Manual_Pose
{
public:
  Manual_Pose()
  {
    //Subscriber for Encoder Odometry
    encoder_odometry_sub_=n_.subscribe("/odom", 10, &Manual_Pose::encoder_odom_callback, this);
    //Subscriber for Encoder Odometry yaw Angle
    encoder_odom_yaw_sub_=n_.subscribe("/odom_yaw",10, &Manual_Pose::enc_odom_yaw_callback, this);
    //Publisher for manual movenment
    manual_cmd_vel_pub_=n_.advertise<geometry_msgs::Twist>("cmd_vel",10);
    //Service server for robot translation
    translation_service_= n_.advertiseService("manual_translation_X", &Manual_Pose::srv_translation, this);
    //Service server for robot rotation
    rotation_service_= n_.advertiseService("manual_rotation_Z", &Manual_Pose::srv_rotation, this);
  }

  //Callback function for Encoder Odometry_x
  void encoder_odom_callback(const nav_msgs::Odometry& odom_msg)
  {
    odom.pose.pose.position.x = odom_msg.pose.pose.position.x;
  }
  // Callback function for Encoder_Odometry_Yaw
  void enc_odom_yaw_callback(const std_msgs::Float32& odom_yaw_msg){
    odom_yaw.data = odom_yaw_msg.data;
  }

  //Robot Translation service function
  bool srv_translation(robomuse_hardware::manual_pose::Request &req, robomuse_hardware::manual_pose::Response &res)
  {
    x_new = req.position + odom.pose.pose.position.x;
    translation_flag = 1;
    ROS_INFO("Manual Robot Translation Service Ready");
    ROS_INFO("The new x_value =[%f]",x_new);
    return true;
  }

  //Robot Rotation service function
  bool srv_rotation(robomuse_hardware::manual_pose::Request &req, robomuse_hardware::manual_pose::Response &res)
  {
    rot_new = ((req.position*degrees_to_radian) + odom_yaw.data);
    rotation_flag = 1;
    ROS_INFO("Manual Robot Rotation Service Ready");
    ROS_INFO("The new heading angle of robot in radians =[%f]",rot_new);
    return true;
  }

  //Cmd_vel Publisher
  void manual_pose()
  {
    if(translation_flag == 1)
    {
      x_old = odom.pose.pose.position.x;
      if(x_old != x_new)
      {
        cmd_vel.linear.x = 0.25;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;
        //Speed decrease if distance left is less
        if(x_new - x_old <= 0.1)
        {
          cmd_vel.linear.x = 0.15;
        }
        //Stop the robot if the distance left is smaller that 3cm, the robot will translate for some distance because of Inertia
        if(x_new - x_old <= 0.02)
        {
          cmd_vel.linear.x = 0;
          translation_flag = 0;
        }
        manual_cmd_vel_pub_.publish(cmd_vel);
      }
    }
    if(rotation_flag ==1)
    {
      rot_old = odom_yaw.data;
      if(rot_old != rot_new)
      {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0.6;
        if(rot_new - rot_old <= 0.1)
        {
          cmd_vel.angular.z = 0.3;
        }
        if(rot_new - rot_old <= 0.02)
        {
          cmd_vel.angular.z = 0;
          rotation_flag = 0;
        }
        manual_cmd_vel_pub_.publish(cmd_vel);
      }
    }
  }



private:
  //ROS Initialisation
  ros::NodeHandle n_;
  //ROS Subscribers
  ros::Subscriber encoder_odometry_sub_;
  ros::Subscriber encoder_odom_yaw_sub_;
  //ROS Service's
  ros::ServiceServer translation_service_;
  ros::ServiceServer rotation_service_;
  //ROS Publisher
  ros::Publisher manual_cmd_vel_pub_;

  //ROS Message Variables
  nav_msgs::Odometry odom;
  std_msgs::Float32 odom_yaw;
  geometry_msgs::Twist cmd_vel;

  //Variables for rosservice
  int translation_flag = 0;
  int rotation_flag = 0;
  float req_translation = 0;
  float req_rotation = 0;

  float degrees_to_radian=(3.141519/180);
  //Variables for Publisher
  float x_old;
  float x_new;
  float rot_new;
  float rot_old;

};//End of class Manual_Pose

int main(int argc, char **argv)
{
  //Initiate ROS Node
  ros::init(argc, argv, "Manual_Pose_Service");

  //Initialise ROS Time
  ros::Time::init();
  //Initialise multi-threading
  ros::AsyncSpinner spinner(6);
  spinner.start();
  //Rate define as 10Hz
  ros::Rate rate(10);

  //Object of class Manual_Pose
  Manual_Pose pose;

  while(ros::ok()){
    pose.manual_pose();
    rate.sleep();
  }
  return 0;
}
