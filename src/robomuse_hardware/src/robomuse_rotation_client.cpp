#include "ros/ros.h"
#include "robomuse_hardware/rotation_time.h"
#include <cstdlib>
#include <sstream>
#include <geometry_msgs/Twist.h>

float total_time;
float init_time;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robomuse_rotation_client");
  //if (argc != 3)
  //{
  //  ROS_INFO("usage: Determine the time required for the cmd_vel");
  //  return 1;
  //}

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robomuse_hardware::rotation_time>("rotate_z_time");
  ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("robomuse_diff/cmd_vel", 10);
  ros::Rate rate(10);

  geometry_msgs::Twist pos_vel;
  robomuse_hardware::rotation_time srv;

  srv.request.rotation = atoll(argv[1]);
  long int rotate_val = srv.request.rotation;
  init_time=0;
  if (client.call(srv))
  {
    system("rosnode kill robomuse5_teleop");
    total_time=float(srv.response.total_time);
    while(ros::ok()){
      ROS_INFO("Total time required: %f",total_time);
      pos_vel.linear.y=0;
      pos_vel.linear.z=0;
      pos_vel.angular.x=0;
      pos_vel.angular.y=0;
      pos_vel.angular.z=0;

      while(init_time < total_time){
        if(rotate_val > 0){
          init_time = init_time + 0.1;
          pos_vel.linear.x=0;
          pos_vel.linear.y=0;
          pos_vel.linear.z=0;
          pos_vel.angular.x=0;
          pos_vel.angular.y=0;
          pos_vel.angular.z=0.25;
          cmd_vel.publish(pos_vel);
          rate.sleep();
        }
        else{
          init_time = init_time + 0.1;
          pos_vel.linear.x=0;
          pos_vel.linear.y=0;
          pos_vel.linear.z=0;
          pos_vel.angular.x=0;
          pos_vel.angular.y=0;
          pos_vel.angular.z=-0.25;
          cmd_vel.publish(pos_vel);
          rate.sleep();
        }
      }

      pos_vel.angular.z=0;
      cmd_vel.publish(pos_vel);
      rate.sleep();
      ros::spinOnce();
      ROS_INFO("Rotation Completed, Teleop switched ON");
      system("rosrun teleop_twist_keyboard teleop.py");
      system("rosnode kill robomuse_rotation_client");



    }
    //ROS_INFO("Total time: [%ld]", (long int)srv.response.total_time);

  }
  else
  {
    ROS_ERROR("Failed to call service robomuse rotation");
    return 1;
  }

  return 0;
}
