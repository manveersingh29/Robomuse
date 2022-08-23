#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


float orientation_x;
float orientation_y;
float orientation_z;
float orientation_w;
float angular_velocity_x;
float angular_velocity_y;
float angular_velocity_z;
float linear_acceleration_x;
float linear_acceleration_y;
float linear_acceleration_z;
int flag;



void imu_reciever(const sensor_msgs::Imu& imu_msg)
{
  orientation_x=imu_msg.orientation.y;
  orientation_y=imu_msg.orientation.x;
  orientation_z=-imu_msg.orientation.z;
  orientation_w=imu_msg.orientation.w;
  angular_velocity_x=imu_msg.angular_velocity.x;
  angular_velocity_y=imu_msg.angular_velocity.y;
  angular_velocity_z=imu_msg.angular_velocity.z;
  linear_acceleration_x=imu_msg.linear_acceleration.x;
  linear_acceleration_y=imu_msg.linear_acceleration.y;
  linear_acceleration_z=imu_msg.linear_acceleration.z;

  flag= flag +1 ;

  printf("Value of flag is:%i\n",flag );

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu_rater");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("imu_high", 100, imu_reciever);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_in", 100);

  ros::Rate rate(10);


  sensor_msgs::Imu imu;

  ros::Time measurement_time = ros::Time::now();

  imu.header.stamp = measurement_time;
  imu.header.frame_id= "imu_link";
  imu.linear_acceleration_covariance[0] = 0;
  imu.linear_acceleration_covariance[4] = 0;
  imu.linear_acceleration_covariance[8] = 0;

  imu.angular_velocity_covariance[0] = -1;
  imu.angular_velocity_covariance[4] = 0;
  imu.angular_velocity_covariance[8] = 0;

  imu.orientation_covariance[0] = 0;
  imu.orientation_covariance[4] = 0;
  imu.orientation_covariance[8] = 0;

  if(flag>10){

    imu.orientation.x=orientation_x;
    imu.orientation.y=orientation_y;
    imu.orientation.z=orientation_z;
    imu.orientation.w=orientation_w;

    imu.angular_velocity.x = angular_velocity_x;
    imu.angular_velocity.y = angular_velocity_y;
    imu.angular_velocity.z = angular_velocity_z;

    imu.linear_acceleration.x = linear_acceleration_x;
    imu.linear_acceleration.y = linear_acceleration_y;
    imu.linear_acceleration.z = linear_acceleration_z;

    imu_pub.publish(imu);
    //rate.sleep();
    flag=0;
  }
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
