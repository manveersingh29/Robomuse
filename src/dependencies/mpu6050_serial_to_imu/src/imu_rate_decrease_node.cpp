#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic to publish
    pub_ = n_.advertise<sensor_msgs::Imu>("/imu_in", 10);
    //Topic to subscribe
     sub_ = n_.subscribe("/imu_high", 10, &SubscribeAndPublish::callback, this);

  }

  void callback(const sensor_msgs::Imu& imu_msg)
  {

    ros::Time measurement_time = ros::Time::now();
    imu.header.frame_id = imu_msg.header.frame_id;
    imu.header.stamp = measurement_time;
    imu.orientation.x = (imu_msg.orientation.x);
    imu.orientation.y = (imu_msg.orientation.y);
    imu.orientation.z = (imu_msg.orientation.z);
    imu.orientation.w = imu_msg.orientation.w;
    imu.linear_acceleration.x = imu_msg.linear_acceleration.x;
    imu.linear_acceleration.y = imu_msg.linear_acceleration.y;
    imu.linear_acceleration.z = imu_msg.linear_acceleration.z;
    imu.angular_velocity.x = imu_msg.angular_velocity.x;
    imu.angular_velocity.y = imu_msg.angular_velocity.y;
    imu.angular_velocity.z = imu_msg.angular_velocity.z;
    imu.linear_acceleration_covariance[0] = 10000;
    imu.linear_acceleration_covariance[4] = 10000;
    imu.linear_acceleration_covariance[8] = 0;
    imu.angular_velocity_covariance[0] = -1;
    imu.angular_velocity_covariance[4] = 0;
    imu.angular_velocity_covariance[8] = 0;
    imu.orientation_covariance[0] = 1000;
    imu.orientation_covariance[4] = 1000;
    imu.orientation_covariance[8] = 0.01;
    //ROS_INFO("Subscribed value %f",imu.orientation.x);

  }

  void imu_publish(){
      pub_.publish(imu);
      //ROS_INFO("Published value %f",imu.orientation.x);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  sensor_msgs::Imu imu;
  int flag;
};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "imu_rate_decrease_node");
  //initialise ros time
  ros::Time::init();
  ros::Rate rate(10);

  //Object of class SubscribeAndPublish
  SubscribeAndPublish IMU_Object;
  while(ros::ok()){
    IMU_Object.imu_publish();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
