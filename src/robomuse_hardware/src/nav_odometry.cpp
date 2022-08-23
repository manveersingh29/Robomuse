#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
class nav_odometry
{
public:
  nav_odometry()
  {
    //Subscriber for Encoder Odometry
    enc_odom_sub_=n_.subscribe("robomuse_diff/odom", 10, &nav_odometry::encoder_odom_callback, this);
    //Subscriber for ekf_odom_combined
    ekf_odom_sub_=n_.subscribe("/robot_pose_ekf/odom_combined", 10, &nav_odometry::ekf_combined_callback, this);

    //Publisher for nav_odometryetry
    nav_odometry_pub_=n_.advertise<nav_msgs::Odometry>("/nav_odometry",1);

  }
  //Navigation Odometry Publisher
  void nav_odometry_publisher()
  {
      nav_odom.header = enc_odom.header;
      nav_odom.child_frame_id = enc_odom.child_frame_id;
      nav_odom.twist = enc_odom.twist;
      nav_odom.pose = ekf_odom.pose;
      nav_odometry_pub_.publish(nav_odom);
  }
  //Encoder Odometry Callback
  void encoder_odom_callback(const nav_msgs::Odometry& odom_msg)
  {
    enc_odom.header = odom_msg.header;
    enc_odom.child_frame_id = odom_msg.child_frame_id;
    enc_odom.twist = odom_msg.twist;
  }

  //Ekf_combined  callback
  void ekf_combined_callback(const geometry_msgs::PoseWithCovarianceStamped& ekf_msg)
  {
    ekf_odom.pose = ekf_msg.pose;
  }

private:
  //ROS Initialisation
  ros::NodeHandle n_;
  ros::Publisher nav_odometry_pub_;
  ros::Subscriber enc_odom_sub_;
  ros::Subscriber ekf_odom_sub_;

  //ROS Message Variables
  nav_msgs::Odometry nav_odom;
  nav_msgs::Odometry enc_odom;
  geometry_msgs::PoseWithCovarianceStamped ekf_odom;

};//End of class nav_odom

int main(int argc, char **argv)
{
  //Initiate ROS Node
  ros::init(argc, argv, "nav_odometry");

  //Initialise ROS Time
  ros::Time::init();
  ros::Rate rate(10);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  //Object of class nav_odometry
  nav_odometry Status_Object;
  while(ros::ok()){
    Status_Object.nav_odometry_publisher();
    rate.sleep();
  }

  return 0;
}
