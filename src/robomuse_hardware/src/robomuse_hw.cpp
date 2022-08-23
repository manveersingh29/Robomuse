#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_state_controller/joint_state_controller.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>


class MyRobot : public hardware_interface::RobotHW
{
public:

  MyRobot()
  {
    //Connect and Register Joint State Interface for both the wheels
    hardware_interface::JointStateHandle state_handle_left("chassis_to_left_wheel", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_left);

    hardware_interface::JointStateHandle state_handle_right("chassis_to_right_wheel", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_right);

    registerInterface(&jnt_state_interface);

    //Connect and Register Joint Velocity Interface for both the wheels (receives motor commands(rad/s))
    hardware_interface::JointHandle vel_handle_left(jnt_state_interface.getHandle("chassis_to_left_wheel"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_left);

    hardware_interface::JointHandle vel_handle_right(jnt_state_interface.getHandle("chassis_to_right_wheel"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_right);

    registerInterface(&jnt_vel_interface);
  }

  //ROS Time functions
  ros::Time get_time()
  {
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
  }

  ros::Duration get_period()
  {
    return curr_update_time - prev_update_time;
  }

  //Robot velocity and Encoder ticks read function
  void read(const ros::Duration &period)
  {
     left_wheel_ticks = ema_filter_encoder(_wheel_angle[0], left_ticks_old);
     right_wheel_ticks = ema_filter_encoder(_wheel_angle[1], right_ticks_old);
     distance_left = (left_wheel_ticks*0.0114391007);                  //factor to change encoder ticks in metres
     distance_right = (right_wheel_ticks*0.0114391007);
     //Save the position and velocity for Joint States
       pos[0] += distance_left;
       vel[0] = left_wheel_vel;
       pos[1] += distance_right;
       vel[1] = right_wheel_vel;

       //Current encoder ticks saved as old ticks for the filters
       left_ticks_old = left_wheel_ticks;
       right_ticks_old = right_wheel_ticks;
  }

  //Function to publish motor commands in rad/s
  void write()
  {
    right_motor_cmd = ema_filter_motor(cmd[1], prev_r_motor);
    left_motor_cmd = ema_filter_motor(cmd[0], prev_l_motor);

    left_wheel_vel_msg.data = (left_motor_cmd);
    right_wheel_vel_msg.data = (right_motor_cmd);
    left_wheel_vel_pub_.publish(left_wheel_vel_msg);
    right_wheel_vel_pub_.publish(right_wheel_vel_msg);

    //Save current motor commands as old motor commands
    prev_r_motor = right_motor_cmd;
    prev_l_motor = left_motor_cmd;
  }


  //Function to reset encoder ticks during data loss
  void reset_encoder_ticks()
  {
    if (leftCallbackFlag == 0 && rightCallbackFlag == 0)
    {
      _wheel_angle[1] = 0;
      _wheel_angle[0] = 0;
    }
    leftCallbackFlag=0;
    rightCallbackFlag=0;
  }

  //Function to reset robot velocity during data loss
  void reset_wheel_velocity()
  {
    if (rightvelcallbackflag == 0 && leftvelcallbackflag == 0)
    {
      vel[0] = 0;
      vel[1] = 0;
    }
    rightvelcallbackflag=0;
    leftvelcallbackflag=0;
  }

private:
  ros::NodeHandle nh;

  ros::Time curr_update_time, prev_update_time;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  //Initialised with zero commands
  double cmd[2] = {0, 0};
  double pos[2] = {0, 0};
  double vel[2] = {0, 0};
  double eff[2] = {0, 0};
  double _wheel_angle[2] = {0, 0};

  std_msgs::Float32 left_wheel_old_vel_msg;
  std_msgs::Float32 right_wheel_old_vel_msg;
  std_msgs::Float32 left_wheel_vel_msg;
  std_msgs::Float32 right_wheel_vel_msg;

  //Read function Variables
  double left_wheel_ticks = 0;
  double right_wheel_ticks = 0;
  double distance_left  = 0;
  double distance_right = 0;

  //Write function Variables
  double right_motor_cmd = cmd[1];
  double left_motor_cmd = cmd[0];

  //Variables for ema_filter_motor function
  double prev_avg_motor=0;
  double prev_l_motor=0;
  double prev_r_motor=0;
  double alpha_motor = 0.45;

  //Variables for ema_filter_encoder function
  double prev_avg_enc=0;
  double left_ticks_old=0;
  double right_ticks_old=0;
  double alpha_enc = 0.6;
  int leftCallbackFlag = 0;
  int rightCallbackFlag = 0;
  int rightvelcallbackflag = 0;
  int leftvelcallbackflag = 0;
  double right_wheel_vel = 0;
  double left_wheel_vel = 0;


  //Callback function for Left wheel encoder ticks
  void leftWheelAngleCallback(const std_msgs::Int16& msg) {
    _wheel_angle[0] = msg.data;
    leftCallbackFlag = 1;
  }

  //Callback function for Right wheel encoder ticks
  void rightWheelAngleCallback(const std_msgs::Int16& msg) {
    _wheel_angle[1] = msg.data;
    rightCallbackFlag = 1;
  }

  //Callback function for Left wheel velocity
  void leftWheelVelCallback(const std_msgs::Float32& l_vel_msg) {
  left_wheel_vel = (l_vel_msg.data/1044.33);
    leftCallbackFlag = 1;
  }

  //Callback function for Right wheel velocity
  void rightWheelVelCallback(const std_msgs::Float32& r_vel_msg) {
    right_wheel_vel = (r_vel_msg.data/1044.33);
    rightCallbackFlag = 1;
  }


  //EMA Filter function for motor commands
  double ema_filter_motor(double motor_value, double prev_avg_motor){
    double ema_avg_motor=((1-alpha_motor)*prev_avg_motor + (alpha_motor*motor_value));
    return ema_avg_motor;
  }

  //EMA Filter function for encoder ticks (removal of noise)
  double ema_filter_encoder( double encoder_value, double prev_avg_enc ){
   double ema_avg_encoder=((1-alpha_enc)*prev_avg_enc + (alpha_enc*encoder_value));
   return ema_avg_encoder;
  }

  //Publishers for motor commands (rad/s)
  ros::Publisher left_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("lmotor_cmd", 10);
  ros::Publisher right_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("rmotor_cmd", 10);

  //Subscribers for encoder ticks and robot velocity
  ros::Subscriber left_wheel_angle_sub_ = nh.subscribe("lwheel", 10, &MyRobot::leftWheelAngleCallback, this);
  ros::Subscriber right_wheel_angle_sub_ = nh.subscribe("rwheel", 10, &MyRobot::rightWheelAngleCallback, this);
  ros::Subscriber right_wheel_vel_sub_ = nh.subscribe("rwheel_vel", 10, &MyRobot::rightWheelVelCallback, this);
  ros::Subscriber left_wheel_vel_sub_ = nh.subscribe("lwheel_vel", 10, &MyRobot::leftWheelVelCallback, this);

};



int main(int argc, char **argv)
{
  //Setup the robomuse_hwinterface_node
  ros::init(argc, argv, "robomuse_hw");

  //Setup the control manager update loop
  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);  //Controller manager would update the robot HW interface objects(read as well as write)

 //Spinner thread for multithreading
  ros::AsyncSpinner spinner(10);
  spinner.start();


  //Control loop
  ros::Time prev_time=ros::Time::now();
  ros::Rate rate(10.0); //Updation rate 10Hz

  while(ros::ok())
  {
    const ros::Time   time = ros::Time::now();
    const ros::Duration period = time-prev_time;

    robot.read(period);
    cm.update(time, period);
    robot.write();
    robot.reset_encoder_ticks();
    robot.reset_wheel_velocity();
    rate.sleep();

  }
  return 0;

}
