# RoboMuse V Hardware Package

This package is developed to control the RoboMuse V drive using ROS. 


## Prerequisites

Below is a series of ROS packages which are required and should be installed beforehand:
1. [ROS control](http://wiki.ros.org/ros_control)
2. [ROS controllers](http://wiki.ros.org/ros_controllers). The ros contollers utilised in RoboMuse V are [Differential drive controller](http://wiki.ros.org/diff_drive_controller) and [Joint State Controller](http://wiki.ros.org/joint_state_controller)
3. [Robot Pose EKF](http://wiki.ros.org/robot_pose_ekf)
4. [Rosserial](http://wiki.ros.org/rosserial)


## Package Execution

A step by step commands to start the communication with the drive micro-controller

Run roscore, if not running

```
roscore
```

To connect with the arduino board (connected to the motor driver). Use the right baud rate and usb id. You can permanently change the USB ID and baud rate in the ros node.

```
rosrun rosserial_python serial_node_kangaroo.py /dev/ttyUSB0 _baud:=38400
```

Launch file to run the hardware interface node, ros controllers and controller manager. 

```
roslaunch robomuse_hardware hw_control.launch
```

To connect arduino board (connected to imu)

```
rosrun rosserial_python serial_node_imu.py 
```

Run the "enc_calibrated_odometry" node, which utilises the benifit of robomuse_calibration package to correct the encoder odometry 
 
```
'todo'
```

Run the below "ekf_factored" launch file for fusion of IMU heading angle and Encoder odometry used by robot_pose ekf package 
```
roslaunch robomuse_hardware ekf_odom_factored.launch 
```

## Testing and Debugging

Explains how to test ros nodes, launch ros files and debug errors

### ROS Topic information and communication 

Utilise the benifit of rostopic info, echo and hz to check the incoming data and it's rate. 
For eg:

```
rostopic hz /robomuse_diff/odom
```
Will show the communication rate of Odometry Information(should be 10Hz).Simmiliarly, motors commands and encoder data should communcate at 10Hz on their specific topics. 

### Arduino Communication 

Check the right port and baud rate if the rosserial node is not connecting 

### Odometric Error

#### Checkup Tests.
	1.Restart the hw_control launch file and rotate the robot 360* and check the values provided by the robomuse_diff/odom.
	2.Restart the hw_control launch file and translate the robot to a specific position and compare the odometry values and the real world values.

#### Calibration of the Odometry 
Due to change in the behaviour of the encoder sensors, wheel radius or other factors, the robot's odometry change
	1.Please go through the Robomuse_Calibrati on and run the odometry calibration protocols when needed. Run the calibration launch file included in the robomuse_calibration package and call the calibration service. The robot would move on a desiged path and publish the correction factors to a yaml file, which is then utilised by the "enc_calibrated_odometry"done  node to correct the encoder odometry. 
	2.Translate the robot using teleoperation to a specific distance, compare the encoder_x value provided in robomuse_diff/odom rostopic to the real distance moved. Find the ratio and multiply them to the variables provided in the read function in "robomuse_hw.cpp" node.
Similarly,the same test can also be done by rotating the robot 360*.  



















