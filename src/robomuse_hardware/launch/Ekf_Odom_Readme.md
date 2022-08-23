# Robot_Pose_Ekf Package Application

This ReadMe contains the procedure to implement [Robot_Pose_Ekf](wiki.ros.org/robot_pose_ekf) package for obtaining fused Odometry for Navigation of RoboMuse 5.0 

## About the Package 

This package is used to estimate the pose of a robot, based on (partial) pose measurements coming from different sources, Encoder Odometry and IMU sensor in the case of RoboMuse 5.0.

## Running the launch file for obtaining fused Odometry. 

### Prerequisite Packages

Below is a series of ROS packages which are required and should be installed beforehand:
1. [Robot Pose EKF](http://wiki.ros.org/robot_pose_ekf)
2. [Rosserial](http://wiki.ros.org/rosserial)

### Nodes and Launch Files

Run roscore, if not running

```
roscore
```

Connect with the arduino board connected to the **Kangaroo x2** motion controller. 

```
rosrun rosserial_python serial_node_kangaroo.py /dev/ttyUSB0 _baud:=38400
```

Launch the file to run the hardware interface node, ros controllers and controller manager. 

```
roslaunch robomuse_hardware hw_control.launch
```

Connect with the arduino board connected to **IMU** sensor

```
rosrun rosserial_python serial_node_imu.py 
```

Run the **ekf_odom** launch file 
```
roslaunch robomuse_hardware ekf_odom.launch 
```

Run the **nav_odometry** node for publishing Odometry messages 

```
rosrun robomuse_hardware nav_odometry
```

### Troubleshooting

**NOTE:-** 
1.The Robot_Pose_Ekf package works only when the transforms of all the joints are available, ensure that the **robot_state_publisher** launch file is running before launching **ekf_odom** file. 

2.For obtaining correct data, ensure that the axes of IMU sensor and the IMU joint in URDF are set in **NED** ( North-East-Down).

Solutions to possible errors for **ekf_odom** launch file can be found on this [link](http://wiki.ros.org/robot_pose_ekf/Troubleshooting)

## Using Fused Odometry in Navigation Stack

Most of the above nodes and launch files are a part of the **hw_navigation** launch file, run the remaining files after launching

```
roslaunch robomuse_navigation hw_navigation
```

**IMP**

Ensure that the transform **odom**-->**base_link** from the encoder odometry is unticked using the rqt_reconfigure everytime when **hw_navigation** file is started.
Using:
```
rosrun rqt_reconfigure rqt_reconfigure
```

Ensure that the rate of the above tf is 10Hz, check by:
```
rosrun rqt_tf_tree rqt_tf_tree
```
