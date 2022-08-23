#!/bin/bash
roslaunch urg_node urg_lidar.launch
sleep 1s
roslaunch rplidar_ros rplidar.launch
sleep 1s
roslaunch ira_laser_tools laserscan_multi_merger.launch
sleep 1s
roslaunch robomuse_hardware hw_control.launch
sleep 1s
roslaunch robomuse_navigation hw_navigation.launch
