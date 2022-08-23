#!/bin/bash

file=~/../../dev/ttyUSB0
if [ -e "$file" ]
then
    if [ ! -w "$file" ]
    then
        sudo chmod 666 "$file" && echo "Permissions have been corrected"
        fi
	
    elif [ ! -e $file ]
    then
        echo " The USB is not working properly. Find the error! "
    fi

roslaunch rplidar_ros rplidar.launch
