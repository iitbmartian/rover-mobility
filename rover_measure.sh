#!/bin/sh 
cd ~/ws/rover_ws;
source devel/setup.zsh;
# echo 'Please enter ROS_MASTER_URI';
# read ROS_MASTER_URI_READ;
# export ROS_MASTER_URI=$ROS_MASTER_URI_READ;
# export ROS_MASTER_URI=http://labpc:11311/
echo 'Voltage Measurement'
roslaunch rover_measure measure.launch
