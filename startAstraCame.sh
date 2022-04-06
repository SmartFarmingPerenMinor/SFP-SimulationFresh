#!/bin/env bash

ws=`pwd`"/catkin_ws"

source /opt/ros/noetic/setup.bash
source ./catkin_ws/devel/setup.bash

roscd astra_camera
./scripts/create_udev_rules

cd $ws && catkin_make --pkg astra_camera

roscore & sleep 3 &&
rviz & sleep 5 &&
roslaunch astra_camera astrapro.launch & sleep 15 &&
rostopic list
pwd
