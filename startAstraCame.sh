#!/bin/env bash

ws=`pwd`"/catkin_ws"

source /opt/ros/noetic/setup.bash
source ./catkin_ws/devel/setup.bash

if [ !-d "catkin_ws/src/astra_camera" ]
then
	sudo apt install ros-noetic-rgbd-launch libuvc-dev ros-noetic--libuvc-camera ros-noetic-libuvc-ros -y
	cd /catkin_ws/src && git clone https://github.com/orbbec/ros_astra_camera
	mv ros_astra_camera astra_camera
	chmod +x ./astra_camera/scripts/create_udev_rules
fi


roscd astra_camera
./scripts/create_udev_rules

cd $ws && catkin_make --pkg astra_camera

roscore & sleep 3 &&
rviz & sleep 5 &&
roslaunch astra_camera astrapro.launch
