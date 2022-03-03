#!/bin/bash

( cd ./catkin_ws && catkin_make )

source /opt/ros/noetic/setup.bash
source ./catkin_ws/devel/setup.bash

roslaunch ur_e_gazebo ur10e.launch limited:=true &
sleep 8

roslaunch ur10_e_moveit_config ur10_e_moveit_planning_execution.launch sim:=true limited:=true &
sleep 8
