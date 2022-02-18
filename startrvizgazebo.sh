#!/bin/bash

( cd ./catkin_ws && catkin_make )


source ./catkin_ws/devel/setup.bash

roslaunch ur_gazebo ur10.launch limited:=true &
sleep 15

roslaunch ur10_e_moveit_config ur10_e_moveit_planning_execution.launch sim:=true limited:=true &
sleep 15

roslaunch ur10_moveit_config moveit_rviz.launch config:=true &
sleep 15