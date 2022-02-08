#!/bin/bash

sudo sh -c 'echo "deb http://packages.rog.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl -y

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update -y

sudo apt install ros-noetic-desktop-full -y

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc &
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin python3-catkin-tools -y

