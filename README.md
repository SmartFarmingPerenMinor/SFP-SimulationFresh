# **[DEPRICATED]**

# About The Project 

This project is a research project for Lector Robotics of Inholland Alkmaar where the goal was set to prone pear tree's. This repo focusses on simulating the tree and moving the robotic arm in Gazebo & Rviz

# Getting Started

## Setup steps

### Required OS
Install a version of ubuntu 20.04 [download link](https://ubuntu.com/download/desktop/thank-you?version=20.04.4&architecture=amd64).

### Start steps

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install git  -y
```
### download repo

```bash
git clone git@github.com:SmartFarmingPerenMinor/SFP-SimulationFresh.git
```

## Install

```bash
cd SFP-SimulationFresh
./install
```

# Starting Simulation

## Start Gazebo

Make sure that current directory is **SFP-SimulationFresh**.

```bash
./startgazebo.sh
```

## Start Rviz

```bash
startrvizgazebo.sh
```

### Stoping the simulation

Close the started simulation software and run the follow command in the directory is **SFP-SimulationFresh**.

```bash
./stop_ros.sh
```
