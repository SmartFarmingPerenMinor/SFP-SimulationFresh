# Testplan period 1

## Introduction
This is a testplan written for the robotic arm simulation

## To be tested
- Robotic arm simulation software
- Forward kinematics

## Not going to be tested
- The real robotic arm

## Testing


### Installation of operating system
Before starting the test you will have to install a version of ubuntu 20.**.** LTS [link](https://releases.ubuntu.com/20.04/). You can try it on a VM but preferably it will be a installation of the IOS on your PC as the VM can make troubles with your simulation software. Using a USB stick you can flash the drive using [balenaEtcher](https://www.balena.io/etcher/). Next off you will have to change the bootorder of your device, this is device specific so you would have to lookup your own device manual to find out but most oftenly you can open the bootmenu with F1,F2 or F3. Once this is changed you will boot from the USB and the installation will start. A in depth installation guide regarding the installation can be found [here](https://itsfoss.com/install-ubuntu/)

#### config for VM version
System -> Motherboard:
- 4GB ram or more

system -> Processor:
- minum of 3 cpu core's

system -> Acceleration:
- Paravirtualisation-interface: Standard
- Hardwarevirtualisation: on

Screen -> Screen
- Vram 128MB

Storage:
- ubuntu 20.xx.xx LTS [link](https://ubuntu.com/download/desktop/thank-you?version=20.04.3&architecture=amd64)
- drive size 50GB minimum

### Downloading the files
For our testplan we will be executing everything from the home directory, this is purely out of convenience.
```bash
cd
```

We will clone the repository using git if you do not have git installed please execute `` sudo apt install git`` after doing that you will be able to clone the repository using the command down below.
```bash
git clone https://github.com/SmartFarmingPerenMinor/SFP-SimulationFresh.git
```

This command will download all the files on your PC.
 

### Installation of the software
Going forward into testing we start of by installation, this can be done automatically or if this doesn't work (crashes during installation) you can execute it manually.

#### Automatic installation
Change the directory to the location you installed (for us this is the home directory)
```bash
cd ~/SFP-SimulationFresh 
```
After changing directory we can start the installation using the installer

If the installer is not a executable use ` chmod +x install.sh `


install the files using this script we made!
```bash
./install.sh
```
#### Manual installation
Move to the correct directory
```bash
cd ~/SFP-SimulationFresh
```

We start of by adding the ros repository to the computer
```bash
sudo sh -c 'echo "deb http://packages.rog.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

We install curl to download a key that we need to access the repository
```bash
sudo apt install curl -y
```

Add the key to your computer to access the repo
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Download build tools and pull in the repository
```bash
sudo apt update -y & sudo apt upgrade -y & sudo apt install build-essentials
```

Download ros noetic full
```bash
sudo apt install ros-noetic-desktop-full -y
```

Vm's can be a annoying
```bash
sudo apt install ros-noetic-moveit ros-noetic-moveit-plugins ros-noetic-moveit-planners -y
```

Add ros to your bashrc to start it on boot and add the source right now so we do not need to restart the pc
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc &
source ~/.bashrc
```

Download some files we need to compile
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin python3-catkin-tools -y
```

update ros
```bash
sudo rosdep init && rosdep update
```

### Starting the simulation

#### Automatically

Move to the correct directory
```bash
cd ~/SFP-SimulationFresh
```

Run the start script 
```bash
./startgazebo.sh
```

#### Manually

First we build it

```bash
cd ./catkin_ws
catkin_make
```

Then open up 2 terminals

execute this in both terminals
```bash
cd ~/SFP-SimulationFresh
source ./catkin_ws/devel/setup.bash
```

then execute this in the first
```bash
roslaunch ur_gazebo ur10.launch limited:=true
```

And this in the second
```bash
roslaunch ur10_e_moveit_config ur10_e_moveit_planning_execution.launch sim:=true limited:=true
```


### Starting the kinematics

## Criteria for succes
#### Download criteria
- Can you download the files?

#### install the files criteria
- Does the automatic installer work? yes or no?

#### Start criteria
- Did the ./start script automatically start the simulator?
- Can you see the arm in the world?

#### Code criteria
- Can you enter a world coordinate?
- Does the arm move after inputting the world coordinate?

