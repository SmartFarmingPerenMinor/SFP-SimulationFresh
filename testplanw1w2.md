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

### Downloading the files
For our testplan we will be executing everything from the home directory
```
cd
```

Then we will clone the repository like so
```
git clone https://github.com/SmartFarmingPerenMinor/SFP-SimulationFresh.git
```
 

### Installation of the software
Going forward into testing we start of by installation, this can be done automatically or if this doesn't work (crashes during installation) you can execute it manually.

#### Automatic installation
Change the directory to the location you installed (for us this is the home directory)
``` 
cd ~/SFP-SimulationFresh 
```
After changing directory we can start the installation using the installer

\ If the installer is not a executable use ` chmod +x install.sh `


\\ install the files!
```
./install.sh
```
#### Manual installation
Move to the correct directory
```
cd ~/SFP-SimulationFresh
```

Afterwards we will continue with all the steps written down in the install.sh
```
sudo sh -c 'echo "deb http://packages.rog.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```
sudo apt install curl -y
```
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```
sudo apt update -y & sudo apt upgrade -y & sudo apt install build-essentials
```
```
sudo apt install ros-noetic-desktop-full -y
```
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc &
source ~/.bashrc
```
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin python3-catkin-tools -y
```
```
rosdep update & rosdep init
```

### Starting the simulation

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

