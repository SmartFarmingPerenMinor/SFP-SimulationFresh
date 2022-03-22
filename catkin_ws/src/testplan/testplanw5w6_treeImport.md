# Testplan period 1

## Introduction
This is a testplan written for the robotic arm simulation

## To be tested
- automate tree import in gazebo

## Testing

### Installation of steps
For the install steps follow the steps of testplanw1w2.md [link](https://github.com/SmartFarmingPerenMinor/SFP-SimulationFresh/blob/master/catkin_ws/src/testplan/testplanw1w2.md)

#### Download the required python package

```bash
sudo apt update
sudo apt install -y python3-pip
pip3 install Jinja2
```

#### Importing the tree with python

To import the three run following commands
```bash
cd SFP-SimulationFresh/objectImporting
python3 importObj.py
```

Example values are:
- x = 0.7
- y = 0.5
- z = 0.0

The Z is wil be in the air if it is above 0.0.
The x and y value's can bechanged with out any problems.

### Starting the simulation

Move to the correct directory
```bash
cd ~/SFP-SimulationFresh
```

Run the start script 
```bash
./startgazebo.sh
```

## Criteria for succes
#### Download criteria
- Can you download the files?

#### setup the file criteria
- Does the setup script let the tree appear? yes or no?

#### Start criteria
- Did the ./startgazebo script automatically start the simulator?
- Can you see the arm in the world?
- Can you see the tree in the world?
  - If not was the setup script run in SFP-SimulationFresh/catkin_ws/src/test/worlds