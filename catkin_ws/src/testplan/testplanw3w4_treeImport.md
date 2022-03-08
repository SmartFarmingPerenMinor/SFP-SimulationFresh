# Testplan period 1

## Introduction
This is a testplan written for the robotic arm simulation

## To be tested
- automate tree import in gazebo

## Testing

### Installation of steps
For the install steps follow the steps of testplanw1w2.md (link)[https://github.com/SmartFarmingPerenMinor/SFP-SimulationFresh/blob/master/testplanw1w2.md#testing]

To import the three run following commands
```bash
cd SFP-SimulationFresh/catkin_ws/src/test/worlds
./setup.sh
```

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