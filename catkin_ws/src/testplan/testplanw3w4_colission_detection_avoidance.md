# Testplan period 1

## Introduction
This is a testplan written for the collision avoidance and detection

## To be tested
- The robot arm can't go under the world
- The robot arm can't go inside of the add blocks
  - The  robot arm will try to move to a position near on of the block without hitting it 

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
./startrvizgazebo.sh
```

### Render collision boxes

Move to the correct directory
```bash
cd ~/SFP-SimulationFresh/catkin_ws/src/testplan/scripts
```

Run the spawn_collision_blocks program
```bash
python3 spawn_collision_blocks.py
```

### Collision testing

#### Moving the robotic arm

drag the cyaan orp at the end effector (end of the robotic arm) around

![moving the end effector](https://github.com/SmartFarmingPerenMinor/SFP-SimulationFresh/blob/master/catkin_ws/src/testplan/pictures/end_effector.png)


#### Moving in the arm Rviz

After moving  the arm press the **Plan & Execute** button

![moving the arm](https://github.com/SmartFarmingPerenMinor/SFP-SimulationFresh/blob/master/catkin_ws/src/testplan/pictures/plan_execute.png)

If no part of the arm is **red** the arm will move to set point
Dont force the end effector in a collision box this will make it impossible to move the arm

##### Arm stuk in collision box

The following steps wil fix it:
- close Rviz
- close Gazebo **don't save the state**
- ctr + c in the window where ./startrvizgazebo.sh
- ./stop_ros.sh
- ./startrvizgazebo.sh

## Criteria for succes
#### Download criteria
- Can you download the files?

#### setup the file criteria
- Do the collision boxes render after running spawn_collision_blocks? yes or no?

#### Start criteria
- Did the ./startgazebo script automatically start the simulator?
- Can you see the arm in the world?
- Can you see the 2 collision boxes in the world?
  - If not was the spawn_collision_blocks program started in SFP-SimulationFresh/catkin_ws/src/testplan/scripts
