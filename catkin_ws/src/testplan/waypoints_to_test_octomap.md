# Testplan period 1

## Introduction
This is a testplan written for the robotic arm simulation

## To be tested
- Octomap collision detection

## Testing

### Installation of steps
For the install steps follow the steps of testplanw1w2.md [link](https://github.com/SmartFarmingPerenMinor/SFP-SimulationFresh/blob/master/catkin_ws/src/testplan/testplanw1w2.md)


#### Moving the arm

To run the program from move to dir SFP-SimulationFresh:
Open terminal:
```bash
source catkin_ws/devel/setup.bash
rosrun test main.py
```

The program will ask you to set waypoints.
These waypoints are set with respect to the end-effector.
Copy and paste waypoints into terminal up to before the next checkpoint for demo movement.
Then press enter until movement/planning takes place.

Demo values are:
Waypoint 1
```bash
0, 0, 0.3
x
0, 0.3, 0
x
-0.05, 0.05, 0

```

Waypoint 2: Should not be reachable (Planning visible in RViz
```bash
-0.1, 0.3, 0

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

If errors, close rviz & gazebo and run
```bash
./clean_startRvizGazebo.sh
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
- Can you see the octomap scan in Rviz?
  - If not was the setup script run in SFP-SimulationFresh/catkin_ws/src/test/worlds?