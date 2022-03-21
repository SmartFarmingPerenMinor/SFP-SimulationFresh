#!/usr/bin/python3

import rospy
import moveit_commander
#For vsc.. intellisense
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose


from math import pi, tau, dist, fabs, cos, sin

from moveit_commander.conversions import pose_to_list

class endEffectorMover:
    def __init__(self, arguments):
        # init commander / rospy node
        moveit_commander.roscpp_initialize(arguments)
        rospy.init_node("test_move", disable_signals=True)

        # instantiate the robot
        self.robot = RobotCommander()

        # instantiate the scene
        self.scene = PlanningSceneInterface()

        # instantiate moveGroupCommander
        group_name = 'manipulator'
        self.move_group = MoveGroupCommander(group_name)
        rospy.on_shutdown(self.move_group.stop)
        rospy.on_shutdown(self.move_group.clear_pose_targets)
        rospy.on_shutdown(moveStop)
        # subscribe to the topic
        self.display_trajectory_publisher = rospy.Publisher(
                "/move_group/display_planned_path",
                DisplayTrajectory,
                queue_size = 20,
        )

        # setup current state as home
        self.move_group.set_named_target('home')

    def calcQuaternions(self, phi, theta, psi):
        qw = cos(phi/2) * cos(theta/2) * cos(psi/2) + sin(phi/2) * sin(theta/2) * sin(psi/2)
        qx = sin(phi/2) * cos(theta/2) * cos(psi/2) - cos(phi/2) * sin(theta/2) * sin(psi/2)
        qy = cos(phi/2) * sin(theta/2) * cos(psi/2) + sin(phi/2) * cos(theta/2) * sin(psi/2)
        qz = cos(phi/2) * cos(theta/2) * sin(psi/2) - sin(phi/2) * sin(theta/2) * cos(psi/2)
        return qw, qx, qy, qz

    def printInfo(self):

        planning_frame = self.move_group.get_planning_frame()
        print("=== planning frame: %s" % planning_frame)

        planning_pipeline = self.move_group.get_planning_pipeline_id()
        print("=== planning pipeline: %s" % planning_pipeline)
 
        eef_link = self.move_group.get_end_effector_link()
        print("=== end effector link: %s" % eef_link)

        group_names = self.robot.get_group_names()
        print("=== available planning groups:", group_names)

        robot_state = self.robot.get_current_state()
        print("=== robot state")
        print(robot_state)

        
        print("")

    def promptLocationAndMove(self):
        moreMovement: bool = True
        try:
            button = input("[Enter] for coord, [H] for home: ")
        except ValueError:
            print("Invalid value. Enter a valid value.")
        if button == "":
            while(moreMovement):
                try:
                    x = float(input('Enter a x coordinate: '))
                    y = float(input('Enter a y coordinate: '))
                    z = float(input('Enter a z coordinate: '))
                except ValueError:
                    print("Invalid value. Enter a valid value.")
                else:
                    moreMovement = False

            self.moveTo(x,y,z)
        elif button.lower() == "h":
            self.move_group.set_named_target("home")
            self.move_group.go(wait=True)

            self.move_group.stop()
            self.move_group.clear_pose_targets()

    def moveTo(self, x, y, z):
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        move_group = self.move_group

        move_group.set_pose_target(pose_goal)

        move_group.construct_motion_plan_request()
        #plan the path
        plan = move_group.plan()
        plan_success: bool = plan[0]

        #check if the resulting path is valid and exit if not
        if plan_success == False:
            print("Planning failed!")
            self.promptMove()
            return
        
        # self.visualizePlanning(plan)
        print("Planning succeeded, moving")
        plan = move_group.go(wait=True)

        move_group.stop()
        move_group.clear_pose_targets()

        self.promptMove()
        

    def visualizePlanning(self, plan):
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def promptMove(self):
        try:
          char = input("[Enter] request new coordinate, [Z] to exit: ")
        except ValueError:
            print("Invalid value. Enter a valid value.")
        if char == "":
            self.promptLocationAndMove()
        elif char.lower() == "z":
            rospy.signal_shutdown("")

def moveStop():
    print("Program exited. Goodbye.")