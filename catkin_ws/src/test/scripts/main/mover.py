#!/usr/bin/python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos, sin

from moveit_commander.conversions import pose_to_list

class endEffectorMover:
    def __init__(self, arguments):
        # init commander / rospy node
        moveit_commander.roscpp_initialize(arguments)
        rospy.init_node("test_move", anonymous=True)

        # instantiate the robot
        self.robot =  moveit_commander.RobotCommander()

        # instantiate the scene
        self.scene = moveit_commander.PlanningSceneInterface()

        # instantiate moveGroupCommander
        group_name = 'manipulator'
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # subscribe to the topic
        display_trajectory_publisher = rospy.Publisher(
                "/move_group/display_planned_path",
                moveit_msgs.msg.DisplayTrajectory,
                queue_size = 20,
        )

        # setup current state as home
        self.move_group.set_named_target('home')
        
        self.addPlane("ground", 0,0,0,0,0,1)


    def addPlane(self, planeName :  str, x : float, y: float, z: float, normX: float, normY: float, normZ: float):
        rospy.sleep(1)
        plane_pose = geometry_msgs.msg.PoseStamped()
        plane_pose.header.frame_id = self.move_group.get_planning_frame()
        plane_pose.pose.orientation.w, plane_pose.pose.position.x, plane_pose.pose.position.y, plane_pose.pose.position.z = self.calcQuaternions(x, y, z)
        self.scene.add_plane(planeName, plane_pose, normal=(normX,normY,normZ))
        print("[mover] added ground plane")

    def calcQuaternions(self, phi, theta, psi):
        qw = cos(phi/2) * cos(theta/2) * cos(psi/2) + sin(phi/2) * sin(theta/2) * sin(psi/2)
        qx = sin(phi/2) * cos(theta/2) * cos(psi/2) - cos(phi/2) * sin(theta/2) * sin(psi/2)
        qy = cos(phi/2) * sin(theta/2) * cos(psi/2) + sin(phi/2) * cos(theta/2) * sin(psi/2)
        qz = cos(phi/2) * cos(theta/2) * sin(psi/2) - sin(phi/2) * sin(theta/2) * cos(psi/2)
        return qw, qx, qy, qz

    def printInfo(self):
        planning_frame = self.move_group.get_planning_frame()
        print("=== planning frame: %s" % planning_frame)

        eef_link = self.move_group.get_end_effector_link()
        print("=== end effector link: %s" % eef_link)

        group_names = self.robot.get_group_names()
        print("=== available planning groups:", self.robot.get_group_names())

        print("=== robot state")
        print(self.robot.get_current_state())
        print("")

    def promptLocationAndMove(self):
        x = float(input('Enter a x coordinate: '))
        y = float(input('Enter a y coordinate: '))
        z = float(input('Enter a z coordinate: '))

        self.moveTo(x,y,z)

    def moveTo(self, x, y, z):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        self.move_group.set_pose_target(pose_goal)

        #plan the path
        plan = self.move_group.plan()
        plan_succes = plan[0]

        #check if the resulting path is valid and exit if not
        if plan_succes == False:
            print("planning failed, press CTRL-Z to exit or request a new coordinate")
            return

        print("planning succeded, moving")
        plan = self.move_group.go(wait=True)

        self.move_group.stop()

        print("path executed press CTRL-Z to exit or request a new coordinate")
        self.promptLocationAndMove()


