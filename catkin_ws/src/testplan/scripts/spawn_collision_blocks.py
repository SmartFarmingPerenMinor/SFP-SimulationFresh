#!/usr/bin/env python3.8

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

from math import pi, tau, dist, fabs, cos, sin

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def main():
    # init commander / rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("test_move", anonymous=True)

    # instantiate the robot
    robot =  moveit_commander.RobotCommander()

    # instantiate the scene
    scene = moveit_commander.PlanningSceneInterface()

    # instantiate moveGroupCommander
    group_name = 'manipulator'
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size = 20,
    )

    planning_frame = move_group.get_planning_frame()
    print("=== planning frame: %s" % planning_frame)

    eef_link = move_group.get_end_effector_link()
    print("=== end effector link: %s" % eef_link)

    group_names = robot.get_group_names()
    print("=== available planning groups:", robot.get_group_names())

    print("=== robot state")
    print(robot.get_current_state())
    print("")

    addPlane(scene, planning_frame, "ground", 0,0,0,0,0,1)

    boxName1 = "box1"
    addBoxToScene(scene, planning_frame, boxName1, 0.2, 0.5, 0.5 , 0, 1.25, 0.25)
    boxName2 = "box2"
    addBoxToScene(scene, planning_frame, boxName2, 0.2, 0.5, 0.5 , 0, -1.25, 0.25)

    print("\n\n\nThe program will stop after 2 minutes")

    delay = 60*2.5
    time.sleep(delay)

    move_group.stop()


def addBoxToScene(scene, frameId : str, boxName: str, x_size: float, y_size: float, z_size: float, x:float, y: float, z: float):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = frameId
    box_pose.pose.orientation.w, box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z = calcQuaternions(x, y, z)
    scene.add_box(boxName, box_pose, size=(x_size,y_size,z_size))

def addPlane(scene, frameId : str, planeName :  str, x : float, y: float, z: float, normX: float, normY: float, normZ: float):
    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = frameId
    plane_pose.pose.orientation.w, plane_pose.pose.position.x, plane_pose.pose.position.y, plane_pose.pose.position.z = calcQuaternions(x, y, z)
    scene.add_plane(planeName, plane_pose, normal=(normX,normY,normZ))

def calcQuaternions(phi, theta, psi):
    qw = cos(phi/2) * cos(theta/2) * cos(psi/2) + sin(phi/2) * sin(theta/2) * sin(psi/2)
    qx = sin(phi/2) * cos(theta/2) * cos(psi/2) - cos(phi/2) * sin(theta/2) * sin(psi/2)
    qy = cos(phi/2) * sin(theta/2) * cos(psi/2) + sin(phi/2) * cos(theta/2) * sin(psi/2)
    qz = cos(phi/2) * cos(theta/2) * sin(psi/2) - sin(phi/2) * sin(theta/2) * cos(psi/2)
    return qw, qx, qy, qz

#def handler(SIGINT, handler):
#    print("stopping program")

if __name__ == "__main__":
    main()

