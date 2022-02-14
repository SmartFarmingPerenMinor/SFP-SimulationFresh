#!/usr/bin/env python3.8

from socket import MSG_NOSIGNAL
import os
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur10_e_move_test', anonymous=True)
pose = Pose()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

def main():    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    display_trajectory_pub = rospy.Publisher("move_group/display_planned_path", DisplayTrajectory, queue_size=20)

    pose_one()
    move_to_pos()
    pose_two()
    move_to_pos()
    rospy.spin()
    moveit_commander.roscpp_shutdown()

def pose_one():
    pose.orientation.w = 0.0
    pose.position.x = 0.4
    pose.position.y = 0.1
    pose.position.z = 0.4

def pose_two():
    pose.orientation.w = 0.0
    pose.position.x = 0.2
    pose.position.y = 0.15
    pose.position.z = 0.2

def move_to_pos() -> bool:
    move_group.set_pose_target(pose)
    print("Moving!")
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    
    return True

if __name__ == "__main__":
    main()
