#!/usr/bin/python3

import sys
import rospy

from mover import endEffectorMover
from camera import cameraViewer
from depth_camera import depthViewer
from world import worldBuilder

def main():
    endEffectorMoverObject = endEffectorMover(sys.argv)
    # worldBuilderObj = worldBuilder(endEffectorMoverObject)
    # worldBuilderObj.addPlane("ground_plane")
    # cameraViewerObject = cameraViewer()
    # depthViewerObject = depthViewer("/ur10e/camera1/depth/image_raw")
    # endEffectorMoverObject.promptLocationAndMove()
    wps = endEffectorMoverObject.set_waypoints()
    endEffectorMoverObject.cartesian_path_execution(wps)
    





if __name__ == "__main__":
    try: 
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Process interrupted!")
