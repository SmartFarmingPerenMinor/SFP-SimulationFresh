#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

import cv2
from cv_bridge import CvBridge

class cameraViewer:
    def __init__(self):
        self.image = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/ur10e/camera1/color/image_raw', Image, self.image_callback)
        rospy.on_shutdown(cv2.destroyAllWindows)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('3d image', self.image)
        cv2.waitKey(1)
