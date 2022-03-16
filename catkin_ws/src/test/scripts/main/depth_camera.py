#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
from cv2 import SimpleBlobDetector_Params, SimpleBlobDetector_create, SimpleBlobDetector
from cv_bridge import CvBridge

class depthViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        self.sub = rospy.Subscriber("/ur10e/camera1/depth/image_raw", Image, self.image_callback)
        rospy.on_shutdown(cv2.destroyAllWindows)

    def image_callback(self, msgdepth):
        rospy.loginfo_once("Establishing depth img Callback")
        image = self.bridge.imgmsg_to_cv2(msgdepth,"passthrough")
        self.image = image           
        cv2.imshow('Depth view', image)
        cv2.waitKey(2)
    def get_image_dimensions(self):
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
