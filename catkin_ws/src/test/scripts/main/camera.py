#!/usr/bin/env python3

from cgitb import enable
from tkinter import E
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
from cv2 import SimpleBlobDetector_Params, SimpleBlobDetector_create, SimpleBlobDetector
from cv_bridge import CvBridge

class cameraViewer:
    def __init__(self, enable_contour: bool = True):
        self.bridge = CvBridge()
        self.image = None
        self.enable_contour = True
        self.image_sub = rospy.Subscriber('/ur10e/camera1/color/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('image_timer', Image, queue_size=10)
        rospy.on_shutdown(cv2.destroyAllWindows)

    def image_callback(self, msg):
        rospy.loginfo_once("Establishing img Callback")
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # new_image = self.image.copy()
        self.image = image

        if self.enable_contour:
            cv2.imshow('Detected contours', self.contour()[1])
        else:
            cv2.imshow('Camera Output', image)
        
        cv2.waitKey(2)

        # Show the total number of contours that were detected
        # print('Total number of contours detected: ' + str(len(contours)))

    def contour(self):

        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        binary = cv2.threshold(gray, 100, 255, cv2.THRESH_OTSU)[1]
        inverted_binary = ~binary
        kernel = np.ones((10,10),np.uint8)
        shrinked = cv2.dilate(inverted_binary,kernel,iterations = 1)
        edged=cv2.Canny(shrinked,30,200)
        contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        all_contours = np.concatenate(contours)
        return self.contour_hulls(all_contours)

    def contour_hulls(self, all_contours):
        
        hulls = []

        epsilon = 0.0004*cv2.arcLength(all_contours,True)
        hull = cv2.approxPolyDP(all_contours,epsilon,True)
        hulls.append(hull)
        
        # Draw contours + hull results
        hullsss = np.concatenate(np.array(hulls))
        image_cropped = cv2.UMat(self.image)
        cv2.drawContours(image_cropped,hullsss,-1,(0,255,0),5)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image))
                
        params = SimpleBlobDetector_Params()
        detector = SimpleBlobDetector_create(params)

        keypoints = detector.detect(image_cropped)

        image_final = cv2.drawKeypoints(image_cropped, keypoints, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return keypoints, image_final
    
def get_blob_relative_position(image, keyPoint):
    rows = float(image.shape[0])
    cols = float(image.shape[1])
    # print(rows, cols)
    center_x    = 0.5*cols
    center_y    = 0.5*rows
    # print(center_x)
    x = (keyPoint.pt[0] - center_x)/(center_x)
    y = (keyPoint.pt[1] - center_y)/(center_y)
    return(x,y)