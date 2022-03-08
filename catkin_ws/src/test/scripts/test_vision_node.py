#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class Follower:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/ur10e/camera1/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        new_image = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_OTSU)
        inverted_binary = ~binary
        contours, hierarchy = cv2.findContours(inverted_binary, 
        cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        with_contours = cv2.drawContours(image, contours, -1,(255,0,255),3)
        cv2.imshow('Detected contours', with_contours)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # Show the total number of contours that were detected
        print('Total number of contours detected: ' + str(len(contours)))

def main():
    rospy.init_node('test_vision_node')
    rospy.on_shutdown(cv2.destroyAllWindows())
    follower = Follower()
    rospy.spin()


if __name__ == "__main__":
    main()
