#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class WebCamNode:
    def __init__(self):
        rospy.init_node("web_cam", anonymous=True)
        self.pub_image = rospy.Publisher("/web_cam/image_raw", Image, queue_size=1)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)

    def loop(self):
        while not rospy.is_shutdown():
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.circle(img, (320,240), 50, (0,255,0), -1)
            msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.pub_image.publish(msg)
            self.rate.sleep()

if __name__ == "__main__":
    node = WebCamNode()
    node.loop()
