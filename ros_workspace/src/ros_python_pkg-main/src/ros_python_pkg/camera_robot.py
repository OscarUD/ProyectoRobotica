#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraRobotNode:
    def __init__(self):
        rospy.init_node("camera_robot", anonymous=True)
        self.pub_image = rospy.Publisher("/camera_robot/image_raw", Image, queue_size=1)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)  # 10 Hz

    def loop(self):
        while not rospy.is_shutdown():
            # Imagen simulada: fondo negro con un cuadrado blanco
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.rectangle(img, (100,100), (200,200), (255,255,255), -1)
            msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.pub_image.publish(msg)
            self.rate.sleep()

if __name__ == "__main__":
    node = CameraRobotNode()
    node.loop()
