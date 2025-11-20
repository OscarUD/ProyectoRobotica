#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np

class DetectorNode:
    def __init__(self):
        rospy.init_node("detector", anonymous=True)
        self.sub_image = rospy.Subscriber("/camera_robot/image_raw", Image, self.callback_image)
        self.pub_pos_cubos = rospy.Publisher("/posicioncubos", Float32MultiArray, queue_size=1)
        self.pub_pos_fichas = rospy.Publisher("/posicionfichas", Float32MultiArray, queue_size=1)
        self.bridge = CvBridge()

    def callback_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Simulación: generar posiciones aleatorias
        cubos = np.random.rand(5,2).flatten()  # 5 cubos x,y
        fichas = np.random.rand(3,2).flatten()  # 3 fichas x,y

        self.pub_pos_cubos.publish(Float32MultiArray(data=cubos.tolist()))
        self.pub_pos_fichas.publish(Float32MultiArray(data=fichas.tolist()))

if __name__ == "__main__":
    node = DetectorNode()
    rospy.spin()
