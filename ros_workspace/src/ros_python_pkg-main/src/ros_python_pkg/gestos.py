#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import random

class GestosNode:
    def __init__(self):
        rospy.init_node("gestos", anonymous=True)
        self.sub_webcam = rospy.Subscriber("/web_cam/image_raw", Image, self.callback_image)
        self.pub_juego = rospy.Publisher("/juego", Int32, queue_size=1)
        self.pub_ganador = rospy.Publisher("/ganador", Int32, queue_size=1)
        self.bridge = CvBridge()

    def callback_image(self, msg):
        # Aquí se podría procesar la imagen para detectar gestos
        juego_detectado = random.randint(0,1)
        ganador = random.randint(0,1)
        self.pub_juego.publish(Int32(juego_detectado))
        self.pub_ganador.publish(Int32(ganador))

if __name__ == "__main__":
    node = GestosNode()
    rospy.spin()
