
from base_node import BaseNode
import rospy    # Documentation: http://docs.ros.org/en/melodic/api/rospy/html/
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class NodoWebcam(BaseNode):

    def __init__(self, name='nodo_webcam', param_namespace='~', package_path='', clear_params_on_exit=True):
        # Inicializar BaseNode
        super().__init__(name, param_namespace, package_path, clear_params_on_exit)

        # Inicializar la cámara
        self.video = cv2.VideoCapture(0)
        if not self.video.isOpened():
            self.logwarn("No se pudo abrir la cámara", force=True)
        
        # Inicializar CvBridge
        self.bridge = CvBridge()

        # Arrancar el loop principal
        self.start_loop()

    # Inicializar publisher
    def init_publishers(self):
        self.pub = rospy.Publisher('webcam_image', Image, queue_size=10)

    # No necesitamos subscribers por ahora
    def init_subscribers(self):
        pass

    # Loop principal: capturar frame y publicar
    def loop(self):
        ret, frame = self.video.read()
        if not ret:
            self.logwarn("No se pudo capturar la imagen")
            return

        # Convertir frame a mensaje ROS y publicar
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)
        self.loginfo("Frame publicado")

    # Se llama al recargar parámetros
    def on_reload(self):
        pass

    # Se llama al cerrar ROS
    def on_close(self):
        self.video.release()
        self.loginfo("Cámara liberada y nodo cerrado")

# Bloque principal
if __name__ == "__main__":
    try:
        nodo = NodoWebcam()
    except rospy.ROSInterruptException:
        pass
