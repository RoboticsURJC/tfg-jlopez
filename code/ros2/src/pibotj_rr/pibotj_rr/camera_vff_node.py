import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import tflite_runtime.interpreter as tflite
from std_msgs.msg import String
import signal
import sys
import time
from geometry_msgs.msg import Polygon, Point32

class CameraVFFNode(Node):
    def __init__(self):
        super().__init__('camera_vff_node')
        #self.cameraDeviceNumber = 0
        #self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        #if not self.camera.isOpened():
        #    self.get_logger().error('Failed to open camera device %d' % self.cameraDeviceNumber)
        #    rclpy.shutdown()
        #    return

        #self.bridgeObject = CvBridge()
        #self.topicNameFrames = 'camera_vff'
        #self.queueSize = 10
        #self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        # Suscribirse a las coordenadas 192x192 HAY QUE CAMBIARLO
        #self.polygon_publisher = self.create_publisher(Polygon, 'pothole_coords', self.queueSize)


        # tamaño de la cola de mensajes
        self.queueSize = 10
        self.subscription = self.create_subscription(
            Polygon, 
            'pothole_coords',
            self.coords_callback,
            self.queueSize)
        
        # Usa 10 Hz
        #self.periodCommunication = 0.1  
        #self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
  
        # Maneja la señal de Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info('set signal handler')
    

    #def timer_callbackFunction(self):
    #    success, frame = self.camera.read()
    #    if not success:
    #        self.get_logger().error('Failed to read frame from camera')
    #        return

    #    resized_frame = cv2.resize(frame, (192, 192))

        # Convertir y publicar la imagen
    #    ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(newframe, encoding="bgr8")

    #    self.publisher.publish(ROSImageMessage)

    def coords_callback(self, coords):

        if not coords.points or len(coords.points) == 0:
            return

        for i, point in enumerate(coords.points):
            
            # cómo hacer para enlazar el vff, esto son los puntos
            # de cada coordenada del bache
            print(point.x, point.y)
    

    def cleanup(self):
        self.camera.release()

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received, shutting down...')
        self.cleanup()
        sys.exit(0)  

    def __del__(self):
        self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    publisherObject = CameraVFFNode()

    try:
        rclpy.spin(publisherObject)
    except KeyboardInterrupt:
        publisherObject.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        publisherObject.cleanup()
        publisherObject.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
