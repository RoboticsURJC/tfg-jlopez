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


class CameraDLNode(Node):
    def __init__(self):
        super().__init__('camera_dl_node')
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        if not self.camera.isOpened():
            self.get_logger().error('Failed to open camera device %d' % self.cameraDeviceNumber)
            rclpy.shutdown()
            return

        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'camera_dl'
        self.queueSize = 10
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        # Usa 10 Hz
        self.periodCommunication = 0.1  
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
  
        # Maneja la señal de Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info('set signal handler')
        

    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        if not success:
            self.get_logger().error('Failed to read frame from camera')
            return

        resized_frame = cv2.resize(frame, (192, 192))
        # Filtro que detecta bien el entorno conocido
        gray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
        th1 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,23,-50)

        # ayuda a quitar los últimos picos blancos de la imagen
        kernel = np.ones((3, 3), np.uint8) 
        opened_th1 = cv2.morphologyEx(th1, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(opened_th1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filtrar los contornos pequeños o irrelevantes
        line_positions = []
        min_contour_length = 50 
        for contour in contours:
            if cv2.arcLength(contour, True) > min_contour_length:
                # Calcular los momentos para encontrar el centro del contorno
                M = cv2.moments(contour)
                # Evitar división por cero
                if M["m00"] != 0:  
                    cX = int(M["m10"] / M["m00"])  
                    cY = int(M["m01"] / M["m00"])  
            
                    line_positions.append((cX, cY))  
            
        # Contar las líneas detectadas
        num_lines_detected = len(line_positions)
        print(num_lines_detected)

        # si la línea es 1 hay que mirar por donde se encuentra 
        # la línea para cambiar la trayectoria 
        if(num_lines_detected == 1):

            # Analizar las posiciones de las líneas
            width = resized_frame.shape[1]  # Ancho de la imagen
            #for position in line_positions:
            cX, cY = line_positions[0]
            if cX < width // 3:
                print("Línea detectada a la izquierda")
            elif cX > (2 * width) // 3:
                print("Línea detectada a la derecha")
            else:
                print("Línea detectada en el centro")


        bgr_image = cv2.cvtColor(opened_th1, cv2.COLOR_GRAY2BGR)

        ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(bgr_image, encoding="bgr8")

        self.publisher.publish(ROSImageMessage)

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
    publisherObject = CameraDLNode()

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
