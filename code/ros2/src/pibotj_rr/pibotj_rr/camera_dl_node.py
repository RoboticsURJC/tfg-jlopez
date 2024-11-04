import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import signal
import sys
import time
from geometry_msgs.msg import Twist

class CameraDLNode(Node):
    def __init__(self):
        super().__init__('camera_dl_node')
        #self.cameraDeviceNumber = 0
        #self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        #if not self.camera.isOpened():
        #    self.get_logger().error('Failed to open camera device %d' % self.cameraDeviceNumber)
        #    rclpy.shutdown()
        #    return

        # suscriba a la imagen de la cámara

        self.queueSize = 10

        self.subscription = self.create_subscription(
            Image,
            '/camera_tf4',
            self.camera_callback,
            self.queueSize) 
        
        #self.subscription = self.create_subscription(
        #    Image,
        #    '/camera_tf3',
        #    self.camera_callback,
        #    self.queueSize) 



        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'camera_dl'
        #self.queueSize = 10
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        self.vel_publisher = self.create_publisher(Twist, 'dl_vel', self.queueSize)

        # Cada 0.1 segundos se publica = 10Hz
        #self.periodCommunication = 0.1  
        #self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
  
        # Maneja la señal de Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info('set signal handler')

        self.detected = False
        self.detect_time = 0
        

    def camera_callback(self, msg):

        resized_frame = self.bridgeObject.imgmsg_to_cv2(msg, desired_encoding='bgr8')
       
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

        twist = Twist()
        twist.linear.x = 0.0  
        twist.angular.z = 0.0  

        current_time = time.time()

        if(num_lines_detected == 2):

            self.reset_detection()

            print("Seguir recto")
            twist.linear.x = 0.5  
            twist.angular.z = 0.0  

        # si la línea es 1 hay que mirar por donde se encuentra 
        # la línea para cambiar la trayectoria 
        if(num_lines_detected == 1):

            self.reset_detection()
            # Analizar las posiciones de las líneas
            # Ancho de la imagen
            width = resized_frame.shape[1] 
            #for position in line_positions:
            cX, cY = line_positions[0]
            if cX < width // 3:
                print("Línea detectada a la izquierda")
                # gira hacia la derecha
                # angular es NEGATIVA
                twist.linear.x = 0.5  
                twist.angular.z = -0.25 

            elif cX > (2 * width) // 3:
                print("Línea detectada a la derecha")
                # gira hacia la izquierda
                # angular es POSITIVA
                twist.linear.x = 0.5  
                twist.angular.z = 0.25 
            else:
                print("Línea detectada en el centro")
                # seguir recto
                # lineal: 0.5
                # angular 0.0
                twist.linear.x = 0.5  
                twist.angular.z = 0.0

        if(num_lines_detected == 0): 
            print("Seguir recto durante 3s y luego girar")
            
            # durante 3 segundos y empezar a girar 
            if not self.detected:
                self.detected = True
                self.detect_time = current_time
            else: 
                # ya ha sido detectado 
                twist.linear.x = 0.5  
                twist.angular.z = 0.0
            
            if (current_time - self.detect_time) > 3:
                # mandar girar 
                twist.linear.x = 0.5  
                twist.angular.z = -0.125
                   
        # Publicar velocidad angular y lineal
        self.vel_publisher.publish(twist)

        bgr_image = cv2.cvtColor(opened_th1, cv2.COLOR_GRAY2BGR)

        ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(bgr_image, encoding="bgr8")

        self.publisher.publish(ROSImageMessage)

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received, shutting down...')
        sys.exit(0)  

    def reset_detection(self):
        self.detected = False
        self.detect_time = 0


def main(args=None):
    rclpy.init(args=args)
    publisherObject = CameraDLNode()

    try:
        rclpy.spin(publisherObject)
    except KeyboardInterrupt:
        publisherObject.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        #publisherObject.cleanup()
        publisherObject.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
