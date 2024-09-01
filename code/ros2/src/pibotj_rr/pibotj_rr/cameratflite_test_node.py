import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
#import tensorflow as tf
# EL BUENO ES ESTE
import tflite_runtime.interpreter as tflite

#import tflite_runtime.interpreter as interpreter
from std_msgs.msg import String
import signal
import sys

class CameraTFLITETestNode(Node):
    def __init__(self):
        super().__init__('cameratflite_test_node')
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        if not self.camera.isOpened():
            self.get_logger().error('Failed to open camera device %d' % self.cameraDeviceNumber)
            rclpy.shutdown()
            return

        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_imagetf_test'
        self.queueSize = 20
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        #self.periodCommunication = 0.1 # Reduce to 10 Hz for stability
        #self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        #self.i = 0

         # Publicador que publica string 
        self.topicNameDetection = 'topic_tf2detected' 
        self.detection_publisher = self.create_publisher(String, self.topicNameDetection, self.queueSize)

        
        self.periodCommunication = 0.1  # Reduce to 10 Hz for stability
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

        # Cargar el modelo TFLite
        self.model_path = '/home/juloau/robot_ws/src/pibotj_rr/custom_model_lite/best_full_integer_quant_edgetpu.tflite'
        #self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
        #self.interpreter = tflite.lite.Interpreter(model_path=self.model_path)
        # working one
        #self.interpreter = tflite.Interpreter(model_path=self.model_path)

        self.interpreter = tflite.Interpreter(model_path=self.model_path, experimental_delegates=[tflite.load_delegate('/usr/lib/aarch64-linux-gnu/libedgetpu.so.1')])

        #self.interpreter = tflite.Interpreter(model_path=self.model_path)


        self.interpreter.allocate_tensors()

        # Obtener detalles de entrada y salida del modelo
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Signal handler for cleanup
        signal.signal(signal.SIGINT, self.signal_handler)

    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        if not success:
            self.get_logger().error('Failed to read frame from camera')
            return

        # Get the dimensions of the frame
        height, width, channels = frame.shape

        # Redimensionar el marco a las dimensiones requeridas por el modelo
        input_shape = self.input_details[0]['shape']
        height, width = input_shape[1], input_shape[2]
        resized_frame = cv2.resize(frame, (width, height))
        
        # Convertir la imagen a formato adecuado
        input_data = np.expand_dims(resized_frame, axis=0).astype(np.float32)
        input_data = (input_data - 127.5) / 127.5  # Normalización como en el ejemplo

        # convertir lla imagen a int8
        scale, zero_point = self.input_details[0]['quantization']
        input_data = (input_data / scale + zero_point).astype(np.int8)

        # Realizar la inferencia
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        prediction = np.squeeze(output_data)

        # Obtener el valor máximo del array de predicción
        max_value = np.max(prediction)

        # MODIFICAR ESTO PARA QUE LA LÓGICA SEA CONSISTENTE
        # ES DECIR, QUE SI VARIOS MENSAJES SON VERDAD, SE CONSIDERE UN BACHE

        if max_value > 0.90:  
            label = "Pothole detected"
            detection_message = String()
            detection_message.data = "Yes"
            
            # Crea una función que saque el contorno de la imagen y sus píxeles 
            newframe = self.get_pothole_coords(frame)
            # para canny y dilate si que necesitamos la linea de debajo
            #newframe = cv2.cvtColor(newframe, cv2.COLOR_GRAY2BGR)

        else:
            label = "No pothole"
            detection_message = String()
            detection_message.data = "No"

            newframe = frame

        # Publicar el mensaje de detección  en formato string
        self.detection_publisher.publish(detection_message)

        # Añadir la etiqueta al marco
        cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Convertir y publicar la imagen con la etiqueta
        #ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(frame, encoding="bgr8")
        ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(newframe, encoding="bgr8")
 
        self.publisher.publish(ROSImageMessage)

    def cleanup(self):
        self.camera.release()

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received, shutting down...')
        self.cleanup()
        sys.exit(0)  


    #def get_contours(sel, img, img_contour):

    #    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    #    for cnt in contours:
    #        area = cv2.contourArea(cnt)

    #        if area < 1000: 
    #            cv2.drawContours(img_contour, cnt, -1,(255,0,255), 7)
    #            peri=cv2.arcLength(cnt, True)
    #            approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
    #            print(len(approx))
    #            print(approx)

    def get_contours(self, img, img_contour):
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1500:  # Ignorar pequeños contornos para reducir ruido
                cv2.drawContours(img_contour, cnt, -1,(255,0,255), 5)

                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                
                print(len(approx))
                print(approx)

                # Dibujar los contornos y el polígono aproximado
                #cv2.drawContours(img_contour, [approx], 0, (255, 0, 255), 5)
                
                # Detectar la forma según el número de vértices
                #if len(approx) == 3:
                #    shape_name = "Triangle"
                #elif len(approx) == 4:
                #    x, y, w, h = cv2.boundingRect(approx)
                #    aspectRatio = float(w) / h
                #    if 0.95 <= aspectRatio <= 1.05:
                #        shape_name = "Square"
                #    else:
                #        shape_name = "Rectangle"
                #elif len(approx) == 5:
                #    shape_name = "Pentagon"
                #elif len(approx) == 6:
                #    shape_name = "Hexagon"
                #else:
                #    shape_name = "Circle"
                
                # Dibujar el nombre de la forma en la imagen
                #x, y, w, h = cv2.boundingRect(approx)
                #cv2.putText(img_contour, shape_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    
    def get_pothole_coords(self, image):

        img_blur = cv2.GaussianBlur(image,(7,7),1)
        img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)

        height, width = img_gray.shape

    
        # Excluye 200 píxeles de cada lado
        #min_distance_from_edge = 100  # pixels
        #mask = np.zeros_like(img_gray)
        #mask[min_distance_from_edge:height - min_distance_from_edge, min_distance_from_edge:width - min_distance_from_edge] = 255


        # Excluye 100 píxeles en la parte superior e inferior no de los lados
        min_distance_from_top_bottom = 100  # pixels
        mask = np.zeros_like(img_gray)
        mask[min_distance_from_top_bottom:height - min_distance_from_top_bottom, 0:width] = 255

    
        # Se aplica el filtro Canny a la imagen reducida
        # El valor más acercado es mínimo: 80 y máximo: 180
        img_canny = cv2.Canny(img_gray, 80, 180)
        img_canny_masked = cv2.bitwise_and(img_canny, mask)
        kernel = np.ones((5,5))
        img_dilated = cv2.dilate(img_canny_masked, kernel, iterations=1)

        img_contour = image.copy()
        self.get_contours(img_dilated,img_contour)
        
        return img_contour    

    
    def __del__(self):
        self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    publisherObject = CameraTFLITETestNode()
    #rclpy.spin(publisherObject)
    #publisherObject.destroy_node()
    #rclpy.shutdown()

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
