#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tflite_msgs.msg import TFLite, TFInference
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np
import cv2

class TFLiteNode(Node):

    def __init__(self):
        super().__init__('tflite_node')

        # Parámetros
        self.declare_parameter("model_path", "/home/juloau/Desktop/robot_ws/src/pibotj_rr/custom_model_lite/detect.tflite")

        model_path = self.get_parameter("model_path").value

        # Cargar el modelo TFLite
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        # Obtener información de los tensores
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Crear el publicador y suscriptor
        self.image_sub = self.create_subscription(
            Image,
            '/topic_camera_image',  # Tópico de imagen de la cámara
            self.image_callback,
            1
        )
        
        self.tflite_pub = self.create_publisher(TFLite, '/TFLiteReal', 1)

        self.bridge = CvBridge()

    def image_callback(self, msg_image):
        # Convertir el mensaje de imagen a un formato que pueda ser procesado por OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg_image, "bgr8")
        input_data = self.preprocess_image(cv_image)

        # Ejecutar la inferencia
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])

        # Procesar los resultados de la inferencia
        tflite_msg = self.create_tflite_message(output_data, msg_image)
        
        # Publicar los resultados
        self.tflite_pub.publish(tflite_msg)

    def preprocess_image(self, image):
        # Preprocesar la imagen para el modelo (ajustar según tu modelo)
        input_shape = self.input_details[0]['shape']
        image = cv2.resize(image, (input_shape[2], input_shape[1]))
        image = image.astype(np.float32)
        image = (image / 255.0)  # Normalización
        image = np.expand_dims(image, axis=0)
        return image

    def create_tflite_message(self, output_data, msg_image):
        # Crear un mensaje TFLite con los resultados de la inferencia
        tflite_msg = TFLite()
        # Ajustar según el formato de salida de tu modelo
        for output in output_data[0]:
            inference = TFInference()
            inference.score = float(output[0])
            inference.label = 'pothole'  # Ajustar según las etiquetas de tu modelo
            # Ejemplo de bbox, ajustar según el formato de salida de tu modelo
            inference.bbox = [0.0, 0.0, 1.0, 1.0]
            tflite_msg.inference.append(inference)
        
        tflite_msg.header.stamp = msg_image.header.stamp
        tflite_msg.header.frame_id = msg_image.header.frame_id
        return tflite_msg

def main(args=None):
    rclpy.init(args=args)
    tflite_node = TFLiteNode()
    rclpy.spin(tflite_node)
    tflite_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
