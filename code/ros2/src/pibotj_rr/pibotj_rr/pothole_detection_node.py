import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32 
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf

class PotholeDetectionNode(Node):
    def __init__(self):
        super().__init__('pothole_detection_node')
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        if not self.camera.isOpened():
            self.get_logger().error('Failed to open camera device %d' % self.cameraDeviceNumber)
            rclpy.shutdown()
            return

        # Publicador que publica imagen
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_cameratf_image'
        self.queueSize = 20
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        
        # Publicador que publica string 
        self.topicNameDetection = 'topic_tfdetected' 
        self.detection_publisher = self.create_publisher(String, self.topicNameDetection, self.queueSize)
        # Alternativamente, puedes usar Int32:
        # self.detection_publisher = self.create_publisher(Int32, self.topicNameDetection, self.queueSize)
        
        self.periodCommunication = 0.1  # Reduce to 10 Hz for stability
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

        # Cargar el modelo TFLite
        self.model_path = '/home/juloau/Desktop/robot_ws/src/pibotj_rr/custom_model_lite/detect.tflite'  # Reemplaza con la ruta de tu modelo
        self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()

        # Obtener detalles de entrada y salida del modelo
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        if not success:
            self.get_logger().error('Failed to read frame from camera')
            return

        # Redimensionar el marco a las dimensiones requeridas por el modelo
        input_shape = self.input_details[0]['shape']
        height, width = input_shape[1], input_shape[2]
        resized_frame = cv2.resize(frame, (width, height))
        
        # Convertir la imagen a formato adecuado
        input_data = np.expand_dims(resized_frame, axis=0).astype(np.float32)
        input_data = (input_data - 127.5) / 127.5  # Normalización como en el ejemplo

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
            # Alternativamente, con Int32:
            # detection_message = Int32()
            # detection_message.data = 1  # 1 para indicar que se detectó un bache
        else:
            label = "No pothole"
            detection_message = String()
            detection_message.data = "No"
            # Alternativamente, con Int32:
            # detection_message = Int32()
            # detection_message.data = 0  # 0 para indicar que no se detectó un bache

        # Publicar el mensaje de detección  en formato string
        self.detection_publisher.publish(detection_message)

        # Añadir la etiqueta al marco
        cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Convertir y publicar la imagen con la etiqueta
        ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(ROSImageMessage)

    def __del__(self):
        self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    publisherObject = PotholeDetectionNode()
    rclpy.spin(publisherObject)
    publisherObject.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
