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

class CameraTFv3Node(Node):
    def __init__(self):
        super().__init__('camera_tf3_node')
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        if not self.camera.isOpened():
            self.get_logger().error('Failed to open camera device %d' % self.cameraDeviceNumber)
            rclpy.shutdown()
            return

        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'camera_tf3'
        self.queueSize = 20
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        # Publicador que publica string 
        self.topicNameDetection = 'pothole_detected' 
        self.detection_publisher = self.create_publisher(String, self.topicNameDetection, self.queueSize)

        
        self.periodCommunication = 0.1  # Reduce to 10 Hz for stability
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        #self.i = 0

        # Inicializa el Filtro de Kalman
        #self.x_k = np.array([[0]])  # Estado inicial (magnitud del bache, por ejemplo)
        #self.P_k = np.array([[1]])  # Covarianza inicial (incertidumbre sobre el estado)
        #self.A = np.array([[1]])    # Matriz de transición de estado
        #self.B = np.array([[0]])    # Matriz de control (sin control externo)
        #self.H = np.array([[1]])    # Matriz de observación
        #self.Q = np.array([[0.001]])  # Covarianza del proceso (ajústalo según tu sistema)
        #self.R = np.array([[0.1]])    # Covarianza de las mediciones (ajústalo según el ruido de las mediciones)
        #self.u_k = np.array([[0]])    # Sin control externo


        # Cargar el modelo TFLite
        self.model_path = '/home/juloau/robot_ws/src/pibotj_rr/custom_model_lite/bestv2_full_integer_quant_edgetpu.tflite'

        self.interpreter = tflite.Interpreter(model_path=self.model_path, experimental_delegates=[tflite.load_delegate('/usr/lib/aarch64-linux-gnu/libedgetpu.so.1')])
        print("loaded interpreter")

        self.interpreter.allocate_tensors()
        print("allocated tensors")

        # Obtener detalles de entrada y salida del modelo
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        print("set input and output details")
        #print(self.input_details)
        #print(self.output_details)


        # Maneja la señal de Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        print("set signal handler")


    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        if not success:
            self.get_logger().error('Failed to read frame from camera')
            return

        height, width, channels = frame.shape

        # Redimensionar el marco a las dimensiones requeridas por el modelo
        input_shape = self.input_details[0]['shape']
        height, width = input_shape[1], input_shape[2]
        resized_frame = cv2.resize(frame, (width, height))


        
        # Convertir la imagen a formato adecuado
        input_data = np.expand_dims(resized_frame, axis=0).astype(np.float32)
        input_data = (input_data - 127.5) / 127.5  # Normalización como en el ejemplo

        # convertir la imagen a int8
        scale, zero_point = self.input_details[0]['quantization']
        input_data = (input_data / scale + zero_point).astype(np.int8)

        # Establecer el tensor deentrada y realizar la inferencia
        # self.input_details[0]['index'] devuelve 0
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        # tiene forma [1,38,756] y se supone que esta clase no nos hace falta 
        #output_data0 = self.interpreter.get_tensor(self.output_details[0]['index'])
        #prediction0 = np.squeeze(output_data0)

        # tiene forma [1, 48, 48, 32] y esta clase sí se trata del modelo entrenado
        output_data1 = self.interpreter.get_tensor(self.output_details[1]['index'])
        # elimina la dimensión del batch
        prediction1 = np.squeeze(output_data1)
    

        # Esto se hace para asegurarse de que el tensor tiene al menos dos canales en su última dimensión. 
        # Suponiendo que el segundo canal es la máscara de baches
        if prediction1.shape[-1] > 1:
             # Extrae la máscara para la clase 'pothole' lo he sacado del .yaml: 0 es la clase que buscamos

            # Obtener la escala y el punto cero del tensor de salida
            scale, zero_point = self.output_details[1]['quantization']

            # Descuantificar la máscara de baches mirando el canal 0 es la clase que buscamos

            pothole_mask = (prediction1[:, :, 0].astype(np.float32) - zero_point) * scale
            
            #umbral = 0.9
            # clase 0 llamada como object
            #object_mask = pothole_mask > umbral  # Esto te da una máscara binaria (True/False) para la clase 0
            
            #print("object mask", object_mask)
            #object_count = np.sum(object_mask)
            #print(f"Número de píxeles detectados como clase 0: {object_count}")
            # Obtener las dimensiones del array

            #altura, ancho = object_mask.shape

            # Imprimir las dimensiones
            #print(f"Tamaño de object_mask: {altura} filas, {ancho} columnas")

            #print("Before quantification 1 ", np.max(pothole_mask2))

        else:
            self.get_logger().error('Unexpected number of channels in prediction output')
            return

        # Descuantificar la máscara para su uso
        #scale, zero_point = self.output_details[1]['quantization']
        #pothole_mask = (pothole_mask.astype(np.float32) - zero_point) * scale

        #scale2, zero_point2 = self.output_details[1]['quantization']
        #pothole_mask2 = (pothole_mask2.astype(np.float32) - zero_point2) * scale2

        
        # Determinar si se ha detectado un bache
        max_value = np.max(pothole_mask)
        print("Max value after 0 ", max_value)

        #max_value2 = np.max(pothole_mask2)
        #print("Max value after 1 ", max_value2)

        # Predicción del filtro de Kalman
        #x_k_pred, P_k_pred = self.kalman_predict()

        # Actualización con la nueva medición (max_value)
        #z_k = np.array([[max_value]])  # Medición actual del sistema
        #self.x_k, self.P_k = self.kalman_update(x_k_pred, P_k_pred, z_k)

        #print("Kalman", self.x_k[0][0])

        # MODIFICAR ESTO PARA QUE LA LÓGICA SEA CONSISTENTE
        # ES DECIR, QUE SI VARIOS MENSAJES SON VERDAD, SE CONSIDERE UN BACHE

        if max_value > 0.6 :  
        #if self.x_k[0][0] > 1.0:  

            label = "Pothole detected"
            detection_message = String()
            detection_message.data = "Yes"
            
            # Crea una función que saque el contorno de la imagen y sus píxeles 
            #newframe = self.get_pothole_coords(frame)

            # resized_frame es la imagen de 192,192
            newframe = self.get_pothole_coords(resized_frame)

            # para canny y dilate si que necesitamos la linea de debajo
            #newframe = cv2.cvtColor(newframe, cv2.COLOR_GRAY2BGR)

        else:
            label = "No pothole"
            detection_message = String()
            detection_message.data = "No"

            #newframe = frame

            # resized_frame es la imagen de 192,192
            newframe = resized_frame


        # Publicar el mensaje de detección  en formato string
        self.detection_publisher.publish(detection_message)

        # Añadir la etiqueta al marco
        #cv2.putText(newframe, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)

        # Convertir y publicar la imagen con la etiqueta
        ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(newframe, encoding="bgr8")

        self.publisher.publish(ROSImageMessage)

    def cleanup(self):
        self.camera.release()

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received, shutting down...')
        self.cleanup()
        sys.exit(0)  


    def get_contours(self, img, img_contour):
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1500:  # Ignorar pequeños contornos para reducir ruido
                cv2.drawContours(img_contour, cnt, -1,(255,0,255), 5)

                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                
                #print(len(approx))
                #print(approx)
 
    def get_pothole_coords(self, image):

        img_blur = cv2.GaussianBlur(image,(7,7),1)
        img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)

        height, width = img_gray.shape

        # Excluye 100 píxeles en la parte superior e inferior no de los lados
        #min_distance_from_top_bottom = 100  # pixels
        #mask = np.zeros_like(img_gray)
        #mask[min_distance_from_top_bottom:height - min_distance_from_top_bottom, 0:width] = 255

        # Se aplica el filtro Canny a la imagen reducida
        # El valor más acercado es mínimo: 80 y máximo: 180
        img_canny = cv2.Canny(img_gray, 80, 180)
        #img_canny_masked = cv2.bitwise_and(img_canny, mask)
        kernel = np.ones((5,5))
        #img_dilated = cv2.dilate(img_canny_masked, kernel, iterations=1)
        img_dilated = cv2.dilate(img_canny, kernel, iterations=1)

        img_contour = image.copy()
        self.get_contours(img_dilated,img_contour)
        
        return img_contour    

    def kalman_predict(self):
        # Predicción del estado
        x_k_pred = self.A @ self.x_k + self.B @ self.u_k
        # Predicción de la covarianza
        P_k_pred = self.A @ self.P_k @ self.A.T + self.Q
        return x_k_pred, P_k_pred

    def kalman_update(self, x_k_pred, P_k_pred, z_k):
        # Ganancia de Kalman
        K = P_k_pred @ self.H.T @ np.linalg.inv(self.H @ P_k_pred @ self.H.T + self.R)
        # Actualización del estado
        x_k_new = x_k_pred + K @ (z_k - self.H @ x_k_pred)
        # Actualización de la covarianza
        P_k_new = (np.eye(P_k_pred.shape[0]) - K @ self.H) @ P_k_pred
        return x_k_new, P_k_new

    
    def __del__(self):
        self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    publisherObject = CameraTFv3Node()

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
