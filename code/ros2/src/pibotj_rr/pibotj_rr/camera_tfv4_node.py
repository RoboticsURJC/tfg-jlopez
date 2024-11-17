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


class CameraTFv4Node(Node):
    def __init__(self):
        super().__init__('camera_tf4_node')
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        if not self.camera.isOpened():
            self.get_logger().error('Failed to open camera device %d' % self.cameraDeviceNumber)
            rclpy.shutdown()
            return

        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'camera_tf4'
        self.queueSize = 10
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        # Publicador del array con coordenadas
        self.polygon_publisher = self.create_publisher(Polygon, 'pothole_coords', self.queueSize)

        # Publicador del array con coordenadas
        self.polygon_vff_publisher = self.create_publisher(Polygon, 'pothole_vff_coords', self.queueSize)

        # Usa 10 Hz
        self.periodCommunication = 0.1  
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
  
        # Cargar el modelo TFLite
        self.model_path = '/home/juloau/robot_ws/src/pibotj_rr/custom_model_lite/bestv2_full_integer_quant_edgetpu.tflite'

        self.interpreter = tflite.Interpreter(model_path=self.model_path, experimental_delegates=[tflite.load_delegate('/usr/lib/aarch64-linux-gnu/libedgetpu.so.1')])
        self.get_logger().info('loaded interpreter')

        self.interpreter.allocate_tensors()
        self.get_logger().info('allocated tensors')
        

        # Obtener detalles de entrada y salida del modelo
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.get_logger().info('set input and output details')

        # Maneja la señal de Ctrl +C
        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info('set signal handler')
        
        self.ema_value = None 

        self.largest_contour = None 
        self.detected = False
        self.detect_time = 0
        self.detect_vff_time = 0

    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        if not success:
            self.get_logger().error('Failed to read frame from camera')
            return

        height, width, channels = frame.shape

        # Redimensionar la iamgen a las dimensiones requeridas por el modelo
        input_shape = self.input_details[0]['shape']
        height, width = input_shape[1], input_shape[2]
        resized_frame = cv2.resize(frame, (width, height))

        # Convertir la imagen a formato adecuado
        input_data = np.expand_dims(resized_frame, axis=0).astype(np.float32)
        # Normalización 
        input_data = (input_data - 127.5) / 127.5 

        # convertir la imagen a int8
        scale, zero_point = self.input_details[0]['quantization']
        input_data = (input_data / scale + zero_point).astype(np.int8)

        # Establecer el tensor de entrada y realizar la inferencia
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        # Salida 0: tiene forma [1,38,756] y se supone que esta clase no nos hace falta 
        #output_data0 = self.interpreter.get_tensor(self.output_details[0]['index'])
        #prediction0 = np.squeeze(output_data0)

        # Salida 1: tiene forma [1, 48, 48, 32] y esta clase sí se trata del modelo entrenado
        output_data1 = self.interpreter.get_tensor(self.output_details[1]['index'])
        # elimina la dimensión del batch
        prediction1 = np.squeeze(output_data1)
    
        # Esto se hace para asegurarse de que el tensor tiene al menos dos canales en su última dimensión. 
        if prediction1.shape[-1] > 1:

            # Obtener la escala y el punto cero del tensor de salida
            scale, zero_point = self.output_details[1]['quantization']

            # Descuantifica la máscara de baches mirando el canal 0
            pothole_mask_class0 = (prediction1[:, :, 0].astype(np.float32) - zero_point) * scale
            # Descuantifica la máscara de baches mirando el canal 1
            pothole_mask_class1 = (prediction1[:, :, 1].astype(np.float32) - zero_point) * scale

        else:
            self.get_logger().error('Unexpected number of channels in prediction output')
            return

        # Determinar si se ha detectado un bache
        max_value_class0 = np.max(pothole_mask_class0)

        max_value_class1 = np.max(pothole_mask_class1)

        # se usa media móvil exponencial para reducir los picos
        ema_value_updated_class0 = self.update_ema(max_value_class0)

        ema_value_updated_class1 = self.update_ema(max_value_class1)

        newframe = resized_frame.copy()
        polygon_coords = Polygon()
        polygon_vff_coords = Polygon()

        current_time = time.time()
        
        pixels = None
        # si se detecta bache, se dibuja el contorno más grande detectado pero sólo 
        # se almacena el contorno más grande detectado en esos 4 segundos
        if ema_value_updated_class0 > 0.6:
            pixels = self.extract_contour_pixels(pothole_mask_class0, resized_frame)

        elif ema_value_updated_class1 > 0.6:
            pixels = self.extract_contour_pixels(pothole_mask_class1, resized_frame)

        # Si no hay detección
        else:  
            # si no hay detección y estaba detectado previamente, resetear la detección
            if self.detected:
                self.reset_detection()

        # si hay contorno detectado 
        if pixels is not None:
            # si no estaba detectado, se asigna como contorno más grande el actual
            if not self.detected:
                self.detected = True
                self.detect_time = current_time
                self.detect_vff_time = current_time
                self.largest_contour = pixels  

            # Ya ha sido detectado
            else: 
                # Comprobar si el contorno sigue siendo el más grande
                if len(pixels) > len(self.largest_contour):
                    self.largest_contour = pixels

            # Dibuja el contorno más grande en la imagen
            cv2.drawContours(newframe, [self.largest_contour], -1, (0, 255, 0), 2)
            # Comprobar si han pasado 3 segundos
            if (current_time - self.detect_time) > 2:
                # Convertir las coordenadas al formato del publicador 
                polygon_coords = self.convert_coords(self.largest_contour)

                #self.reset_detection()  # Reinicia la detección

            # Publica las coordenadas para el movimiento de vff
            if (current_time - self.detect_vff_time) > 1.0:
                # Convertir las coordenadas al formato del publicador 
                polygon_vff_coords = self.convert_coords(self.largest_contour)

                #self.reset_detection()  # Reinicia la detección

            if (current_time - self.detect_time) > 3.0 and (current_time - self.detect_vff_time) > 1.0:
                self.reset_detection()

        # No se ha encontrado un contorno
        else:  
            # Si se había detectado previamente
            if self.detected:  
                # resetea la detección
                self.reset_detection()

        # Publicar las coordenadas y que el nodo de pinhole las convierta
        self.polygon_publisher.publish(polygon_coords)

        self.polygon_vff_publisher.publish(polygon_vff_coords)

        # Convertir y publicar la imagen
        ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(newframe, encoding="bgr8")

        self.publisher.publish(ROSImageMessage)

    def reset_detection(self):
        self.detected = False
        self.detect_time = 0
        self.detect_vff_time = 0
        self.largest_contour = None

    def convert_coords(self, coords):

        pol = Polygon()
        
        for pixel in coords:
            point = Point32()
            # Coordenada X
            point.x = float(pixel[0])  
            # Coordenada Y
            point.y = float(pixel[1])  
            # Z puede ser 0 ya que es un contorno 2D
            point.z = 0.0  

            pol.points.append(point)

        return pol

    def cleanup(self):
        self.camera.release()

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received, shutting down...')
        self.cleanup()
        sys.exit(0)  

    def update_ema(self,new_value):

        alpha = 0.08
        if self.ema_value is None:
            # Inicializa el primer valor
            self.ema_value = new_value  
        else:
            self.ema_value = alpha * new_value + (1 - alpha) * self.ema_value
        return self.ema_value


    def extract_contour_pixels(self, pothole_mask, resized_frame):
        # Escalar para que aparezca en el tamaño correcto
        scale_factor = 192 / 48

        # Binarizar la máscara del bache
        _, binary_mask = cv2.threshold(pothole_mask, 0.6, 1, cv2.THRESH_BINARY)

        # Encontrar los contornos del bache
        contours, _ = cv2.findContours(binary_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Dibujar los contornos en la imagen original
        largest_contour = None
        largest_area = 0

        for cnt in contours:
            # Escalar los contornos al tamaño correcto
            cnt = cnt.astype(np.float32) * scale_factor

            # Calcular el área del bache
            area = cv2.contourArea(cnt)
        
            # Filtrar contornos pequeños
            if area > 200:
                # Mantener el contorno más grande
                if area > largest_area:
                    largest_area = area
                    largest_contour = cnt.astype(np.int32)

        # Dibujar solo el contorno más grande
        if largest_contour is not None:
            # Convertir a una lista de píxeles (x, y)
            contour_pixels = largest_contour.reshape(-1, 2) 

            return contour_pixels
       
        return None

    def __del__(self):
        self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    publisherObject = CameraTFv4Node()

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
