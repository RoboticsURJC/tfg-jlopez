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
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        if not self.camera.isOpened():
            self.get_logger().error('Failed to open camera device %d' % self.cameraDeviceNumber)
            rclpy.shutdown()
            return

        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'camera_vff'
        self.queueSize = 10
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        # Publicador del array con coordenadas
        #self.polygon_publisher = self.create_publisher(Polygon, 'pothole_coords', self.queueSize)

        # Usa 10 Hz
        self.periodCommunication = 0.1  
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
  
        # Cargar el modelo TFLite
        #self.model_path = '/home/juloau/robot_ws/src/pibotj_rr/custom_model_lite/bestv2_full_integer_quant_edgetpu.tflite'

        #self.interpreter = tflite.Interpreter(model_path=self.model_path, experimental_delegates=[tflite.load_delegate('/usr/lib/aarch64-linux-gnu/libedgetpu.so.1')])
        #self.get_logger().info('loaded interpreter')

        #self.interpreter.allocate_tensors()
        #self.get_logger().info('allocated tensors')
        

        # Obtener detalles de entrada y salida del modelo
        #self.input_details = self.interpreter.get_input_details()
        ##self.output_details = self.interpreter.get_output_details()
        #self.get_logger().info('set input and output details')

        # Maneja la señal de Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info('set signal handler')
        
        #self.ema_value = None  # Inicializa el valor EMA en el constructor

        #self.largest_contour = None  # Variable para almacenar el contorno más grande
        #self.detected = False
        #self.detect_time = 0


    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        if not success:
            self.get_logger().error('Failed to read frame from camera')
            return

        #height, width, channels = frame.shape
        resized_frame = cv2.resize(frame, (192, 192))

        # Redimensionar el marco a las dimensiones requeridas por el modelo
        #input_shape = self.input_details[0]['shape']
        #height, width = input_shape[1], input_shape[2]
        #resized_frame = cv2.resize(frame, (width, height))

        # Convertir la imagen a formato adecuado
        #input_data = np.expand_dims(resized_frame, axis=0).astype(np.float32)
        # Normalización 
        #input_data = (input_data - 127.5) / 127.5 

        # convertir la imagen a int8
        #scale, zero_point = self.input_details[0]['quantization']
        #input_data = (input_data / scale + zero_point).astype(np.int8)

        # Establecer el tensor deentrada y realizar la inferencia
        # self.input_details[0]['index'] devuelve 0
        #self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        #self.interpreter.invoke()

        # Salida 0: tiene forma [1,38,756] y se supone que esta clase no nos hace falta 
        #output_data0 = self.interpreter.get_tensor(self.output_details[0]['index'])
        #prediction0 = np.squeeze(output_data0)

        # Salida 1: tiene forma [1, 48, 48, 32] y esta clase sí se trata del modelo entrenado
        #output_data1 = self.interpreter.get_tensor(self.output_details[1]['index'])
        # elimina la dimensión del batch
        #prediction1 = np.squeeze(output_data1)
    

        # Esto se hace para asegurarse de que el tensor tiene al menos dos canales en su última dimensión. 
        #if prediction1.shape[-1] > 1:

            # Obtener la escala y el punto cero del tensor de salida
            #scale, zero_point = self.output_details[1]['quantization']

            # Descuantificar la máscara de baches mirando el canal 0 es la clase que buscamos
            #pothole_mask = (prediction1[:, :, 0].astype(np.float32) - zero_point) * scale
        #else:
        #    self.get_logger().error('Unexpected number of channels in prediction output')
        #    return

        # Determinar si se ha detectado un bache
        #max_value = np.max(pothole_mask)
        # se usa media móvil exponencial para reducir los picos
        #ema_value_updated = self.update_ema(max_value)

        #newframe = resized_frame.copy()
        #polygon_coords = Polygon()

        #current_time = time.time()
        
        # si se detecta bache, se dibuja el contorno más grande detectado pero sólo 
        # se almacena el contorno más grande detectado en esos 4 segundos
        #if ema_value_updated > 0.6:
        #    pixels = self.extract_contour_pixels(pothole_mask, resized_frame)

            # si hay contorno detectado 
        #    if pixels is not None:
                # si no estaba detectado, se asigna como contorno más grande el actual
        #        if not self.detected:
        #            self.detected = True
        #            self.detect_time = current_time
        #            self.largest_contour = pixels  

                # Ya ha sido detectado
        #        else: 
                    # Comprobar si el contorno sigue siendo el más grande
        #            if len(pixels) > len(self.largest_contour):
        #                self.largest_contour = pixels

                # Dibuja el contorno más grande en la imagen
        #        cv2.drawContours(newframe, [self.largest_contour], -1, (0, 255, 0), 2)
                # Comprobar si han pasado 4 segundos
        #        if (current_time - self.detect_time) > 4:
                    # Convertir las coordenadas al formato del publicador 
        #            polygon_coords = self.convert_coords(self.largest_contour)

        #            self.reset_detection()  # Reinicia la detección

            # No se ha encontrado un contorno
        #    else:  
        #        # Si se había detectado previamente
        #        if self.detected:  
                    # resetea la detección
        #            self.reset_detection()

        # Si no hay detección
        #else:  
            # si no hay detección y estaba detectado previamente, resetear la detección
        #    if self.detected:
        #        self.reset_detection()

        # Publicar las coordenadas y que el nodo de pinhole las convierta
        #self.polygon_publisher.publish(polygon_coords)

        # Convertir y publicar la imagen
        #ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(newframe, encoding="bgr8")



        # Filtro que detecta bien el entorno conocido
        gray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
        th1 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,23,-50)

        # ayuda a quitar los últimos picos blancos de la imagen
        kernel = np.ones((3, 3), np.uint8) 
        opened_th1 = cv2.morphologyEx(th1, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(opened_th1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filtrar los contornos pequeños o irrelevantes
        #line_contours = []
        min_contour_length = 50 
        num_lines = 0
        for contour in contours:
            if cv2.arcLength(contour, True) > min_contour_length:
                #line_contours.append(contour)
                num_lines += 1

        # Contar las líneas detectadas
        #num_lines_detected = len(line_contours)

        print(num_lines)

        bgr_image = cv2.cvtColor(opened_th1, cv2.COLOR_GRAY2BGR)

        ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(bgr_image, encoding="bgr8")

        self.publisher.publish(ROSImageMessage)

    def reset_detection(self):
        self.detected = False
        self.detect_time = 0
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

    
    def detect_lane_side(self,image):

        # Convertir a escala de grises
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
        # Aplicar suavizado y detectar bordes con Canny
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 30, 100)

        # Definir una región de interés para enfocarse en la carretera
        height, width = edges.shape
        region_of_interest = np.array([[
            (0, height),
            (width // 2, height // 2),
            (width, height)
        ]], dtype=np.int32)

        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, region_of_interest, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Detectar líneas con la Transformada de Hough
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=100)

        if lines is not None:
            left_lines = []
            right_lines = []

            # Clasificar líneas como izquierda o derecha
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1)  # Pendiente de la línea

                if slope < 0:  # Líneas del lado izquierdo
                    left_lines.append(line)
                elif slope > 0:  # Líneas del lado derecho
                    right_lines.append(line)

            # Si hay líneas a la izquierda y derecha
            if left_lines and right_lines:
                # Tomar la posición media de las líneas
                left_x_mean = np.mean([line[0][0] for line in left_lines])  # Coordenada X del lado izquierdo
                right_x_mean = np.mean([line[0][0] for line in right_lines])  # Coordenada X del lado derecho

                print(left_x_mean)
                print(right_x_mean)
            
                # Centro de la imagenresized_frame
                center_x = width // 2
                print(center_x)


                # Comparar la posición de las líneas con el centro de la imagen
                if center_x < left_x_mean:
                    print("El robot está en el lado izquierdo de la carretera.")
                elif center_x > right_x_mean:
                    print("El robot está en el lado derecho de la carretera.")
                else:
                    print("El robot está en el centro del carril.")
            else:
                print("No se detectaron suficientes líneas para determinar la posición.")
        else:
            print("No se detectaron líneas.")

        # Mostrar la imagen con bordes y líneas detectadas
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)


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
