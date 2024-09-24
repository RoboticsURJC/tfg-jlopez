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

        # Usa 10 Hz
        self.periodCommunication = 0.1  
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
  
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

        # Maneja la señal de Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        print("set signal handler")
        
        self.ema_value = None  # Inicializa el valor EMA en el constructor


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
        # Normalización 
        input_data = (input_data - 127.5) / 127.5 

        # convertir la imagen a int8
        scale, zero_point = self.input_details[0]['quantization']
        input_data = (input_data / scale + zero_point).astype(np.int8)

        # Establecer el tensor deentrada y realizar la inferencia
        # self.input_details[0]['index'] devuelve 0
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

            # Descuantificar la máscara de baches mirando el canal 0 es la clase que buscamos
            pothole_mask = (prediction1[:, :, 0].astype(np.float32) - zero_point) * scale


        else:
            self.get_logger().error('Unexpected number of channels in prediction output')
            return

        
        # Determinar si se ha detectado un bache
        max_value = np.max(pothole_mask)

        print("Value ", max_value)

        ema_value_updated = self.update_ema(max_value)

        print(ema_value_updated)


        if max_value > 0.6 :  

            label = "Pothole detected"
            detection_message = String()
            detection_message.data = "Yes"
            
            # Dibuja una imagen con el contorno más grande 
            newframe = self.extract_pothole_contour(pothole_mask,resized_frame)

        else:
            label = "No pothole"
            detection_message = String()
            detection_message.data = "No"

            # resized_frame es la imagen de 192,192
            newframe = resized_frame


        # Publicar el mensaje de detección  en formato string
        self.detection_publisher.publish(detection_message)

        # Convertir y publicar la imagen con la etiqueta
        ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(newframe, encoding="bgr8")

        self.publisher.publish(ROSImageMessage)

    def cleanup(self):
        self.camera.release()

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received, shutting down...')
        self.cleanup()
        sys.exit(0)  


    #def get_contours(self, img, img_contour):
    #    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #    for cnt in contours:
    #        area = cv2.contourArea(cnt)
    #        if area > 1500:  # Ignorar pequeños contornos para reducir ruido
    #            cv2.drawContours(img_contour, cnt, -1,(255,0,255), 5)

    #            peri = cv2.arcLength(cnt, True)
    #            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                
                #print(len(approx))
                #print(approx)
 
    #def get_pothole_coords(self, image):

    #    img_blur = cv2.GaussianBlur(image,(7,7),1)
    #    img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)

    #    height, width = img_gray.shape

        # Se aplica el filtro Canny a la imagen reducida
        # El valor más acercado es mínimo: 80 y máximo: 180
    #    img_canny = cv2.Canny(img_gray, 80, 180)
        #img_canny_masked = cv2.bitwise_and(img_canny, mask)
    #    kernel = np.ones((5,5))
        #img_dilated = cv2.dilate(img_canny_masked, kernel, iterations=1)
    #    img_dilated = cv2.dilate(img_canny, kernel, iterations=1)

    #    img_contour = image.copy()
    #    self.get_contours(img_dilated,img_contour)
        
    #    return img_contour


    def update_ema(self,new_value):

        alpha = 0.08
        if self.ema_value is None:
            # Inicializa el primer valor
            self.ema_value = new_value  
        else:
            self.ema_value = alpha * new_value + (1 - alpha) * self.ema_value
        return self.ema_value


    #def extract_pothole_contours_and_area(self, pothole_mask, resized_frame):
        # es necesario escalarlo porque sino aparece en pequeño
    #    scale_factor = 192 / 48

        # Binarizar la máscara del bache con un umbral de 0.6
    #    _, binary_mask = cv2.threshold(pothole_mask, 0.6, 1, cv2.THRESH_BINARY)

        # Encontrar los contornos del bache
    #    contours, _ = cv2.findContours(binary_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Dibujar los contornos en la imagen original
    #    img_with_contours = resized_frame.copy()

    #    for cnt in contours:
            # Escalar los contornos al tamaño correcto
    #        cnt = cnt.astype(np.float32) * scale_factor

            # Calcular el área del bache
    #        area = cv2.contourArea(cnt)
    #        print(f"Área del bache detectado: {area} píxeles cuadrados")
            # descartan los contornos pequeños
    #        if area > 200: 

                # Dibujar el contorno escalado en la imagen
    #            cv2.drawContours(img_with_contours, [cnt.astype(np.int32)], -1, (0, 255, 0), 2)

    #    return img_with_contours

    def extract_pothole_contour(self, pothole_mask, resized_frame):
        # Escalar para que aparezca en el tamaño correcto
        scale_factor = 192 / 48

        # Binarizar la máscara del bache
        _, binary_mask = cv2.threshold(pothole_mask, 0.6, 1, cv2.THRESH_BINARY)

        # Encontrar los contornos del bache
        contours, _ = cv2.findContours(binary_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Dibujar los contornos en la imagen original
        img_with_contours = resized_frame.copy()
        largest_contour = None
        largest_area = 0

        for cnt in contours:
            # Escalar los contornos al tamaño correcto
            cnt = cnt.astype(np.float32) * scale_factor

            # Calcular el área del bache
            area = cv2.contourArea(cnt)
            #print(f"Área del bache detectado: {area} píxeles cuadrados")
        
            # Filtrar contornos pequeños
            if area > 200:
                # Mantener el contorno más grande
                if area > largest_area:
                    largest_area = area
                    largest_contour = cnt.astype(np.int32)

        # Dibujar solo el contorno más grande
        if largest_contour is not None:
            cv2.drawContours(img_with_contours, [largest_contour], -1, (0, 255, 0), 2)
            # Convertir a una lista de píxeles (x, y)
            contour_pixels = largest_contour.reshape(-1, 2) 
            print(f"Píxeles del contorno más grande: {contour_pixels}")
            # CALCULAR EL MODELO PIN HOLE AQUÍ

        # a lo mejor solo devolver el píxeles del contorno más grande

        return img_with_contours





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
