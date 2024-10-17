import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MotorsVFFNode(Node):
    def __init__(self):
        super().__init__('motors_vff_node')

        # Configuración de los servomotores
        # Pines GPIO para los dos servos
        self.servo_pins = [4, 18] 
        # Configura el modo de numeración de los pines GPIO
        GPIO.setmode(GPIO.BCM) 

        # Inicializa los servomotores
        self.servos = []
        for pin in self.servo_pins:
            # Configura el pin como salida
            GPIO.setup(pin, GPIO.OUT)  
            # Inicializa PWM en el pin con frecuencia de 50Hz
            servo = GPIO.PWM(pin, 20)  
            # Comienza el PWM con un ciclo de trabajo de 0%
            servo.start(0) 
            self.servos.append(servo)

        # se tiene que suscribir a velocidad lineal y angular /vff_vel /dl_vel
        self.subscription = self.create_subscription(
            Twist,
            'move_controller',
            self.command_callback,
            10)  # 10 es el tamaño del buffer de la cola de mensajes
        
        # Estado inicial de los servos
        self.stop()
    
    def forward(self):
        # motor izquierdo
        self.set_servo_angle(180, self.servos[0])
        # motor derecho
        self.set_servo_angle(0, self.servos[1])

    def backward(self):
        # motor izquierdo
        self.set_servo_angle(0, self.servos[0])
        # motor derecho
        self.set_servo_angle(180, self.servos[1])

    def left(self):
        # motor izquierdo
        self.servos[0].ChangeDutyCycle(1) 
        # motor derecho
        self.set_servo_angle(0, self.servos[1])

    def right(self):
        # motor izquierdo
        self.set_servo_angle(180, self.servos[0])
        # motor derecho
        self.set_servo_angle(90, self.servos[1])

    def stop(self):
        # Desactiva PWM de los 2 motores para pararlos
        self.servos[0].ChangeDutyCycle(0) 
        self.servos[1].ChangeDutyCycle(0) 

    # cambiar esto por un callback y mirar si el valor de angular de vff es distinto
    # de 0 significa que tendrá más prioridad que la detección de la línea
    # vff es MAS PRIORITARIO y se DETECTA VIENDO SI LA ANGULAR ES DISTINTO DE 0 

    # para que gire cuando haya DL usar un giro de unos 15º  

    # dentro de ese callback  hay que hacer la conversión de valores decimales a valores angulares
    # usando la regla de 3: res = (90 * valor_obtenido) / 0.5 para la rueda derecha  
    # en el giro de hacia la izquierda 
    
    def command_callback(self, msg):
        # Obtiene el contenido del mensaje
        command = msg.data  

        if command == "forward":
            self.forward()
        elif command == "backward":
            self.backward()
        elif command == "right":
            self.right()
        elif command == "left":
            self.left()
        elif command == "stop":
            self.stop()

    def set_servo_angle(self, angle, servo):
        # Ajusta el ángulo del servomotor dado un ángulo específico
        # Calcula el ciclo de trabajo necesario
        duty = float(angle) / 18.0 + 2.5 
        servo.ChangeDutyCycle(duty) 

    def __del__(self):
        # Limpia y detiene los servomotores y la configuración de GPIO cuando el nodo se destruye
        for servo in self.servos:
            servo.stop()
        # Limpia la configuración de los pines GPIO
        GPIO.cleanup()  

def main(args=None):
    rclpy.init(args=args) 
    controller_node = MotorsVFFNode()  
    rclpy.spin(controller_node) 
    controller_node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__':
    main()  
