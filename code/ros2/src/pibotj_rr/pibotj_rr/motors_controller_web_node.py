import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ControllerWebNode(Node):
    def __init__(self):
        super().__init__('motors_controller_web_node')

        # Configuración de los servomotores
        self.servo_pins = [4, 18]  # Pines GPIO para los dos servos
        GPIO.setmode(GPIO.BCM)  # Configura el modo de numeración de los pines GPIO

        # Inicializa los servomotores
        self.servos = []
        for pin in self.servo_pins:
            GPIO.setup(pin, GPIO.OUT)  # Configura el pin como salida
            servo = GPIO.PWM(pin, 20)  # Inicializa PWM en el pin con frecuencia de 50Hz
            servo.start(0)  # Comienza el PWM con un ciclo de trabajo de 0%
            self.servos.append(servo)

        # Suscripción al tópico 'servo_command'
        self.subscription = self.create_subscription(
            String,
            'move_controller',
            self.command_callback,
            10)  # 10 es el tamaño del buffer de la cola de mensajes
        
        # Estado inicial de los servos
        #self.servos_moving = False
        self.stop()

    #def move_servos(self):
        #self.get_logger().info('Moving servos forward')
        # motor izquierdo
        #self.set_servo_angle(180, self.servos[0])
        # motor derecho
        #self.set_servo_angle(0, self.servos[1])
        #self.servos_moving = True
    
    def forward(self):
        #self.get_logger().info('Moving forward')
        # motor izquierdo
        self.set_servo_angle(180, self.servos[0])
        # motor derecho
        self.set_servo_angle(0, self.servos[1])
        #self.servos_moving = True

    def backward(self):
        #self.get_logger().info('Moving backward')
        # motor izquierdo
        self.set_servo_angle(0, self.servos[0])
        # motor derecho
        self.set_servo_angle(180, self.servos[1])
        #self.servos_moving = True

    def left(self):
        #self.get_logger().info('Moving left')
        # motor izquierdo
        #self.set_servo_angle(90, self.servos[0])

        self.servos[0].ChangeDutyCycle(1) 
        # motor derecho
        self.set_servo_angle(0, self.servos[1])

        #self.servos_moving = True

    def right(self):
        #self.get_logger().info('Moving right')
        # motor izquierdo
        self.set_servo_angle(180, self.servos[0])
        # motor derecho
        self.set_servo_angle(90, self.servos[1])
        #self.servos_moving = True

    def stop(self):
        # Detiene los servos en su posición actual
        #self.get_logger().info('Stopping servos')

        # Desactiva PWM de los 2 motores para pararlos
        self.servos[0].ChangeDutyCycle(0) 
        self.servos[1].ChangeDutyCycle(0) 
        #self.servos_moving = False

    
    
    def command_callback(self, msg):
        # Obtiene el contenido del mensaje
        command = msg.data  
        #self.get_logger().info(f'Received command: {command}') 


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
        #else:
        #    self.stop()


    def set_servo_angle(self, angle, servo):
        # Ajusta el ángulo del servomotor dado un ángulo específico
        duty = float(angle) / 18.0 + 2.5  # Calcula el ciclo de trabajo necesario
        servo.ChangeDutyCycle(duty)  # Ajusta el ciclo de trabajo del servomotor

    def __del__(self):
        # Limpia y detiene los servomotores y la configuración de GPIO cuando el nodo se destruye
        for servo in self.servos:
            servo.stop()
        GPIO.cleanup()  # Limpia la configuración de los pines GPIO

def main(args=None):
    rclpy.init(args=args)  # Inicializa el cliente ROS 2
    controller_node = ControllerWebNode()  # Crea una instancia del nodo ControllerNode
    rclpy.spin(controller_node)  # Mantiene el nodo en ejecución
    controller_node.destroy_node()  # Destruye el nodo cuando termina
    rclpy.shutdown()  # Apaga ROS 2

if __name__ == '__main__':
    main()  # Ejecuta la función principal si el archivo se ejecuta directamente
