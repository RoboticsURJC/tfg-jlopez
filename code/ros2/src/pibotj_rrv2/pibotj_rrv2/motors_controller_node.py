import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ControllerNode(Node):
    def __init__(self):
        super().__init__('motors_controller_node')

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
            'pothole_detected',
            self.command_callback,
            10)  # 10 es el tamaño del buffer de la cola de mensajes
        
        # Estado inicial de los servos
        self.servos_moving = False
        self.move_servos()

    def move_servos(self):
        self.get_logger().info('Moving servos forward')
        # motor izquierdo
        self.set_servo_angle(180, self.servos[0])
        # motor derecho
        self.set_servo_angle(0, self.servos[1])
        self.servos_moving = True

    def stop_servos(self):
        # Detiene los servos en su posición actual
        self.get_logger().info('Stopping servos')
        # motor izquierdo
        #self.set_servo_angle(90, self.servos[0]) 
        # motor derecho
        #self.set_servo_angle(90, self.servos[1])

        # Desactiva PWM de los 2 motores para pararlos
        self.servos[0].ChangeDutyCycle(0) 
        self.servos[1].ChangeDutyCycle(0) 

        self.servos_moving = False

    
    
    def command_callback(self, msg):
        # Función llamada cuando se recibe un mensaje en el tópico 'servo_command'
        command = msg.data  # Obtiene el contenido del mensaje
        self.get_logger().info(f'Received command: {command}')  # Imprime el comando recibido en el log

        # MODIFICAR ESTE LÓGICA PARA QUE DESPUÉS DE X TIEMPO SIGA HACIA ADELANTE 
        # Y DESCUBRA NUEVOS BACHES
        if command == "Yes" and self.servos_moving:
            self.stop_servos()
        elif command == "No" and not self.servos_moving:
            self.move_servos()


        #if command.startswith('S'):
        #    parts = command.split(':')  # Divide el comando en partes separadas por ':'
        #    angle = int(parts[1])  # Obtiene el ángulo deseado del comando
        #    servo_number = int(parts[0][1:])  # Obtiene el número del servomotor del comando

        #    if 1 <= servo_number <= 2:
                # Ajusta el ángulo del servomotor correspondiente
        #        self.set_servo_angle(angle, self.servos[servo_number - 1])
        #    else:
        #        self.get_logger().error("Invalid servo number")  # Imprime un error si el número del servomotor no es válido

        #if command.startswith('S'):
        #    parts = command.split(':')  # Divide el comando en partes separadas por ':'
        #    angles = parts[1].split(',')  # Obtiene los ángulos deseados para los servos
        #    servo_number = int(parts[0][1:])  # Obtiene el número del servomotor del comando

        #    if 1 <= servo_number <= 2:
                # Ajusta el ángulo del servomotor correspondiente
        #        self.set_servo_angle(int(angles[0]), self.servos[0])
        #        if len(angles) > 1:
        #            self.set_servo_angle(int(angles[1]), self.servos[1])
        #    else:
        #        self.get_logger().error("Invalid servo number")  # Imprime un error si el número del servomotor no es válido


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
    controller_node = ControllerNode()  # Crea una instancia del nodo ControllerNode
    rclpy.spin(controller_node)  # Mantiene el nodo en ejecución
    controller_node.destroy_node()  # Destruye el nodo cuando termina
    rclpy.shutdown()  # Apaga ROS 2

if __name__ == '__main__':
    main()  # Ejecuta la función principal si el archivo se ejecuta directamente
