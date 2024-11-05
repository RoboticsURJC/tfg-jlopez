import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading

class MotorsVFFV2Node(Node):
    def __init__(self):
        super().__init__('motors_vffv2_node')

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

        self.queueSize = 10
        # se tiene que suscribir a velocidad lineal y angular /vff_vel /dl_vel
        self.subscription = self.create_subscription(
            Twist,
            '/vff_vel',
            self.vff_callback,
            self.queueSize) 

        self.subscription = self.create_subscription(
            Twist,
            '/dl_vel',
            self.dl_callback,
            self.queueSize) 

        self.vff_angular_vel = 0.0

        self.vff_vel = None
        self.dl_vel = None

        # Estado inicial de los servos
        self.stop()

        # Para evitar condiciones de carrera
        self.lock = threading.Lock()
    
    def vff_callback(self, msg: Twist):
        # Cuando acaba su flujo después del with, hace join solo
        with self.lock:
            self.vff_angular_vel = msg.angular.z
            self.vff_vel = msg
            self.move_motors()

    def dl_callback(self, msg: Twist):
        # Cuando acaba su flujo después del with, hace join solo
        with self.lock:
            self.dl_vel = msg
            self.move_motors()
    
    def move_motors(self):
        if self.vff_angular_vel != 0.0 and self.vff_vel is not None:
            # Prioridad para el VFF
            print("VFF")
            self.set_vel(self.vff_vel)
        elif self.dl_vel is not None:
            # Prioridad para DL si no hay rotación angular con VFF
            print("DL")
            self.set_vel(self.dl_vel)

    def set_vel(self, vel):
        # Asegúrate de que vel no es None y contiene valores válidos
        if vel is not None:
            lvalue = 0
            rvalue = 0
            if vel.linear.x > 0:

                if vel.angular.z > 0:
                    # gira hacia la izquierda
                    print("girar hacia la izquierda")
                    lvalue = 0.0
                    rvalue = self.set_right_duty_value(abs(vel.angular.z))

                elif vel.angular.z < 0:
                    # gira hacia la derecha
                    print("girar hacia la derecha")
                    lvalue = self.set_left_duty_value(abs(vel.angular.z))
                    rvalue = 0.0

                else: 
                    # va recto
                    print("ir recto")
                    #lvalue = 12.5
                    #rvalue = 2.5
                    lvalue = self.set_left_duty_value(abs(vel.linear.x))
                    rvalue = self.set_right_duty_value(abs(vel.linear.x))

                print("Motor izquierdo " + str(abs(lvalue)) + " Motor derecho " + str(abs(rvalue)))
                self.set_motors_vel(abs(lvalue),abs(rvalue))
            else: 
                self.stop()


    def set_left_duty_value(self, value_decimal):

        angle = (180*value_decimal + 0.5*180)
        
        duty = float(angle) / 18.0 + 2.5 

        return duty

    def set_right_duty_value(self, value_decimal):

        angle = (-180*value_decimal + 0.5*180)

        duty = float(angle) / 18.0 + 2.5

        return duty

    def set_motors_vel(self, lval, rval):
        
        # motor izquierdo
        #self.set_servo_angle(lval, self.servos[0])
        self.servos[0].ChangeDutyCycle(lval) 

        # motor derecho
        #self.set_servo_angle(rval, self.servos[1])
        self.servos[1].ChangeDutyCycle(rval) 



    def stop(self):
        # Desactiva PWM de los 2 motores para pararlos
        self.servos[0].ChangeDutyCycle(0) 
        self.servos[1].ChangeDutyCycle(0) 

    # te tenid que cambiarlo porque he pasado de 20 a 10 Hz
    #def set_servo_angle(self, angle, servo):
        # Ajusta el ángulo del servomotor dado un ángulo específico
        # Calcula el ciclo de trabajo necesario
    #    duty = float(angle) / 18.0 + 2.5 
    #    servo.ChangeDutyCycle(duty) 

    def __del__(self):
        # Limpia y detiene los servomotores y la configuración de GPIO cuando el nodo se destruye
        for servo in self.servos:
            servo.stop()
        # Limpia la configuración de los pines GPIO
        GPIO.cleanup()  

def main(args=None):
    rclpy.init(args=args) 
    controller_node = MotorsVFFV2Node()  
    rclpy.spin(controller_node) 
    controller_node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__':
    main()  
