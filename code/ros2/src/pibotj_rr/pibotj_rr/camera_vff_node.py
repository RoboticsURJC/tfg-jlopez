import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Polygon
import math
import signal
import sys
from geometry_msgs.msg import Pose2D
import time

class CameraVFFNode(Node):
    def __init__(self):
        super().__init__('vff_node')

        # Suscribir al tópico que da las coordenadas del bache (Polygon)
        self.queueSize = 10
        self.subscription = self.create_subscription(
            Pose2D,
            'min_coords', 
            self.pothole_callback,
            self.queueSize)

        # Publicar comandos de velocidad
        self.vel_publisher = self.create_publisher(Twist, 'vff_vel', self.queueSize)

        # Inicialización de variables
        self.min_coord = None 
        # el robot siempre va a tener esta posición
        self.robot_position = [0.0, 0.0]

        self.vff_attractive_gain = 1.0
        self.vff_repulsive_gain = 3.0

        # Maneja la señal de Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info('set signal handler')

        self.last_non_zero_angular = 0.0
        self.last_non_zero_time = 0.0


    def pothole_callback(self, msg: Pose2D):
        # Actualizar las coordenadas del bache
        self.min_coord = msg

        # Bucle de control VFF
        attractive_force = self.compute_attractive_force()
        repulsive_force = self.compute_repulsive_force()

        # Sumar las fuerzas
        resultant_force = [attractive_force[0] + repulsive_force[0],
                           attractive_force[1] + repulsive_force[1]]

        # Convertir la fuerza resultante en una dirección
        #angle = math.atan2(resultant_force[1], resultant_force[0])
        angle = math.atan2(resultant_force[1], resultant_force[0])

        # Como mucho irá a 0.4 la velocidad lineal
        linear_speed = min(math.sqrt(resultant_force[0] ** 2 + resultant_force[1] ** 2), 0.5) 

        # Si la velocidad angular es distinta de 0, almacenarla y registrar el tiempo
        current_time = time.time()
        if angle != 0.0:
            self.last_non_zero_angular = angle
            self.last_non_zero_time = current_time
        # Si la velocidad angular es 0, pero ha pasado menos de 1.5 segundo desde el último valor distinto de 0, usar el último valor
        elif current_time - self.last_non_zero_time < 0.5:
            angle = self.last_non_zero_angular
        else:
            angle = 0.0  # Reiniciar si ha pasado más de 1 segundo sin cambios

        # para evitar que vaya para atrás
        #if angle > 0.0:
        #    angle = 0.5
        #elif angle < 0.0:
        #    angle = -0.5

        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angle 

        #print(twist)

        self.vel_publisher.publish(twist)

    # si el obstáculo está lejos --> fuerza decrece
    # si el obstáculo está cerca --> fuerza crece 
    def compute_repulsive_force(self):
        # La fuerza repulsiva siempre será 0 si no hay un bache a menos de 15 cm 
        repulsive_force = [0.0, 0.0]

        if (self.min_coord.x > 0.0 and self.min_coord.y > 0.0):

            pothole_x = self.min_coord.x
            pothole_y = self.min_coord.y

            # Calcular la distancia al bache
            distance = math.sqrt((pothole_x - self.robot_position[0]) ** 2 +
                                     (pothole_y - self.robot_position[1]) ** 2)

            #print(distance)
   
            # si el bache está a menos de 20 cm aplicar la repulsión 
            if pothole_x < 200.0:  
                # Inversamente proporcional a la distancia
                force_magnitude = self.vff_repulsive_gain / distance 
                angle_to_pothole = math.atan2(pothole_y - self.robot_position[1],
                                                  pothole_x - self.robot_position[0])

                # Descomponer la fuerza en componentes x e y
                repulsive_force[0] -= force_magnitude * math.cos(angle_to_pothole)
                repulsive_force[1] -= force_magnitude * math.sin(angle_to_pothole)

                # La fuerza  repulsiva queda (y, -x)
                #repulsive_force[0] -= force_magnitude * math.sin(angle_to_pothole)
                #repulsive_force[1] += force_magnitude * math.cos(angle_to_pothole)

        return repulsive_force

    # si el obstáculo está lejos --> fuerza constante
    # si el obstáculo está cerca --> fuerza decrece 
    def compute_attractive_force(self):

        attractive_force = [0.0, 0.0]

        # las coordenadas del obstáculo 
        # si no hay obstáculo , las coordenadas de este es 0.0
        obs_x = self.min_coord.x
        obs_y = self.min_coord.y

        # Mantener constante la fuerza atractiva
        force_magnitude = self.vff_attractive_gain 
        angle_to_target = math.atan2(obs_y - self.robot_position[1],
                                     obs_x - self.robot_position[0])

        # Descomponer la fuerza en componentes x e y
        force_magnitude = self.vff_attractive_gain 
        attractive_force[0] = force_magnitude * math.cos(angle_to_target)
        attractive_force[1] = force_magnitude * math.sin(angle_to_target)

        return attractive_force


    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received, shutting down...')
        sys.exit(0)  

def main(args=None):

    rclpy.init(args=args)
    publisherObject = CameraVFFNode()

    try:
        rclpy.spin(publisherObject)
    except KeyboardInterrupt:
        publisherObject.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        publisherObject.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
