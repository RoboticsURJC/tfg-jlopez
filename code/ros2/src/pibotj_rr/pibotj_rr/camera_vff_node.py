import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Polygon
import math
import signal
import sys
from geometry_msgs.msg import Pose2D

class CameraVFFNode(Node):
    def __init__(self):
        super().__init__('vff_node')

        # Suscribir al tópico que da las coordenadas del bache (Polygon)
        self.queueSize = 10
        self.subscription = self.create_subscription(
            Pose2D,
            'min_coords',  # Suponiendo que las coordenadas del bache se publican en este tópico
            self.pothole_callback,
            self.queueSize)

        # Publicar comandos de velocidad
        self.vel_publisher = self.create_publisher(Twist, 'vff_vel', self.queueSize)

        # Inicialización de variables
        self.min_coord = None 
        self.robot_position = [0.0, 0.0] 
        # orientación en radianes
        self.robot_orientation = 0.0  
        self.vff_gain = 1.0 

        # Configuración de la frecuencia de publicación
        # Cada 0.1 segundos se publica = 10 Hz
        #self.timer = self.create_timer(0.1, self.timer_callback)

        # Maneja la señal de Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info('set signal handler')


    def pothole_callback(self, msg: Pose2D):
        # Actualizar las coordenadas del bache
        self.min_coord = msg

        #print(self.min_coord.x)
        #print(self.min_coord.y)


    #def timer_callback(self):print(self.min_coord.x)
        if (self.min_coord.x > 0.0 and self.min_coord.y > 0.0):

            # Bucle de control VFF
            attractive_force = self.compute_attractive_force()
            repulsive_force = self.compute_repulsive_force()

            # Sumar las fuerzas
            resultant_force = [attractive_force[0] + repulsive_force[0],
                           attractive_force[1] + repulsive_force[1]]

            # Convertir la fuerza resultante en una dirección
            angle = math.atan2(resultant_force[1], resultant_force[0])
            linear_speed = min(math.sqrt(resultant_force[0] ** 2 + resultant_force[1] ** 2), 0.5)  # Limitar la velocidad

            # Publicar la velocidad
            twist = Twist()
            twist.linear.x = linear_speed
            # Ajustar la orientación del robot
            twist.angular.z = angle - self.robot_orientation 

            if(twist.linear.x > 0.5):
                twist.linear.x = 0.5

            self.vel_publisher.publish(twist)

            # Actualizar la posición del robot
            self.update_robot_position()


    def compute_repulsive_force(self):
        # Calcular la fuerza repulsiva basada en la posición del bache (si existe)
        repulsive_force = [0.0, 0.0]
        distance = 0
        #if self.min_coord:
           
            # la coordenada es la del mundo real más cercana al bache detectado
            # (usando modelo pinhole)
        pothole_x = self.min_coord.x
        pothole_y = self.min_coord.y

            # Calcular la distancia al bache
        distance = math.sqrt((pothole_x - self.robot_position[0]) ** 2 +
                                     (pothole_y - self.robot_position[1]) ** 2)
        print(pothole_x)
            
            # si el bache está a menos de 7 cm aplicar la repulsión
        # if distance < 70.0
        if pothole_x < 150.0:  
            # Inversamente proporcional a la distancia
            force_magnitude = self.vff_gain / distance 
            angle_to_pothole = math.atan2(pothole_y - self.robot_position[1],
                                                  pothole_x - self.robot_position[0])

            # Descomponer la fuerza en componentes x e y
            repulsive_force[0] -= force_magnitude * math.cos(angle_to_pothole)
            repulsive_force[1] -= force_magnitude * math.sin(angle_to_pothole)

                # otra opción es (y, -x) como lo de unibotics

        return repulsive_force

    def compute_attractive_force(self):
        # Calcular la fuerza atractiva hacia un punto adelante del robot
        attractive_force = [0.0, 0.0]
        
        # El objetivo está 1 metro adelante en la dirección del robot
        target_x = self.robot_position[0] + 1.0 * math.cos(self.robot_orientation)
        target_y = self.robot_position[1] + 1.0 * math.sin(self.robot_orientation)

        # Mantener constante la fuerza atractiva
        force_magnitude = self.vff_gain 
        angle_to_target = math.atan2(target_y - self.robot_position[1],
                                     target_x - self.robot_position[0])

        # Descomponer la fuerza en componentes x e y
        attractive_force[0] = force_magnitude * math.cos(angle_to_target)
        attractive_force[1] = force_magnitude * math.sin(angle_to_target)

        return attractive_force

    def update_robot_position(self):
        # Simular actualización de posición del robot (en un caso real, usaría odometría)
        # Esto es un simple ejemplo que podría reemplazarse con lecturas de odometría o posición real del robot
        # REVISAR ESTO POR SI CAMBIAR LA ORIENTACIÓN DEL ROBOT
        self.robot_position[0] += 0.1 * math.cos(self.robot_orientation)
        self.robot_position[1] += 0.1 * math.sin(self.robot_orientation)

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
