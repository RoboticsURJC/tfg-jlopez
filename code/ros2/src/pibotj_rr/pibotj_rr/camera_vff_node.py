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
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Inicialización de variables
        self.min_coord = None  # Almacenar las coordenadas del bache
        self.robot_position = [0.0, 0.0]  # Posición inicial del robot
        self.robot_orientation = 0.0  # Orientación inicial del robot en radianes
        self.vff_gain = 1.0  # Factor de ganancia del VFF

        # Configuración de la frecuencia de publicación
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Maneja la señal de Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info('set signal handler')


    def pothole_callback(self, msg: Pose2D):
        # Actualizar las coordenadas del bache
        self.min_coord = msg

    def timer_callback(self):

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
        twist.angular.z = angle - self.robot_orientation  # Ajustar la orientación del robot
        print(twist.linear.x, twist.angular.z)
        #self.velocity_publisher.publish(twist)

        # Actualizar la posición del robot (simulación)
        self.update_robot_position()


    def compute_repulsive_force(self):
        # Calcular la fuerza repulsiva basada en la posición del bache (si existe)
        repulsive_force = [0.0, 0.0]
        if self.min_coord:
            #min_distance = 0.0
            #for point in self.pothole_polygon:
                # Convertir las coordenadas de imagen a coordenadas del robot
                # habrá que usar el modelo pin hole de nuevo
            pothole_x = self.min_coord.x
            pothole_y = self.min_coord.y

                # Calcular la distancia al bache
            distance = math.sqrt((pothole_x - self.robot_position[0]) ** 2 +
                                     (pothole_y - self.robot_position[1]) ** 2)
            print(distance)
                #if (min_distance == 0):
               #     min_distance = distance
                #else:
                #    if (distance < min_distance):
                #        min_distance = distance

            #print(min_distance)
            # si está a menos de 7 cm cm aplicar la repulsión
            if distance < 70.0:  # Si el bache está cerca, aplicar fuerza repulsiva
                force_magnitude = self.vff_gain / distance  # Inversamente proporcional a la distancia
                angle_to_pothole = math.atan2(pothole_y - self.robot_position[1],
                                                  pothole_x - self.robot_position[0])

                # Descomponer la fuerza en componentes x e y
                repulsive_force[0] -= force_magnitude * math.cos(angle_to_pothole)
                repulsive_force[1] -= force_magnitude * math.sin(angle_to_pothole)

                # otra opción es (y, -x) como lo de unibotics

            #self.min_coord = None

        return repulsive_force

    def compute_attractive_force(self):
        # Calcular la fuerza atractiva hacia un punto adelante del robot (dinámico)
        attractive_force = [0.0, 0.0]
        
        # El objetivo está 1 metro adelante en la dirección del robot
        target_x = self.robot_position[0] + 1.0 * math.cos(self.robot_orientation)
        target_y = self.robot_position[1] + 1.0 * math.sin(self.robot_orientation)

        force_magnitude = self.vff_gain  # Mantener constante la fuerza atractiva
        angle_to_target = math.atan2(target_y - self.robot_position[1],
                                     target_x - self.robot_position[0])

        # Descomponer la fuerza en componentes x e y
        attractive_force[0] = force_magnitude * math.cos(angle_to_target)
        attractive_force[1] = force_magnitude * math.sin(angle_to_target)

        return attractive_force

    def update_robot_position(self):
        # Simular actualización de posición del robot (en un caso real, usaría odometría)
        # Esto es un simple ejemplo que podría reemplazarse con lecturas de odometría o posición real del robot
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
