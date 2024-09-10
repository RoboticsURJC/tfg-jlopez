import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import pynmea2

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 9600)
        
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
 
        self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1)
        self.publisher_ = self.create_publisher(String, 'gps_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        line = self.serial_connection.readline()

        # Verificar si la línea está vacía
        if not line:
            self.get_logger().warn("Received an empty line from GPS. Skipping...")
            return
        
        decoded_line = line.decode('ascii', errors='ignore').strip()

        # Asegurarse de que la línea sea una sentencia NMEA válida
        if decoded_line.startswith('$'):
            try:
                message = pynmea2.parse(decoded_line)
                #print(f"pynmea2 parsed message: {message}")

                # Crear y publicar un mensaje con los datos relevantes
                ros_msg = String()
                if hasattr(message, 'latitude') and hasattr(message, 'longitude'):
                    ros_msg.data = f"Lat: {message.latitude}, Lon: {message.longitude}"
                    self.publisher_.publish(ros_msg)
                    self.get_logger().info(f"Published GPS data: {ros_msg.data}")
                else:
                    self.get_logger().warn(f"NMEA message does not contain GPS data: {decoded_line}")

            except pynmea2.ParseError as e:
                self.get_logger().error(f"Failed to parse NMEA sentence: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

