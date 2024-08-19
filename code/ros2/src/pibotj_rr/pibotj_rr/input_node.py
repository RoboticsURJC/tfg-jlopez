import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher_ = self.create_publisher(String, 'servo_command', 10)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {command}")

def main(args=None):
    rclpy.init(args=args)
    input_node = InputNode()

    while rclpy.ok():
        command = input("Enter command (S1:<angle1>,<angle2>): ")
        if command.startswith('S1:'):
            try:
                angles = command[3:].split(',')
                angle1 = int(angles[0])
                angle2 = int(angles[1]) if len(angles) > 1 else None
                if 0 <= angle1 <= 180 and (angle2 is None or 0 <= angle2 <= 180):
                    command_str = f'S1:{angle1}'
                    if angle2 is not None:
                        command_str += f',{angle2}'
                    input_node.publish_command(command_str)
                else:
                    print("Angle out of range. Valid range is 0 to 180.")
            except ValueError:
                print("Invalid angles. Please enter integer values.")
        else:
            print("Invalid command. Use the format S1:<angle1>,<angle2>.")

    input_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
