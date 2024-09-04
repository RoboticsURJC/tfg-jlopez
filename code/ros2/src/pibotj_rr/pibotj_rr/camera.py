import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import signal
import sys

class CameraClass(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        if not self.camera.isOpened():
            self.get_logger().error('Failed to open camera device %d' % self.cameraDeviceNumber)
            rclpy.shutdown()
            sys.exit(1)  # Exit with an error code

        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'camera'
        self.queueSize = 20
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        self.periodCommunication = 0.1  # Reduce to 10 Hz for stability
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

        # Signal handler for cleanup
        signal.signal(signal.SIGINT, self.signal_handler)

    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        if not success:
            self.get_logger().error('Failed to read frame from camera')
            return

        frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_LINEAR)

        ROSImageMessage = self.bridgeObject.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(ROSImageMessage)
        self.get_logger().info('Publishing image number %d' % self.i)
        self.i += 1

    def cleanup(self):
        self.camera.release()

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received, shutting down...')
        self.cleanup()
        sys.exit(0)  # Exit gracefully

def main(args=None):
    rclpy.init(args=args)
    publisherObject = CameraClass()
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
