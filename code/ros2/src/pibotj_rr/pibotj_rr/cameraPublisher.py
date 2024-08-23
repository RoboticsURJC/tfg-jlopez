import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class PublisherNodeClass(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        if not self.camera.isOpened():
            self.get_logger().error('Failed to open camera device %d' % self.cameraDeviceNumber)
            rclpy.shutdown()
            return

        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 20
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        self.periodCommunication = 0.1 # Reduce to 10 Hz for stability
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

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

    def __del__(self):
        self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    publisherObject = PublisherNodeClass()
    rclpy.spin(publisherObject)
    publisherObject.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
