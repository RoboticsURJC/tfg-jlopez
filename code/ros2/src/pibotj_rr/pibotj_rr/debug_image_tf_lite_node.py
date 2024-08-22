#!/usr/bin/env python3
import os
import sys
import copy
import re
import importlib
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from tflite_msgs.msg import TFLite, TFInference
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from collections import deque
import cv2

if cv2.__version__ < "4.0.0":
    raise ImportError("Requires opencv >= 4.0, "
                      "but found {:s}".format(cv2.__version__))

class DebugImageTFLite(Node):

    def __init__(self):
        super().__init__('debug_image_tf_lite_node')

        debug_image_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Debug image topic to be published to.')

        tf_lite_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='TFLite returns topic.')

        
        image_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Image topic being fed to TFLite.')

        threshold_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Threshold value for inference score.')

        max_returns_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Max number of returns.')

        max_image_queue_size_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Max image message queue size.')

        self.declare_parameter("debug_topic", "/DebugImageReal", 
            debug_image_topic_descriptor)

        self.declare_parameter("tflite_topic", "/TFLiteReal", 
            tf_lite_topic_descriptor)

        self.declare_parameter("image_topic", "/topic_camera_image", 
            image_topic_descriptor)

        self.declare_parameter("threshold", 0.3, 
            threshold_descriptor)

        self.declare_parameter("max_returns", 5, 
            max_returns_descriptor)

        self.declare_parameter("max_queue", 30, 
            max_image_queue_size_descriptor)
        
        self.debugImageTopic = self.get_parameter("debug_topic").value
        self.imageTopic = self.get_parameter("image_topic").value
        self.TFLiteTopic = self.get_parameter("tflite_topic").value
        self.Threshold = float(self.get_parameter("threshold").value)
        self.MaxReturns = int(self.get_parameter("max_returns").value)
        self.MaxQueueSize = int(self.get_parameter("max_queue").value)

        self.ImageSub = self.create_subscription(Image, '{:s}'.format(self.imageTopic), self.ImageCallback, qos_profile_sensor_data)
        self.TFLiteSub = self.create_subscription(TFLite, '{:s}'.format(self.TFLiteTopic), self.TFLiteCallback, 1)
 
        self.ImagePub = self.create_publisher(Image,'{:s}'.format(self.debugImageTopic), qos_profile_sensor_data)
        
        self.ImageFIFO = deque()

        self.bridge = CvBridge()

        self.Lock = False

    def ImageCallback(self, msgImage):
        stampSec = int(msgImage.header.stamp.sec)+int(msgImage.header.stamp.nanosec)/1000000000
        if len(self.ImageFIFO) > self.MaxQueueSize:
            self.ImageFIFO.pop()
            self.ImageFIFO.appendleft(msgImage)
        else:
            self.ImageFIFO.appendleft(msgImage)
        
    def TFLiteCallback(self, msgTFLite):
        if len(self.ImageFIFO) == 0:
            return

        TFLiteStampSec = int(msgTFLite.camera_info.stamp.sec)+int(msgTFLite.camera_info.stamp.nanosec)/1000000000
        for i in range(len(self.ImageFIFO)):
            ImageData = self.ImageFIFO.pop()
            ImageStampSec = int(ImageData.header.stamp.sec)+int(ImageData.header.stamp.nanosec)/1000000000
            if TFLiteStampSec == ImageStampSec and not self.Lock:
                self.ImageDebug(ImageData, msgTFLite)
                break
            if TFLiteStampSec < ImageStampSec:
                self.ImageFIFO.appendleft(ImageData)
        return
        
                
    def ImageDebug(self, msgImage, msgTFlite):
        self.Lock = True
        CVImage = self.bridge.imgmsg_to_cv2(msgImage, "bgr8")
        imageWidth = CVImage.shape[1]
        imageHeight = CVImage.shape[0]
        returnCount = 0
        for inference in msgTFlite.inference:
            
            if inference.score >= self.Threshold and returnCount < self.MaxReturns:
                if inference.bbox[0] < 1.0 and inference.bbox[0] > 0.0:
                    top = inference.bbox[0]
                elif inference.bbox[0] > 1.0:
                    top = 1.0
                elif inference.bbox[0] < 0.0:
                    top = 0

                if inference.bbox[1] < 1.0 and inference.bbox[1] > 0.0:
                    left = inference.bbox[1]
                elif inference.bbox[1] > 1.0:
                    left = 1.0
                elif inference.bbox[1] < 0.0:
                    left = 0

                if inference.bbox[2] < 1.0 and inference.bbox[2] > 0.0:
                    bottom = inference.bbox[2]
                elif inference.bbox[2] > 1.0:
                    bottom = 1.0
                elif inference.bbox[2] < 0.0:
                    bottom = 0

                if inference.bbox[3] < 1.0 and inference.bbox[3] > 0.0:
                    right = inference.bbox[3]
                elif inference.bbox[3] > 1.0:
                    right = 1.0
                elif inference.bbox[3] < 0.0:
                    right = 0

                w = int((right-left)*imageWidth)
                h = int((bottom-top)*imageHeight)
                x = int(left*imageWidth)
                y = int(top*imageHeight)
                cv2.rectangle(CVImage,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(CVImage, '{:s}: {:0.3f}'.format(inference.label, inference.score),(x+10,y+int(h/2)-10),0,0.5,(0,255,0),2)
                returnCount += 1
            
        msgDebugImage = self.bridge.cv2_to_imgmsg(CVImage, "bgr8")
        msgDebugImage.header.stamp = msgImage.header.stamp
        msgDebugImage.header.frame_id = msgImage.header.frame_id + " DEBUG"
        
        self.ImagePub.publish(msgDebugImage)
        self.Lock = False
        
        return 
            
def main(args=None):
    rclpy.init()
    DITFL = DebugImageTFLite()
    rclpy.spin(DITFL)
    DITFL.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()