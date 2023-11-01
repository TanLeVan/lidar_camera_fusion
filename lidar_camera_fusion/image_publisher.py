#!/usr/bin/env python3
import cv2 as cv
import rclpy 
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
class ImagePublisher(Node):
    def __init__(self):
        self.VIDEO_PATH = "/dev/video2"
        
        super().__init__("image_publisher")
        self.declare_parameter("input_camera",self.VIDEO_PATH )
        self.publisher = self.create_publisher(Image, "front_image",1)
        self.bridge = CvBridge()
        self.cap = cv.VideoCapture(self.get_parameter("input_camera").get_parameter_value().string_value)
        if not self.cap.isOpened():
            print("Error: Cannot open camera")
            exit
        self.timer = self.create_timer(0.1, self.callback)
    def callback(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Cannot recieve frame")
            pass
        img_msg = self.bridge.cv2_to_imgmsg(frame)
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(img_msg)

if __name__ == "__main__":
    rclpy.init()
    imagePublisher = ImagePublisher()
    rclpy.spin(imagePublisher)
    rclpy.shutdown()
