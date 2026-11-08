#!/usr/bin/env python3
"""Test script to verify camera subscription works."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time


class CameraTest(Node):
    def __init__(self):
        super().__init__("camera_test")
        self.bridge = CvBridge()
        self.image_count = 0
        self.create_subscription(Image, "/oak/rgb/image_raw", self.callback, 10)
        self.get_logger().info("Subscribed to /oak/rgb/image_raw")
        
    def callback(self, msg):
        self.image_count += 1
        self.get_logger().info(f"Received image #{self.image_count}")
        if self.image_count >= 5:
            self.get_logger().info("Received 5 images successfully!")
            self.destroy_node()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CameraTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
