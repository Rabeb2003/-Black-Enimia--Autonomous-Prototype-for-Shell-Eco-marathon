#!/usr/bin/env python3
"""ROS node to collect training data for RL."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
import time


class DataCollector(Node):
    """Collects training data from ROS topics."""
    
    def __init__(self):
        super().__init__("data_collector")
        self.bridge = CvBridge()
        self.img_size = (64, 64)
        
        # Data directory
        self.data_dir = "/home/rabeb/Downloads/enimia_black/src/enimia_rl/data"
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Subscribers
        self.create_subscription(Image, "/oak/rgb/image_raw", self.image_callback, 10)
        self.create_subscription(Float32, "/lane_detection/center_error", self.error_callback, 10)
        self.create_subscription(Bool, "/lane_detection/valid", self.valid_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        
        # State
        self.current_image = None
        self.classical_error = 0.0
        self.classical_valid = False
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.frame_count = 0
        
        self.get_logger().info("Data collector started")
        
    def image_callback(self, msg):
        """Handle camera images."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = cv2.resize(img, self.img_size)
            img = img.astype(np.float32) / 255.0
            self.current_image = img
            self.save_observation()
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")
    
    def error_callback(self, msg):
        """Handle classical lane detection error."""
        self.classical_error = msg.data
        
    def valid_callback(self, msg):
        """Handle lane detection validity."""
        self.classical_valid = msg.data
        
    def cmd_vel_callback(self, msg):
        """Handle command velocity."""
        self.current_speed = msg.linear.x
        self.current_steering = msg.angular.z
        
    def save_observation(self):
        """Save observation to file."""
        if self.current_image is None:
            return
            
        self.frame_count += 1
        
        # Save observation
        obs = {
            "image": self.current_image.tolist(),
            "error": float(self.classical_error),
            "valid": bool(self.classical_valid),
            "speed": float(self.current_speed),
            "steering": float(self.current_steering),
            "timestamp": time.time()
        }
        
        # Save to JSON file
        filename = os.path.join(self.data_dir, f"obs_{self.frame_count:06d}.json")
        with open(filename, 'w') as f:
            json.dump(obs, f)
        
        if self.frame_count % 100 == 0:
            self.get_logger().info(f"Collected {self.frame_count} observations")


def main():
    rclpy.init()
    node = DataCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
