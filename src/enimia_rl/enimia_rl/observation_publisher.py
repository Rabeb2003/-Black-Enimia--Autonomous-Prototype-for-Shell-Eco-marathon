#!/usr/bin/env python3
"""ROS node to publish RL observations from camera and lane detection."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class ObservationPublisher(Node):
    """Publishes observations for RL training."""
    
    def __init__(self):
        super().__init__("observation_publisher")
        self.bridge = CvBridge()
        self.img_size = (64, 64)
        
        # Subscribers
        self.create_subscription(Image, "/oak/rgb/image_raw", self.image_callback, 10)
        self.create_subscription(Float32, "/lane_detection/center_error", self.error_callback, 10)
        self.create_subscription(Bool, "/lane_detection/valid", self.valid_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        
        # Publishers
        self.obs_pub = self.create_publisher(Image, "/rl/observation", 10)
        self.obs_json_pub = self.create_publisher(Float32, "/rl/observation_json", 10)
        
        # State
        self.current_image = None
        self.classical_error = 0.0
        self.classical_valid = False
        self.current_speed = 0.0
        self.current_steering = 0.0
        
        self.get_logger().info("Observation publisher started")
        
    def image_callback(self, msg):
        """Handle camera images."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = cv2.resize(img, self.img_size)
            img = img.astype(np.float32) / 255.0
            self.current_image = img
            self.publish_observation()
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
        
    def publish_observation(self):
        """Publish observation for RL training."""
        if self.current_image is None:
            return
            
        # Create observation dict
        obs = {
            "image": self.current_image.tolist(),
            "error": float(self.classical_error),
            "valid": bool(self.classical_valid),
            "speed": float(self.current_speed),
            "steering": float(self.current_steering)
        }
        
        # Publish as JSON string in Float32 message (hack for simplicity)
        obs_msg = Float32()
        obs_msg.data = 0.0  # Placeholder
        self.obs_json_pub.publish(obs_msg)
        
        # Also publish image for debugging
        img_msg = self.bridge.cv2_to_imgmsg((self.current_image * 255).astype(np.uint8), encoding="bgr8")
        self.obs_pub.publish(img_msg)


def main():
    rclpy.init()
    node = ObservationPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
