#!/usr/bin/env python3
"""Run RL wrapper with trained model for online inference."""

import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
from stable_baselines3 import PPO


class RLInferenceNode(Node):
    """ROS node for RL inference using trained model."""
    
    def __init__(self, model_path: str, config_path: str):
        super().__init__("rl_inference_node")
        
        # Load config
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Load trained model
        self.get_logger().info(f"Loading model from {model_path}")
        self.model = PPO.load(model_path)
        self.get_logger().info("Model loaded successfully")
        
        self.bridge = CvBridge()
        self.img_size = self.config['env']['observation_image_size']
        
        # State
        self.current_image = None
        self.classical_error = 0.0
        self.classical_valid = False
        self.current_speed = 0.0
        self.current_steering = 0.0
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Float32, "/rl/correction", 10)
        
        # Subscribers
        self.create_subscription(Image, "/oak/rgb/image_raw", self.image_callback, 10)
        self.create_subscription(Float32, "/lane_detection/center_error", self.error_callback, 10)
        self.create_subscription(Bool, "/lane_detection/valid", self.valid_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        
        # Timer for control loop
        control_freq = self.config['env']['control_frequency']
        self.create_timer(1.0 / control_freq, self.control_loop)
        
        self.get_logger().info("RL Inference node started")
        
    def image_callback(self, msg):
        """Handle camera images."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = cv2.resize(img, self.img_size)
            img = img.astype(np.float32) / 255.0
            self.current_image = img
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
        
    def control_loop(self):
        """Control loop using RL model."""
        if self.current_image is None:
            return
        
        # Get observation
        img_flat = self.current_image.flatten()
        scalars = np.array([self.classical_error, self.current_speed, self.current_steering])
        obs = np.concatenate([img_flat, scalars]).astype(np.float32)
        
        # Get RL action
        action, _ = self.model.predict(obs, deterministic=True)
        rl_correction = action[0]
        
        # Hybrid control: combine classical + RL
        if self.classical_valid:
            # Classical steering from lane detection
            classical_gain = self.config['hybrid'].get('classical_gain', 0.02)
            classical_steering = self.classical_error * classical_gain
            # Add RL correction
            final_steering = classical_steering + rl_correction
        else:
            # Fallback to RL only
            final_steering = rl_correction
        
        # Clamp steering
        max_steering = self.config['safety']['max_steering_angle']
        final_steering = np.clip(final_steering, -max_steering, max_steering)
        
        # Set speed
        target_speed = self.config['safety']['max_speed']
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = target_speed
        cmd.angular.z = final_steering
        self.cmd_pub.publish(cmd)
        
        # Publish debug info
        debug_msg = Float32()
        debug_msg.data = float(rl_correction)
        self.debug_pub.publish(debug_msg)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, 
                        default='./models/enimia_lane_keeping_ppo_offline.zip',
                        help='Path to trained model')
    parser.add_argument('--config', type=str,
                        default='/home/rabeb/Downloads/enimia_black/src/enimia_rl/config/rl_config.yaml',
                        help='Path to config file')
    args = parser.parse_args()
    
    rclpy.init()
    node = RLInferenceNode(args.model, args.config)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
