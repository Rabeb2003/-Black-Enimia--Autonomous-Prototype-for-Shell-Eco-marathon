#!/usr/bin/env python3
"""Run vision-based RL inference with trained model."""

import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
from stable_baselines3 import PPO


class VisionRLInferenceNode(Node):
    """ROS node for vision-based RL inference."""
    
    def __init__(self, model_path: str, config_path: str):
        super().__init__("vision_rl_inference_node")
        
        # Load config
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Load trained model
        self.get_logger().info(f"Loading model from {model_path}")
        self.model = PPO.load(model_path)
        self.get_logger().info("Model loaded successfully")
        
        self.bridge = CvBridge()
        self.img_size = tuple(self.config['env']['observation_image_size'])
        self.max_steering = self.config['safety']['max_steering_angle']
        self.target_speed = 2.0
        
        # State
        self.current_image = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.current_speed = 0.0
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Subscribers
        self.create_subscription(Image, "/oak/rgb/image_raw", self.image_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        
        # Timer for control loop
        control_freq = 20  # Hz
        self.create_timer(1.0 / control_freq, self.control_loop)
        
        self.get_logger().info("Vision RL Inference node started")
        
    def image_callback(self, msg):
        """Handle camera images."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = cv2.resize(img, self.img_size)
            img = img.astype(np.float32) / 255.0
            self.current_image = img
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")
    
    def odom_callback(self, msg):
        """Handle odometry messages."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.current_speed = msg.twist.twist.linear.x
        
    def control_loop(self):
        """Control loop using RL model."""
        if self.current_image is None:
            return
        
        # Get observation (image)
        obs = self.current_image.copy()
        
        # Get RL action
        action, _ = self.model.predict(obs, deterministic=True)
        steering = action[0]
        
        # Clamp steering
        steering = np.clip(steering, -self.max_steering, self.max_steering)
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = self.target_speed
        cmd.angular.z = steering
        self.cmd_pub.publish(cmd)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, 
                        default='./models/vision_lane_keeping_ppo.zip',
                        help='Path to trained model')
    parser.add_argument('--config', type=str,
                        default='/home/rabeb/Downloads/enimia_black/src/enimia_rl/config/rl_config.yaml',
                        help='Path to config file')
    args = parser.parse_args()
    
    rclpy.init()
    node = VisionRLInferenceNode(args.model, args.config)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.cmd_pub.publish(cmd)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
