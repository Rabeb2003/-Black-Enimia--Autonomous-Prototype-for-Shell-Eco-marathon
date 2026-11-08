#!/usr/bin/env python3
"""ROS2 wrapper for hybrid RL + classical lane keeping."""

import os
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from stable_baselines3 import PPO


class RLLaneAssistWrapper(Node):
    """ROS2 wrapper that combines classical control with RL corrections."""
    
    def __init__(self, model_path: str = None):
        super().__init__("rl_lane_assist_wrapper")
        
        # Load config
        config_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "rl_config.yaml"
        )
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Load RL model
        self.model = None
        if model_path and os.path.exists(model_path):
            self.model = PPO.load(model_path)
            self.get_logger().info(f"Loaded RL model from {model_path}")
        else:
            self.get_logger().warn("No RL model loaded, using classical control only")
        
        # ROS2 interfaces
        self.bridge = CvBridge()
        
        # State
        self.current_image = None
        self.classical_error = 0.0
        self.classical_valid = False
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.last_obs_time = 0.0
        self.img_size = self.config['env']['observation_image_size']
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Float32, "/rl/correction", 10)
        
        # Subscribers
        # Use Gazebo camera topic for simulation
        self.create_subscription(Image, "/oak/rgb/image_raw", self.image_callback, 10)
        self.create_subscription(Float32, "/lane_detection/center_error", self.error_callback, 10)
        self.create_subscription(Bool, "/lane_detection/valid", self.valid_callback, 10)
        
        # Timer for control loop
        control_freq = self.config['env']['control_frequency']
        self.create_timer(1.0 / control_freq, self.control_loop)
        
        self.get_logger().info("RL Lane Assist wrapper started")
    
    def image_callback(self, msg):
        """Handle camera images."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = cv2.resize(img, self.img_size)
            img = img.astype(np.float32) / 255.0
            self.current_image = img
            self.last_obs_time = self.get_logger().now().nanoseconds * 1e-9
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")
    
    def error_callback(self, msg):
        """Handle classical lane detection error."""
        self.classical_error = msg.data
    
    def valid_callback(self, msg):
        """Handle lane detection validity."""
        self.classical_valid = msg.data
    
    def _get_observation(self) -> np.ndarray:
        """Get current observation for RL model."""
        if self.current_image is None:
            img_flat = np.zeros(self.img_size[0] * self.img_size[1] * 3, dtype=np.float32)
        else:
            img_flat = self.current_image.flatten()
        
        obs = np.concatenate([
            img_flat,
            np.array([self.classical_error], dtype=np.float32),
            np.array([self.current_speed], dtype=np.float32),
            np.array([self.current_steering], dtype=np.float32),
        ])
        return obs
    
    def control_loop(self):
        """Main control loop."""
        # Get RL correction if model is loaded
        rl_correction = 0.0
        if self.model is not None and self.current_image is not None:
            obs = self._get_observation()
            action, _ = self.model.predict(obs, deterministic=True)
            rl_correction = action[0] * self.config['hybrid']['rl_action_scale']
        
        # Combine classical error with RL correction
        if self.config['hybrid']['use_classical_baseline'] and self.classical_valid:
            base_steering = self.classical_error * 0.5
            target_steering = base_steering + rl_correction
        else:
            target_steering = rl_correction
        
        # Clamp steering
        max_angle = self.config['safety']['max_steering_angle']
        target_steering = np.clip(target_steering, -max_angle, max_angle)
        
        # Set constant speed
        target_speed = 2.0
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = target_speed
        cmd.angular.z = target_steering
        self.cmd_pub.publish(cmd)
        
        # Publish debug info
        self.debug_pub.publish(Float32(data=float(rl_correction)))
        
        self.current_steering = target_steering
        self.current_speed = target_speed


def main(args=None):
    rclpy.init(args=args)
    
    # Get model path from parameter or use default
    model_path = "/home/rabeb/Downloads/enimia_black/src/enimia_rl/models/enimia_rl_final.zip"
    
    node = RLLaneAssistWrapper(model_path)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
