#!/usr/bin/env python3
"""Gym environment for Black Enimia hybrid RL lane keeping."""

import os
import time
import numpy as np
import cv2
import threading
from typing import Dict, Tuple, Optional
import yaml

import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class EnimiaLaneKeepingEnv(gym.Env):
    """Gym environment for Black Enimia lane keeping with hybrid RL + classical control."""
    
    metadata = {"render_modes": ["human"], "render_fps": 20}
    
    def __init__(self, config_path: str = None):
        super().__init__()
        
        # Load config
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(__file__), "..", "config", "rl_config.yaml"
            )
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Initialize ROS
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("enimia_rl_env")
        self.bridge = CvBridge()
        
        # Parameters - use hardcoded values for now
        image_topic = '/oak/rgb/image_raw'
        output_image_topic = '/lane_detection/output'
        error_topic = '/lane_detection/center_error'
        valid_topic = '/lane_detection/valid'
        camera = '455'
        self.max_fps = 15.0
        
        # State
        self.current_image = None
        self.classical_error = 0.0
        self.classical_valid = False
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.episode_step = 0
        self.last_obs_time = 0.0
        
        # Publishers
        self.image_pub = self.node.create_publisher(Image, output_image_topic, 10)
        self.error_pub = self.node.create_publisher(Float32, error_topic, 10)
        self.valid_pub = self.node.create_publisher(Bool, valid_topic, 10)
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        
        # Subscribers
        # Use Gazebo camera topic for simulation
        self.node.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.node.create_subscription(
            Float32, "/lane_detection/center_error", self.error_callback, 10
        )
        self.node.create_subscription(
            Bool, "/lane_detection/valid", self.valid_callback, 10
        )
        
        # Action space: RL correction to classical control
        # Action = correction to steering angle [-0.2, 0.2] radians
        self.action_space = spaces.Box(
            low=-0.2, high=0.2, shape=(1,), dtype=np.float32
        )
        
        # Observation space
        # [downsampled_image (64x64x3), classical_error, current_speed, current_steering]
        img_size = self.config['env']['observation_image_size']
        obs_dim = img_size[0] * img_size[1] * 3 + 3  # image + 3 scalars
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        
        self.img_size = img_size
        
    def image_callback(self, msg):
        """Handle camera images."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = cv2.resize(img, self.img_size)
            img = img.astype(np.float32) / 255.0
            self.current_image = img
            self.last_obs_time = time.time()
        except Exception as e:
            self.node.get_logger().error(f"Image callback error: {e}")
    
    def error_callback(self, msg):
        """Handle classical lane detection error."""
        self.classical_error = msg.data
    
    def valid_callback(self, msg):
        """Handle lane detection validity."""
        self.classical_valid = msg.data
    
    def reset(self, seed: Optional[int] = None) -> Tuple[np.ndarray, Dict]:
        """Reset environment."""
        super().reset(seed=seed)
        
        # Reset state
        self.episode_step = 0
        self.current_image = None
        self.classical_error = 0.0
        self.classical_valid = False
        
        # Wait for first image using simple spin
        timeout = 5.0
        start_time = time.time()
        while self.current_image is None and time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        if self.current_image is None:
            self.node.get_logger().warn("No image received during reset")
            # Create dummy observation
            obs = np.zeros(self.observation_space.shape, dtype=np.float32)
            return obs, {}
        
        self.node.get_logger().info("Image received successfully")
        obs = self._get_observation()
        return obs, {}
    
    def _get_observation(self) -> np.ndarray:
        """Get current observation."""
        if self.current_image is None:
            img_flat = np.zeros(self.img_size[0] * self.img_size[1] * 3, dtype=np.float32)
        else:
            img_flat = self.current_image.flatten()
        
        # Concatenate image with scalar observations
        obs = np.concatenate([
            img_flat,
            np.array([self.classical_error], dtype=np.float32),
            np.array([self.current_speed], dtype=np.float32),
            np.array([self.current_steering], dtype=np.float32),
        ])
        return obs
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute one step."""
        self.episode_step += 1
        
        # Apply RL correction to classical control
        rl_correction = action[0] * self.config['hybrid']['rl_action_scale']
        
        # Combine classical error with RL correction
        if self.config['hybrid']['use_classical_baseline'] and self.classical_valid:
            # Classical control provides base steering
            base_steering = self.classical_error * 0.5  # Scale error to steering
            target_steering = base_steering + rl_correction
        else:
            # RL-only mode
            target_steering = rl_correction
        
        # Clamp steering
        max_angle = self.config['safety']['max_steering_angle']
        target_steering = np.clip(target_steering, -max_angle, max_angle)
        
        # Set constant speed
        target_speed = 2.0  # m/s
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = target_speed
        cmd.angular.z = target_steering
        self.cmd_pub.publish(cmd)
        
        # Wait for next observation
        time.sleep(1.0 / self.config['env']['control_frequency'])
        rclpy.spin_once(self.node, timeout_sec=0.01)
        
        # Get observation
        obs = self._get_observation()
        
        # Calculate reward
        reward = self._calculate_reward(rl_correction, target_steering)
        
        # Check termination
        terminated = self._check_termination()
        truncated = self.episode_step >= self.config['env']['max_episode_steps']
        
        info = {
            "classical_error": self.classical_error,
            "classical_valid": self.classical_valid,
            "rl_correction": rl_correction,
            "target_steering": target_steering,
        }
        
        return obs, reward, terminated, truncated, info
    
    def _calculate_reward(self, rl_correction: float, steering: float) -> float:
        """Calculate reward based on performance."""
        reward = 0.0
        
        # Center error penalty
        if self.classical_valid:
            reward += self.config['reward']['center_error_weight'] * abs(self.classical_error)
        
        # Smoothness penalty (avoid jerky corrections)
        reward += self.config['reward']['smoothness_weight'] * abs(rl_correction)
        
        # Speed reward (encourage forward motion)
        reward += self.config['reward']['speed_weight'] * self.current_speed
        
        # Valid lane bonus
        if self.classical_valid:
            reward += self.config['reward']['valid_lane_bonus']
        
        return reward
    
    def _check_termination(self) -> bool:
        """Check if episode should terminate."""
        # Emergency stop if lane invalid
        if (self.config['safety']['emergency_stop_on_invalid'] and 
            not self.classical_valid and self.episode_step > 10):
            return True
        
        # Terminate if no image for too long
        if time.time() - self.last_obs_time > 2.0:
            return True
        
        return False
    
    def close(self):
        """Clean up."""
        self.node.destroy_node()
        rclpy.shutdown()
