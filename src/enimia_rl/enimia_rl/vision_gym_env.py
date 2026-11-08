#!/usr/bin/env python3
"""Vision-based Gym environment for Black Enimia RL lane keeping (direct from Gazebo)."""

import os
import time
import numpy as np
import cv2
from typing import Dict, Tuple, Optional
import yaml

import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import math


class VisionLaneKeepingEnv(gym.Env):
    """Vision-based Gym environment using Gazebo images and position feedback."""
    
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
        self.node = Node("vision_lane_keeping_env")
        self.bridge = CvBridge()
        
        # Parameters
        self.img_size = tuple(self.config['env']['observation_image_size'])
        self.max_steering = self.config['safety']['max_steering_angle']
        self.target_speed = 0.5  # m/s (further reduced for safer training)
        self.track_width = 3.0  # m (approximate)
        self.max_lateral_error = 2.0  # m (increased to give more time to learn)
        
        # State
        self.current_image = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.current_speed = 0.0
        self.last_obs_time = time.time()
        
        # Subscribers
        image_topic = '/oak/rgb/image_raw'
        self.node.create_subscription(Image, image_topic, self.image_callback, 10)
        self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publisher
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Action space: [steering_angle]
        self.action_space = spaces.Box(
            low=-self.max_steering, high=self.max_steering, shape=(1,), dtype=np.float32
        )
        
        # Observation space: image (H, W, 3) - uint8 for CNN policy
        self.observation_space = spaces.Box(
            low=0, high=255, shape=(*self.img_size, 3), dtype=np.uint8
        )
        
        # Episode tracking
        self.episode_step = 0
        self.max_episode_steps = self.config['env']['max_episode_steps']
        self.start_x = 0.0
        self.start_y = 0.0
        
        self.node.get_logger().info("Vision-based Lane Keeping Environment initialized")
        
    def image_callback(self, msg):
        """Handle camera images."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = cv2.resize(img, self.img_size)
            img = img.astype(np.uint8)  # Keep as uint8 for CNN policy
            self.current_image = img
            self.last_obs_time = time.time()
        except Exception as e:
            self.node.get_logger().error(f"Image callback error: {e}")
    
    def odom_callback(self, msg):
        """Handle odometry messages."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Get yaw from quaternion (manual conversion)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Get speed
        self.current_speed = msg.twist.twist.linear.x
        
    def reset(self, seed: Optional[int] = None) -> Tuple[np.ndarray, Dict]:
        """Reset environment."""
        super().reset(seed=seed)
        
        # Reset state
        self.episode_step = 0
        self.start_x = self.robot_x
        self.start_y = self.robot_y
        
        # Wait for first image
        timeout = 5.0
        start_time = time.time()
        while self.current_image is None and time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        if self.current_image is None:
            self.node.get_logger().warn("No image received during reset")
            # Create dummy observation
            obs = np.zeros((*self.img_size, 3), dtype=np.uint8)
            return obs, {}
        
        self.node.get_logger().info("Image received successfully")
        obs = self.current_image.copy()
        return obs, {}
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Step environment."""
        self.episode_step += 1
        
        # Apply action
        steering = np.clip(action[0], -self.max_steering, self.max_steering)
        
        cmd = Twist()
        cmd.linear.x = self.target_speed
        cmd.angular.z = steering
        self.cmd_pub.publish(cmd)
        
        # Wait for new observation
        time.sleep(0.05)  # 20 Hz control
        rclpy.spin_once(self.node, timeout_sec=0.01)
        
        # Get observation
        obs = self.current_image.copy() if self.current_image is not None else np.zeros((*self.img_size, 3), dtype=np.uint8)
        
        # Calculate reward based on progress and staying on track
        # Progress: forward distance traveled
        dx = self.robot_x - self.start_x
        dy = self.robot_y - self.start_y
        progress = np.sqrt(dx*dx + dy*dy)
        
        # Lateral deviation from starting y (assuming track is along x-axis)
        lateral_error = abs(dy)
        
        # Detect grass in image (green color detection)
        if self.current_image is not None:
            # Convert to float for color detection
            img_float = self.current_image.astype(np.float32) / 255.0
            # Green channel dominance (grass is green)
            green_ratio = np.mean(img_float[:, :, 1])  # Green channel
            # Grass penalty if green is dominant (reduced penalty)
            grass_penalty = -green_ratio * 2.0 if green_ratio > 0.5 else 0.0
        else:
            grass_penalty = 0.0
        
        # Reward components:
        # 1. Progress reward (increased)
        progress_reward = progress * 0.05
        
        # 2. Centering reward (Gaussian - high when centered, low when off-track)
        centering_reward = np.exp(-lateral_error**2 / (2 * 0.5**2)) * 3.0  # Increased from 2.0
        
        # 3. Speed reward (encourage moving, but penalize if stuck in grass)
        if self.current_speed < 0.2:  # Likely stuck in grass
            speed_reward = -0.5  # Reduced penalty for being stuck
        else:
            speed_reward = self.current_speed * 0.5  # Increased reward
        
        # 4. Lateral penalty (reduced penalty)
        lateral_penalty = -lateral_error * 1.0  # Reduced from 3.0
        
        # 5. Steering smoothness penalty (avoid erratic steering)
        steering_penalty = -abs(steering) * 0.05  # Original value
        
        # 6. Alive bonus (reward for staying on track)
        alive_bonus = 0.1
        
        # Total reward
        reward = progress_reward + centering_reward + speed_reward + lateral_penalty + steering_penalty + grass_penalty + alive_bonus
        
        # Termination conditions
        terminated = lateral_error > self.max_lateral_error  # Terminate if too far off-track
        truncated = self.episode_step >= self.max_episode_steps
        
        info = {
            "progress": progress,
            "lateral_error": lateral_error,
            "speed": self.current_speed,
            "steering": steering
        }
        
        return obs, reward, terminated, truncated, info
    
    def close(self):
        """Clean up."""
        # Stop the robot
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    # Test the environment
    env = VisionLaneKeepingEnv()
    obs, info = env.reset()
    print(f"Observation shape: {obs.shape}")
    
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"Step {i}: reward={reward:.3f}, speed={info['speed']:.2f}")
        if terminated or truncated:
            obs, info = env.reset()
    
    env.close()
