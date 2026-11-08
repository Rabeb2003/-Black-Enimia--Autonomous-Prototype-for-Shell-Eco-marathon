#!/usr/bin/env python3
"""Offline Gym environment for Black Enimia RL training from collected data."""

import os
import time
import numpy as np
import json
import glob
from typing import Dict, Tuple, Optional
import yaml

import gymnasium as gym
from gymnasium import spaces


class OfflineEnimiaEnv(gym.Env):
    """Offline Gym environment for Black Enimia lane keeping using collected data."""
    
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
        
        # Data directory
        self.data_dir = "/home/rabeb/Downloads/enimia_black/src/enimia_rl/data"
        
        # Load all observations
        self.observations = self._load_observations()
        self.num_samples = len(self.observations)
        self.current_idx = 0
        
        print(f"Loaded {self.num_samples} observations from {self.data_dir}")
        
        # Action space: RL correction to classical control
        self.action_space = spaces.Box(
            low=-0.2, high=0.2, shape=(1,), dtype=np.float32
        )
        
        # Observation space
        img_size = self.config['env']['observation_image_size']
        obs_dim = img_size[0] * img_size[1] * 3 + 3  # image + 3 scalars
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        
        self.img_size = img_size
        
    def _load_observations(self):
        """Load all observations from JSON files."""
        obs_files = sorted(glob.glob(os.path.join(self.data_dir, "obs_*.json")))
        observations = []
        
        for obs_file in obs_files:
            with open(obs_file, 'r') as f:
                obs = json.load(f)
                observations.append(obs)
        
        return observations
    
    def reset(self, seed: Optional[int] = None) -> Tuple[np.ndarray, Dict]:
        """Reset environment."""
        super().reset(seed=seed)
        
        # Reset to random position
        self.current_idx = np.random.randint(0, self.num_samples)
        self.episode_step = 0
        
        obs = self._get_observation()
        return obs, {}
    
    def _get_observation(self) -> np.ndarray:
        """Get current observation."""
        if self.current_idx >= self.num_samples:
            self.current_idx = 0
            
        obs = self.observations[self.current_idx]
        
        # Flatten image
        img_flat = np.array(obs['image']).flatten()
        
        # Add scalars
        scalars = np.array([obs['error'], obs['speed'], obs['steering']])
        
        # Combine
        full_obs = np.concatenate([img_flat, scalars])
        
        return full_obs.astype(np.float32)
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Step environment."""
        self.episode_step += 1
        self.current_idx = (self.current_idx + 1) % self.num_samples
        
        # Get next observation
        obs = self._get_observation()
        
        # Calculate reward (simplified)
        error = self.observations[self.current_idx]['error']
        reward = -abs(error)  # Penalize lane error
        
        # Episode termination
        terminated = False
        truncated = self.episode_step >= 1000  # Max episode length
        
        info = {}
        
        return obs, reward, terminated, truncated, info
    
    def close(self):
        """Clean up."""
        pass


if __name__ == "__main__":
    # Test the environment
    env = OfflineEnimiaEnv()
    obs, info = env.reset()
    print(f"Observation shape: {obs.shape}")
    
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"Step {i}: reward={reward:.3f}")
        if terminated or truncated:
            obs, info = env.reset()
    
    env.close()
