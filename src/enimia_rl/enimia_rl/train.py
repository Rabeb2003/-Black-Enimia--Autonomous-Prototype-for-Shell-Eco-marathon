#!/usr/bin/env python3
"""Training script for hybrid RL lane keeping."""

import os
import yaml
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor

from enimia_rl.gym_env import EnimiaLaneKeepingEnv


def main():
    # Load config
    config_path = os.path.join(
        os.path.dirname(__file__), "..", "config", "rl_config.yaml"
    )
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Create environment
    env = EnimiaLaneKeepingEnv(config_path)
    env = Monitor(env)
    
    # Create PPO agent (without VecEnv wrapper for debugging)
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log="./logs/tensorboard/",
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )
    
    # Train
    print("Starting training...")
    print(f"Total timesteps: {10000}")
    model.learn(total_timesteps=10000)
    
    # Save model
    model.save("./models/enimia_lane_keeping_ppo")
    print("Training complete! Model saved as enimia_lane_keeping_ppo.zip")
    
    env.close()


if __name__ == "__main__":
    main()
