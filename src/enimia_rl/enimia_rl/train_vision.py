#!/usr/bin/env python3
"""Vision-based RL training script using CNN policy."""

import os
import sys
import yaml
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

# Add parent directory to path
sys.path.insert(0, os.path.dirname(__file__))

from vision_gym_env import VisionLaneKeepingEnv


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--resume', type=str, default=None,
                        help='Path to checkpoint to resume training from')
    args = parser.parse_args()
    
    # Load config
    config_path = os.path.join(
        os.path.dirname(__file__), "..", "config", "rl_config.yaml"
    )
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Create environment
    env = VisionLaneKeepingEnv(config_path)
    env = Monitor(env)
    
    # Create PPO agent with CNN policy for vision
    if args.resume:
        print(f"Resuming training from {args.resume}")
        model = PPO.load(args.resume, env=env)
    else:
        model = PPO(
            "CnnPolicy",  # CNN policy for image observations
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
            policy_kwargs={
                "net_arch": {"pi": [256, 256], "vf": [256, 256]},
                "activation_fn": torch.nn.ReLU,
            },
            device="cuda" if torch.cuda.is_available() else "cpu"
        )
    
    # Checkpoint callback
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path="./models/checkpoints/",
        name_prefix="vision_lane_keeping"
    )
    
    # Train
    print("Starting vision-based training...")
    print(f"Total timesteps: {100000}")
    model.learn(
        total_timesteps=100000,
        callback=checkpoint_callback
    )
    
    # Save model
    model.save("./models/vision_lane_keeping_ppo")
    print("Training complete! Model saved as vision_lane_keeping_ppo.zip")
    
    env.close()


if __name__ == "__main__":
    main()
