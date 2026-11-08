# Enimia Black - Autonomous Prototype for Shell Eco-marathon

## Overview

Enimia Black is an autonomous vehicle prototype designed for the Shell Eco-marathon Europe and Africa competition. This project implements advanced autonomous driving capabilities using ROS2 (Robot Operating System 2) with a focus on energy efficiency and precise navigation.

## Project Description

### Competition Context
The Shell Eco-marathon is a global competition that challenges student teams to design, build, and test energy-efficient vehicles. Enimia Black participates in the autonomous category, where the vehicle must navigate a predefined track without human intervention while maximizing energy efficiency.

### Key Features

- **Autonomous Navigation**: Complete autonomous driving system using ROS2 Navigation Stack (Nav2)
- **Lane Detection**: Ultra-fast lane detection using deep learning models
- **Path Planning**: Cubic spline interpolation for smooth trajectory generation
- **Sensor Fusion**: Integration of multiple sensors including OAK-D camera
- **Gazebo Simulation**: Full simulation environment for testing and development
- **Reinforcement Learning**: RL-based driving policy optimization
- **Energy Optimization**: Focus on minimal energy consumption during autonomous operation

## Hardware Components

- **OAK-D Camera**: Depth AI camera for visual perception
- **Lidar**: 2D/3D lidar for obstacle detection and mapping
- **IMU**: Inertial Measurement Unit for orientation and motion tracking
- **Motor Controllers**: Precision motor control for steering and propulsion
- **Battery Management**: Energy monitoring and optimization system

## Software Architecture

### ROS2 Packages

1. **enimia_black_description**: URDF models and Gazebo simulation files
2. **enimia_rl**: Reinforcement learning environment and training scripts
3. **oak_d_camera**: OAK-D camera driver and image processing
4. **waypoints_calculations**: Path planning and trajectory generation
5. **lane_assist_ros**: Lane detection and tracking system
6. **lane_follower**: Lane following control algorithms

### Navigation Stack

- **SLAM**: Simultaneous Localization and Mapping for environment mapping
- **AMCL**: Adaptive Monte Carlo Localization for precise positioning
- **Nav2**: ROS2 Navigation Stack for global and local path planning
- **Costmap**: Dynamic obstacle avoidance and path optimization

## Installation

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- CUDA-capable GPU (for deep learning models)

### Build Instructions

```bash
# Clone the repository
git clone https://github.com/your-username/enimia_black.git
cd enimia_black

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### Simulation

```bash
# Launch Gazebo simulation with autonomous navigation
ros2 launch enimia_black_description gazebo_nav2_autonomous.launch.py

# Launch SLAM for mapping
ros2 launch enimia_black_description slam.launch.py

# Launch navigation with existing map
ros2 launch enimia_black_description navigation.launch.py map:=/path/to/map.yaml
```

### Real Vehicle

```bash
# Launch camera driver
ros2 launch oak_d_camera camera.launch.py

# Launch lane detection
ros2 launch lane_assist_ros lane_detection.launch.py

# Launch autonomous navigation
ros2 launch enimia_black_description autonomous_navigation.launch.py
```

## Training

### Reinforcement Learning

```bash
# Train RL model
cd src/enimia_rl
python enimia_rl/train.py

# Train vision model
python enimia_rl/train_vision.py
```

### Lane Detection

```bash
# Train lane detection model
cd Ultra-Fast-Lane-Detection-master
python train.py
```

## Configuration

### Maps
Maps are stored in the `maps/` directory. To create a new map:

```bash
ros2 run slam_toolbox async_slam_toolbox_node
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

### Waypoints
Waypoints can be generated using the waypoints creator:

```bash
ros2 launch waypoints_niagara_creator waypoints.launch.py
```

## Team

This project is developed by the Enimia team for the Shell Eco-marathon competition.

## License

This project is licensed under the MIT License - see LICENSE file for details.

## Acknowledgments

- Shell Eco-marathon organization
- ROS2 community
- Open-source autonomous driving projects
