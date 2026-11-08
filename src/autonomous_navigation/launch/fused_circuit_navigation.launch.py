#!/usr/bin/env python3
"""
Master Launch File: Fused High-Performance Circuit Navigation + RL Autopilot for ENIMIA Black.
Launches:
1. fused_lane_detector: Anti-grass/anti-barrier corridor centering (roi_top_ratio=0.32).
2. fused_circuit_controller: Fast straights + hairpin grip + RL steering correction.
3. vehicle_encoder_bridge: Steering wheel angle encoder & wheel rotation encoder odometry.
4. lidar_obstacle_monitor: Cartesian-based collision detection & barrier repulsion.
5. rl_fused_circuit_wrapper: RL autopilot (PPO vision/offline) steering correction.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    output_cmd_topic = LaunchConfiguration('output_cmd_topic')
    cruise_speed = LaunchConfiguration('cruise_speed')
    min_curve_speed = LaunchConfiguration('min_curve_speed')
    min_lap_distance = LaunchConfiguration('min_lap_distance')
    finish_radius = LaunchConfiguration('finish_radius')
    scan_topic = LaunchConfiguration('scan_topic')
    max_fps = LaunchConfiguration('max_fps')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rl = LaunchConfiguration('enable_rl')
    rl_model_path = LaunchConfiguration('rl_model_path')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('image_topic', default_value='/oak/rgb/image_raw',
                              description='Camera RGB image topic'),
        DeclareLaunchArgument('output_cmd_topic', default_value='/cmd_vel',
                              description='Target velocity command topic'),
        DeclareLaunchArgument('cruise_speed', default_value='5.8',
                              description='Straight line cruising speed (m/s)'),
        DeclareLaunchArgument('min_curve_speed', default_value='2.8',
                              description='Hairpin curve speed (m/s)'),
        DeclareLaunchArgument('min_lap_distance', default_value='150.0',
                              description='Min track distance before finish stop (m)'),
        DeclareLaunchArgument('finish_radius', default_value='4.5',
                              description='Proximity to start for finish stop (m)'),
        DeclareLaunchArgument('scan_topic', default_value='/livox/scan',
                              description='LiDAR scan topic'),
        DeclareLaunchArgument('max_fps', default_value='30.0',
                              description='Perception framerate limit'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('enable_rl', default_value='false',
                              description='Enable RL autopilot steering correction (disabled by default for rock-solid stability)'),
        DeclareLaunchArgument('rl_model_path',
                              default_value='/home/rabeb/Downloads/enimia_black/src/enimia_rl/models/vision_lane_keeping_ppo.zip',
                              description='Path to trained RL model (.zip)'),

        # 1. Fused Lane & Curve Detector
        Node(
            package='lane_detection',
            executable='fused_lane_detector',
            name='fused_lane_detector',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'output_image_topic': '/lane_detection/output',
                'error_topic': '/lane_detection/center_error',
                'heading_error_topic': '/lane_detection/heading_error',
                'curvature_topic': '/lane_detection/curvature',
                'valid_topic': '/lane_detection/valid',
                'max_fps': max_fps,
                'roi_top_ratio': 0.32,
                'roi_bottom_ratio': 0.90,
                'smoothing_alpha': 0.75,
                'use_sim_time': use_sim_time,
            }]
        ),

        # 2. Fused Circuit Controller & Lap Manager (with RL correction input)
        Node(
            package='lane_detection',
            executable='fused_circuit_controller',
            name='fused_circuit_controller',
            output='screen',
            parameters=[{
                'input_cmd_topic': '/cmd_vel_safe',
                'output_cmd_topic': output_cmd_topic,
                'lane_error_topic': '/lane_detection/center_error',
                'heading_error_topic': '/lane_detection/heading_error',
                'curvature_topic': '/lane_detection/curvature',
                'lane_valid_topic': '/lane_detection/valid',
                'odom_topic': '/odometry/filtered',
                'stop_topic': '/stop_line/detected',
                'stop_sign_topic': '/stop_sign/detected',
                'lidar_bias_topic': '/lidar/barrier_steer_bias',
                'lidar_brake_topic': '/lidar/emergency_brake',
                'rl_correction_topic': '/rl/steering_correction',
                'enable_rl_correction': enable_rl,
                'wheelbase': 1.704,
                'max_steering_angle': 0.45,
                'max_steer_rate': 1.20,
                'cruise_speed': cruise_speed,
                'start_speed': 5.0,
                'min_curve_speed': min_curve_speed,
                'max_speed': 8.2,
                'max_acceleration': 6.0,
                'max_deceleration': 10.0,
                'kp': 1.15,
                'kd': 0.18,
                'kh': 0.35,
                'kc': 0.80,
                'max_curve_feedforward': 0.25,
                'stop_duration': 5.0,
                'min_lookahead': 2.8,
                'max_lookahead': 5.5,
                'min_lap_distance': min_lap_distance,
                'finish_radius': finish_radius,
                'enable_circuit_lap_stop': True,
                'use_sim_time': use_sim_time,
            }]
        ),

        # 3. Vehicle Encoder Bridge
        Node(
            package='lane_detection',
            executable='vehicle_encoder_bridge',
            name='vehicle_encoder_bridge',
            output='screen',
            parameters=[{
                'joint_states_topic': '/joint_states',
                'wheel_radius': 0.18,
                'wheelbase': 1.704,
                'use_sim_time': use_sim_time,
            }]
        ),

        # 4. LiDAR Obstacle & Barrier Monitor (Cartesian collision detection)
        Node(
            package='lane_detection',
            executable='lidar_obstacle_monitor',
            name='lidar_obstacle_monitor',
            output='screen',
            parameters=[{
                'scan_topic': scan_topic,
                'front_angle_deg': 22.0,
                'side_max_angle_deg': 85.0,
                'danger_distance': 1.30,
                'warning_distance': 3.80,
                'barrier_safe_margin': 1.60,
                'use_sim_time': use_sim_time,
            }]
        ),

        # 5. RL Autopilot Wrapper (PPO vision/offline model) - active ONLY when enable_rl:=true
        Node(
            package='enimia_rl',
            executable='rl_fused_circuit_wrapper',
            name='rl_fused_circuit_wrapper',
            output='screen',
            condition=IfCondition(enable_rl),
            parameters=[{
                'model_path': rl_model_path,
                'image_topic': image_topic,
                'rl_action_scale': 0.15,
                'control_frequency': 25.0,
                'use_sim_time': use_sim_time,
            }]
        ),
    ])
