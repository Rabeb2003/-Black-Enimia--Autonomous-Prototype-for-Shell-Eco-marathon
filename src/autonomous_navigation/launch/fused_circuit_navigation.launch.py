#!/usr/bin/env python3
"""
Master Launch File: Fused High-Performance Circuit Navigation for ENIMIA Black.
Launches:
1. fused_lane_detector: Anti-grass/anti-barrier corridor centering (roi_top_ratio=0.32).
2. fused_circuit_controller: Fast straights (7.5 m/s), hairpin grip (3.0 m/s), full Ackermann (0.45 rad), 5s stop.
3. vehicle_encoder_bridge: Steering wheel angle encoder & wheel rotation encoder odometry.
4. lidar_obstacle_monitor: Real-time obstacle / barrier distance alerts & proactive repulsion (margin=2.20m).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('image_topic', default_value='/oak/rgb/image_raw',
                              description='Camera RGB image topic'),
        DeclareLaunchArgument('output_cmd_topic', default_value='/cmd_vel_lane',
                              description='Target velocity command topic (/cmd_vel_lane for safety gate or /cmd_vel)'),
        DeclareLaunchArgument('cruise_speed', default_value='7.5',
                              description='Straight line cruising speed (m/s)'),
        DeclareLaunchArgument('min_curve_speed', default_value='3.0',
                              description='Hairpin curve speed for zero tire-slip (m/s)'),
        DeclareLaunchArgument('min_lap_distance', default_value='150.0',
                              description='Minimum track distance (m) before return to start stops the car'),
        DeclareLaunchArgument('finish_radius', default_value='4.5',
                              description='Proximity to start line to trigger finish stop (m)'),
        DeclareLaunchArgument('scan_topic', default_value='/livox/scan',
                              description='LiDAR scan topic for obstacle alerts and barrier repulsion'),
        DeclareLaunchArgument('max_fps', default_value='30.0',
                              description='Perception execution framerate limit'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),

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

        # 2. Fused Circuit Controller & Lap Manager
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

        # 3. Vehicle Encoder Bridge (Steering wheel angle & wheel rotation odometry)
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

        # 4. LiDAR Obstacle & Barrier Monitor (Collision alerts and active barrier repulsion)
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
                'barrier_safe_margin': 2.20,
                'use_sim_time': use_sim_time,
            }]
        ),
    ])
