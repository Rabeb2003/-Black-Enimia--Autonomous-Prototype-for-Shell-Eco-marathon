#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('lane_follower')

    return LaunchDescription([
        # Lane Detector – 카메라 이미지 → 차선 Poly-fit 계수
        Node(
            package='lane_follower',
            executable='lane_detector.py',
            name='lane_detector',
            output='screen',
            parameters=[
                {'fixed_points': '[(10,450),(540,430),(435,240),(190,240)]'},
                {'x_m_per_pixel': 0.0016472868217054263},
                {'y_m_per_pixel': 0.003703703703703704}
            ]
        ),

        # Path Planner – 차선 → Look-ahead 점
        Node(
            package='lane_follower',
            executable='path_planner.py',
            name='path_planner',
            output='screen',
            parameters=[
                {'lookahead_dist': 0.85},
                {'shift_distance': 0.425}
            ]
        ),

        # Pure-Pursuit Controller – Look-ahead → 스티어·스로틀
        Node(
            package='lane_follower',
            executable='pure_pursuit_ctrl.py',
            name='pure_pursuit_ctrl',
            output='screen',
            parameters=[
                {'wheel_base': 0.55},
                {'throttle': 0.8}
            ]
        ),
    ])
