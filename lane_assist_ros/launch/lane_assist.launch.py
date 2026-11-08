#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('lane_assist_ros')

    return LaunchDescription([
        Node(
            package='lane_assist_ros',
            executable='lane_assist_node.py',
            name='lane_assist_node',
            output='screen',
            parameters=[
                {'camera': '455'},
                {'linear_speed': 0.5},
                {'steering_gain': 0.02},
            ]
        ),
    ])
