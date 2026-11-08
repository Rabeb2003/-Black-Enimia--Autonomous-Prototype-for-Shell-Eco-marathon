"""Launch real OAK-D Lite via workspace-built depthai_ros_driver.

Requires: source install/setup.bash (overrides apt driver; no diagnostic_updater shim).

Topics:
  /oak/rgb/image_raw
  /oak/stereo/image_raw
  /oak/imu/data
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    return [
        LogInfo(
            msg=(
                f'[oak_d_lite_real] Starting OAK-D Lite (depthai_ros_driver) '
                f'RGB /{name}/rgb/image_raw depth /{name}/stereo/image_raw'
            )
        ),
        ComposableNodeContainer(
            name=f'{name}_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depthai_ros_driver',
                    plugin='depthai_ros_driver::Camera',
                    name=name,
                    parameters=[params_file],
                ),
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('enimia_black_description')
    cfg = os.path.join(pkg, 'config', 'real_sensors', 'oak_d_lite.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument('name', default_value='oak'),
            DeclareLaunchArgument('params_file', default_value=cfg),
            DeclareLaunchArgument('parent_frame', default_value='oak_link'),
            DeclareLaunchArgument('camera_model', default_value='OAK-D-LITE'),
            OpaqueFunction(function=_setup),
        ]
    )
