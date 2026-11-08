"""Mapping matériel: capteurs réels + slam_toolbox sur /livox/scan.

Prérequis:
  source /opt/ros/humble/setup.bash
  source ~/ws_livox/install/setup.bash
  source ~/Downloads/enimia_black/install/setup.bash

slam_toolbox a besoin d'une odométrie (TF odom -> base_footprint).
Sans odométrie véhicule, fournir use_fake_odom:=true (debug uniquement)
ou lancer FAST-LIO / robot_localization en amont.
"""

from __future__ import annotations

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(value: str) -> bool:
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _pkg_share(name: str) -> str | None:
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        return None


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('enimia_black_description')
    workspace_root = os.environ.get(
        'ENIMIA_BLACK_WORKSPACE',
        '/home/rabeb/Downloads/enimia_black',
    )
    maps_dir = os.path.join(workspace_root, 'maps')
    os.makedirs(maps_dir, exist_ok=True)

    slam_params = LaunchConfiguration('slam_params_file').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    use_fake_odom = LaunchConfiguration('use_fake_odom').perform(context)
    save_map = LaunchConfiguration('save_map').perform(context)
    map_output = LaunchConfiguration('map_output').perform(context)
    save_delay = LaunchConfiguration('save_delay').perform(context)

    actions = []

    if _pkg_share('livox_ros_driver2') is None:
        actions.append(
            LogInfo(
                msg=(
                    '[real_mapping] ERROR: livox_ros_driver2 not found. '
                    'Run: source ~/ws_livox/install/setup.bash'
                )
            )
        )
    else:
        actions.append(
            LogInfo(msg='[real_mapping] livox_ros_driver2 OK — starting sensors + SLAM.')
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'real_sensors.launch.py')
            ),
            launch_arguments={
                'use_oak': LaunchConfiguration('use_oak'),
                'use_imu_filter': LaunchConfiguration('use_imu_filter'),
                'use_livox': 'true',
                'use_robot_state_publisher': LaunchConfiguration(
                    'use_robot_state_publisher'
                ),
                'use_rviz': 'false',
                'livox_config_file': LaunchConfiguration('livox_config_file'),
            }.items(),
        )
    )

    if _is_true(use_fake_odom):
        # Identity odom for bench tests without wheel odometry (not for real mapping).
        actions.append(
            LogInfo(
                msg=(
                    '[real_mapping] WARNING: use_fake_odom:=true — '
                    'static odom->base_footprint (debug only).'
                )
            )
        )
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='fake_odom_tf',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'odom',
                    '--child-frame-id', 'base_footprint',
                ],
                output='screen',
            )
        )

    actions.append(
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params, {'use_sim_time': False}],
            output='screen',
        )
    )

    if _is_true(use_rviz):
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': False}],
                output='screen',
            )
        )

    if _is_true(save_map):
        try:
            delay = float(save_delay)
        except ValueError:
            delay = 120.0
        out_dir = os.path.dirname(map_output)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                            '-t', '/map',
                            '-f', map_output,
                            '--fmt', 'png',
                        ],
                        output='screen',
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    pkg = get_package_share_directory('enimia_black_description')
    workspace_root = os.environ.get(
        'ENIMIA_BLACK_WORKSPACE',
        '/home/rabeb/Downloads/enimia_black',
    )
    maps_dir = os.path.join(workspace_root, 'maps')
    cfg_dir = os.path.join(pkg, 'config', 'real_sensors')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_oak', default_value='true'),
            DeclareLaunchArgument('use_imu_filter', default_value='true'),
            DeclareLaunchArgument('use_robot_state_publisher', default_value='true'),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Open mapping RViz config',
            ),
            DeclareLaunchArgument(
                'use_fake_odom',
                default_value='false',
                description=(
                    'Publish static odom->base_footprint (bench only). '
                    'Prefer real odometry / FAST-LIO for mapping.'
                ),
            ),
            DeclareLaunchArgument(
                'slam_params_file',
                default_value=os.path.join(
                    pkg, 'config', 'slam_toolbox_mapping_real.yaml'
                ),
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=os.path.join(
                    pkg, 'rviz', 'mapping_slam_waypoints.rviz'
                ),
            ),
            DeclareLaunchArgument(
                'livox_config_file',
                default_value=os.path.join(cfg_dir, 'MID360_config.json'),
            ),
            DeclareLaunchArgument('save_map', default_value='false'),
            DeclareLaunchArgument(
                'map_output',
                default_value=os.path.join(maps_dir, 'real_livox_map'),
            ),
            DeclareLaunchArgument('save_delay', default_value='120.0'),
            OpaqueFunction(function=_setup),
        ]
    )
