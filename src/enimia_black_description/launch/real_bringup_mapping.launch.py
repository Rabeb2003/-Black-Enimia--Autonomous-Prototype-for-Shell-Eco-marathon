"""Bringup réel: prototype au sol + LiDAR fixé sur livox_link + odom roues/téléop.

Architecture TF (RViz = réalité pendant téléguidage):
  map -> odom -> base_footprint -> (URDF) base_link -> livox_link / oak_link
  Le nuage LiDAR est en frame livox_link (fixé sur le prototype).
  FAST-LIO peut tourner pour la carte PCD, mais NE déplace PAS le RobotModel.

Odométrie:
  - défaut: intégration /cmd_vel (teleop) via wheel_teleop_odom.py
  - plus tard: odom_source:=wheel + /wheel_odom (encodeurs)
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


def _pkg_share(name: str) -> str | None:
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        return None


def _is_true(value: str) -> bool:
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('enimia_black_description')
    workspace_root = os.environ.get(
        'ENIMIA_BLACK_WORKSPACE',
        '/home/rabeb/Downloads/enimia_black',
    )
    maps_dir = os.path.join(workspace_root, 'maps')
    captures_dir = os.path.join(workspace_root, 'captures')
    os.makedirs(maps_dir, exist_ok=True)
    os.makedirs(captures_dir, exist_ok=True)

    use_oak = _is_true(LaunchConfiguration('use_oak').perform(context))
    use_camera_windows = _is_true(
        LaunchConfiguration('use_camera_windows').perform(context)
    )
    use_frame_capture = _is_true(
        LaunchConfiguration('use_frame_capture').perform(context)
    )
    use_fast_lio = _is_true(LaunchConfiguration('use_fast_lio').perform(context))
    use_wheel_odom = _is_true(LaunchConfiguration('use_wheel_odom').perform(context))
    use_livox_cloud = _is_true(LaunchConfiguration('use_livox_cloud').perform(context))
    use_rviz = _is_true(LaunchConfiguration('use_rviz').perform(context))
    use_teleop = _is_true(LaunchConfiguration('use_teleop').perform(context))

    rgb_topic = LaunchConfiguration('rgb_topic').perform(context)
    depth_topic = LaunchConfiguration('depth_topic').perform(context)
    capture_dir = LaunchConfiguration('capture_dir').perform(context)
    save_hz = LaunchConfiguration('save_hz').perform(context)
    fast_lio_config = LaunchConfiguration('fast_lio_config').perform(context)
    oak_name = LaunchConfiguration('oak_name').perform(context)
    rviz_cfg = LaunchConfiguration('rviz_config').perform(context)
    odom_source = LaunchConfiguration('odom_source').perform(context)

    actions = [
        LogInfo(
            msg=(
                '[real_bringup_mapping] Robot on ground (wheel/teleop odom) | '
                'LiDAR fixed on livox_link | FAST-LIO map optional (does not move RobotModel)'
            )
        )
    ]

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'real_sensors.launch.py')
            ),
            launch_arguments={
                'use_oak': 'true' if use_oak else 'false',
                'use_imu_filter': LaunchConfiguration('use_imu_filter'),
                'use_livox': 'true',
                # CustomMsg for FAST-LIO; PointCloud2 for RViz via converter
                'livox_xfer_format': '1',
                'use_laserscan': 'false',
                'use_robot_state_publisher': 'true',
                'use_rviz': 'false',
                'oak_name': oak_name,
                'camera_model': LaunchConfiguration('camera_model'),
                'parent_frame': LaunchConfiguration('parent_frame'),
                'oak_params_file': LaunchConfiguration('oak_params_file'),
                'livox_config_file': LaunchConfiguration('livox_config_file'),
                'livox_frame_id': 'livox_link',
            }.items(),
        )
    )

    # Wheel / teleop odometry — owns odom -> base_footprint (robot on ground)
    if use_wheel_odom:
        actions.append(
            LogInfo(
                msg=(
                    f'[real_bringup_mapping] Wheel/teleop odom source={odom_source} '
                    f'(/cmd_vel or /wheel_odom) -> /odom + TF odom->base_footprint'
                )
            )
        )
        actions.append(
            Node(
                package='enimia_black_description',
                executable='wheel_teleop_odom.py',
                name='wheel_teleop_odom',
                output='screen',
                parameters=[
                    {
                        'odom_frame': 'odom',
                        'base_frame': 'base_footprint',
                        'map_frame': 'map',
                        'cmd_vel_topic': '/cmd_vel',
                        'odom_topic': '/odom',
                        'odom_source': odom_source,
                        'wheel_odom_topic': '/wheel_odom',
                        'wheelbase': 1.704,
                        'publish_tf': True,
                        'publish_map_to_odom': True,
                        'rate_hz': 50.0,
                    }
                ],
            )
        )

    # LiDAR cloud in livox_link (fixed on robot) for RViz
    if use_livox_cloud and _pkg_share('livox_ros_driver2') is not None:
        actions.append(
            LogInfo(
                msg='[real_bringup_mapping] Livox CustomMsg -> /livox/points (frame livox_link)'
            )
        )
        actions.append(
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package='enimia_black_description',
                        executable='livox_custom_to_pointcloud2.py',
                        name='livox_custom_to_pointcloud2',
                        output='screen',
                        parameters=[
                            {
                                'input_topic': '/livox/lidar',
                                'output_topic': '/livox/points',
                                'frame_id': 'livox_link',
                            }
                        ],
                    )
                ],
            )
        )

    if use_oak:
        actions.append(
            Node(
                package='topic_tools',
                executable='relay',
                name='oak_depth_alias',
                arguments=[depth_topic, '/oak/depth/image_raw'],
                output='screen',
            )
        )

    if use_oak and use_camera_windows:
        actions.append(
            TimerAction(
                period=2.5,
                actions=[
                    Node(
                        package='image_view',
                        executable='image_view',
                        name='oak_rgb_view',
                        remappings=[('image', rgb_topic)],
                        parameters=[{'autosize': True}],
                        output='screen',
                    ),
                    Node(
                        package='image_view',
                        executable='image_view',
                        name='oak_depth_view',
                        remappings=[('image', depth_topic)],
                        parameters=[{'autosize': True}],
                        output='screen',
                    ),
                ],
            )
        )

    if use_oak and use_frame_capture:
        actions.append(
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package='enimia_black_description',
                        executable='sensor_frame_capture.py',
                        name='sensor_frame_capture',
                        output='screen',
                        parameters=[
                            {
                                'rgb_topic': rgb_topic,
                                'depth_topic': depth_topic,
                                'output_dir': capture_dir,
                                'save_hz': float(save_hz),
                                'save_rgb': True,
                                'save_depth': True,
                                'show_windows': False,
                                'jpeg_quality': 90,
                                'max_frames': 0,
                            }
                        ],
                    )
                ],
            )
        )

    # FAST-LIO for PCD map only — does NOT parent RobotModel
    if use_fast_lio:
        if _pkg_share('fast_lio') is None:
            actions.append(
                LogInfo(
                    msg='[real_bringup_mapping] ERROR: fast_lio missing — source ~/ros2_ws/install/setup.bash'
                )
            )
        elif _pkg_share('livox_ros_driver2') is None:
            actions.append(
                LogInfo(
                    msg='[real_bringup_mapping] ERROR: livox_ros_driver2 missing — source ~/ws_livox/install/setup.bash'
                )
            )
        else:
            actions.append(
                LogInfo(
                    msg=(
                        '[real_bringup_mapping] FAST-LIO mapping only '
                        f'({fast_lio_config}) — RobotModel follows wheel/teleop odom, not LiDAR body'
                    )
                )
            )
            actions.append(
                TimerAction(
                    period=4.0,
                    actions=[
                        Node(
                            package='fast_lio',
                            executable='fastlio_mapping',
                            name='laserMapping',
                            output='screen',
                            parameters=[fast_lio_config, {'use_sim_time': False}],
                        )
                    ],
                )
            )

    if use_teleop:
        actions.append(
            ExecuteProcess(
                cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
                output='screen',
            )
        )

    if use_rviz:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg],
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    pkg = get_package_share_directory('enimia_black_description')
    cfg_dir = os.path.join(pkg, 'config', 'real_sensors')
    workspace_root = os.environ.get(
        'ENIMIA_BLACK_WORKSPACE',
        '/home/rabeb/Downloads/enimia_black',
    )
    captures_dir = os.path.join(workspace_root, 'captures')
    default_rviz = os.path.join(pkg, 'rviz', 'real_bringup_mapping.rviz')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_oak', default_value='true'),
            DeclareLaunchArgument('use_imu_filter', default_value='false'),
            DeclareLaunchArgument('use_camera_windows', default_value='true'),
            DeclareLaunchArgument('use_frame_capture', default_value='true'),
            DeclareLaunchArgument(
                'use_fast_lio',
                default_value='true',
                description='Build PCD map with FAST-LIO (does not move RobotModel)',
            ),
            DeclareLaunchArgument(
                'use_wheel_odom',
                default_value='true',
                description='Publish /odom from teleop cmd_vel or wheel encoders',
            ),
            DeclareLaunchArgument(
                'odom_source',
                default_value='cmd_vel',
                description='cmd_vel (teleop integrate) or wheel (/wheel_odom encoders)',
            ),
            DeclareLaunchArgument(
                'use_livox_cloud',
                default_value='true',
                description='Publish /livox/points in livox_link for RViz on the robot',
            ),
            DeclareLaunchArgument(
                'use_teleop',
                default_value='false',
                description='Start teleop_twist_keyboard on /cmd_vel',
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('oak_name', default_value='oak'),
            DeclareLaunchArgument('camera_model', default_value='OAK-D-LITE'),
            DeclareLaunchArgument('parent_frame', default_value='oak_link'),
            DeclareLaunchArgument(
                'oak_params_file',
                default_value=os.path.join(cfg_dir, 'oak_d_lite.yaml'),
            ),
            DeclareLaunchArgument(
                'livox_config_file',
                default_value=os.path.join(cfg_dir, 'MID360_config.json'),
            ),
            DeclareLaunchArgument('rgb_topic', default_value='/oak/rgb/image_raw'),
            DeclareLaunchArgument('depth_topic', default_value='/oak/stereo/image_raw'),
            DeclareLaunchArgument('capture_dir', default_value=captures_dir),
            DeclareLaunchArgument('save_hz', default_value='2.0'),
            DeclareLaunchArgument(
                'fast_lio_config',
                default_value=os.path.join(cfg_dir, 'fast_lio_mid360.yaml'),
            ),
            DeclareLaunchArgument('rviz_config', default_value=default_rviz),
            OpaqueFunction(function=_setup),
        ]
    )
