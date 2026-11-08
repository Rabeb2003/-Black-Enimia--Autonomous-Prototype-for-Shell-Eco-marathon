"""Bringup matériel des capteurs ENIMIA Black (OAK-D Lite + IMU + Livox optionnel).

Packages attendus:
  - depthai_ros_driver, imu_filter_madgwick, pointcloud_to_laserscan (requis)
  - livox_ros_driver2 (optionnel) : si absent, OAK/IMU démarrent quand même avec
    un avertissement clair ; après installation + source du workspace Livox,
    le même launch active le Mid-360 sans modifier ce fichier.
"""

from __future__ import annotations

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xacro import process_file


def _pkg_share(name: str) -> str | None:
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        return None


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('enimia_black_description')
    cfg_dir = os.path.join(pkg, 'config', 'real_sensors')

    use_oak = LaunchConfiguration('use_oak').perform(context) == 'true'
    use_imu_filter = LaunchConfiguration('use_imu_filter').perform(context) == 'true'
    use_livox = LaunchConfiguration('use_livox').perform(context) == 'true'
    use_laserscan = LaunchConfiguration('use_laserscan').perform(context) == 'true'
    use_rsp = LaunchConfiguration('use_robot_state_publisher').perform(context) == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'

    oak_params = LaunchConfiguration('oak_params_file').perform(context)
    imu_params = LaunchConfiguration('imu_params_file').perform(context)
    laserscan_params = LaunchConfiguration('laserscan_params_file').perform(context)
    livox_config = LaunchConfiguration('livox_config_file').perform(context)
    oak_name = LaunchConfiguration('oak_name').perform(context)
    parent_frame = LaunchConfiguration('parent_frame').perform(context)
    try:
        livox_xfer_format = int(LaunchConfiguration('livox_xfer_format').perform(context))
    except ValueError:
        livox_xfer_format = 0
    livox_frame_id = LaunchConfiguration('livox_frame_id').perform(context)

    actions = []

    if use_rsp:
        xacro_file = os.path.join(pkg, 'urdf', 'enimia_black_sensors.xacro')
        robot_description = process_file(xacro_file).toxml()
        rsp_params = {'robot_description': robot_description, 'use_sim_time': False}
        actions.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[rsp_params],
            )
        )
        actions.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                parameters=[rsp_params],
            )
        )

    if use_oak:
        # Local launch: workspace depthai_ros_driver (source overlay; no apt shim)
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'oak_d_lite_real.launch.py')
                ),
                launch_arguments={
                    'name': oak_name,
                    'camera_model': LaunchConfiguration('camera_model'),
                    'params_file': oak_params,
                    'parent_frame': parent_frame,
                }.items(),
            )
        )
    else:
        actions.append(LogInfo(msg='[real_sensors] OAK disabled (use_oak:=false).'))

    if use_imu_filter:
        imu_raw = f'{oak_name}/imu/data'
        actions.append(
            LogInfo(
                msg=(
                    '[real_sensors] Starting imu_filter_madgwick '
                    f'(raw: /{imu_raw} -> /imu/data).'
                )
            )
        )
        actions.append(
            Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[imu_params],
                remappings=[
                    ('imu/data_raw', f'/{imu_raw}'),
                    ('imu/data', '/imu/data'),
                ],
            )
        )
    else:
        actions.append(
            LogInfo(msg='[real_sensors] IMU filter disabled (use_imu_filter:=false).')
        )

    livox_share = _pkg_share('livox_ros_driver2')
    if use_livox and livox_share is None:
        actions.append(
            LogInfo(
                msg=(
                    '[real_sensors] WARNING: livox_ros_driver2 is NOT installed. '
                    'Livox Mid-360 will be skipped; OAK/IMU continue. '
                    'Install & source livox_ros_driver2, then relaunch — '
                    'no code change required.'
                )
            )
        )
    elif use_livox and livox_share is not None:
        format_note = (
            'CustomMsg (FAST-LIO)'
            if livox_xfer_format == 1
            else 'PointCloud2 (laserscan/Nav2)'
        )
        actions.append(
            LogInfo(
                msg=(
                    '[real_sensors] livox_ros_driver2 found — starting Mid-360 '
                    f'(config: {livox_config}, xfer_format={livox_xfer_format}={format_note}).'
                )
            )
        )
        actions.append(
            Node(
                package='livox_ros_driver2',
                executable='livox_ros_driver2_node',
                name='livox_lidar_publisher',
                output='screen',
                parameters=[
                    {
                        # 0=PointCloud2, 1=CustomMsg (required by FAST-LIO lidar_type=1)
                        'xfer_format': livox_xfer_format,
                        'multi_topic': 0,
                        'data_src': 0,
                        'publish_freq': 10.0,
                        'output_data_type': 0,
                        'frame_id': livox_frame_id,
                        'user_config_path': livox_config,
                        'cmdline_input_bd_code': 'livox0000000001',
                    }
                ],
            )
        )
        if use_laserscan and livox_xfer_format == 0:
            actions.append(
                Node(
                    package='pointcloud_to_laserscan',
                    executable='pointcloud_to_laserscan_node',
                    name='pointcloud_to_laserscan',
                    output='screen',
                    parameters=[laserscan_params],
                    remappings=[
                        ('cloud_in', '/livox/lidar'),
                        ('scan', '/livox/scan'),
                    ],
                )
            )
        elif use_laserscan and livox_xfer_format != 0:
            actions.append(
                LogInfo(
                    msg=(
                        '[real_sensors] Skipping pointcloud_to_laserscan '
                        '(needs xfer_format:=0 PointCloud2; FAST-LIO uses CustomMsg).'
                    )
                )
            )
    else:
        actions.append(
            LogInfo(msg='[real_sensors] Livox disabled (use_livox:=false).')
        )

    if use_rviz:
        rviz_cfg = os.path.join(pkg, 'rviz', 'display_sensors.rviz')
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_oak',
                default_value='true',
                description='Start depthai_ros_driver (OAK-D Lite)',
            ),
            DeclareLaunchArgument(
                'use_imu_filter',
                default_value='true',
                description='Start imu_filter_madgwick on OAK IMU',
            ),
            DeclareLaunchArgument(
                'use_livox',
                default_value='true',
                description=(
                    'Start Livox Mid-360 if livox_ros_driver2 is available; '
                    'otherwise warn and continue with OAK/IMU only'
                ),
            ),
            DeclareLaunchArgument(
                'use_robot_state_publisher',
                default_value='true',
                description='Publish TF from enimia_black_sensors.xacro',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='false',
                description='Open RViz with display_sensors.rviz',
            ),
            DeclareLaunchArgument(
                'oak_name',
                default_value='oak',
                description='depthai camera node name / namespace prefix',
            ),
            DeclareLaunchArgument(
                'camera_model',
                default_value='OAK-D-LITE',
                description='OAK camera model for depthai_ros_driver',
            ),
            DeclareLaunchArgument(
                'parent_frame',
                default_value='oak_link',
                description='TF parent for the OAK base',
            ),
            DeclareLaunchArgument('cam_pos_x', default_value='0.0'),
            DeclareLaunchArgument('cam_pos_y', default_value='0.0'),
            DeclareLaunchArgument('cam_pos_z', default_value='0.0'),
            DeclareLaunchArgument('cam_roll', default_value='0.0'),
            DeclareLaunchArgument('cam_pitch', default_value='0.0'),
            DeclareLaunchArgument('cam_yaw', default_value='0.0'),
            DeclareLaunchArgument(
                'oak_pointcloud',
                default_value='false',
                description='Enable OAK depth point cloud (heavier CPU/USB)',
            ),
            DeclareLaunchArgument(
                'oak_params_file',
                default_value=os.path.join(cfg_dir, 'oak_d_lite.yaml'),
            ),
            DeclareLaunchArgument(
                'imu_params_file',
                default_value=os.path.join(cfg_dir, 'imu_filter_madgwick.yaml'),
            ),
            DeclareLaunchArgument(
                'laserscan_params_file',
                default_value=os.path.join(cfg_dir, 'pointcloud_to_laserscan.yaml'),
            ),
            DeclareLaunchArgument(
                'livox_config_file',
                default_value=os.path.join(cfg_dir, 'MID360_config.json'),
                description='Livox Mid-360 network JSON (host/lidar IPs)',
            ),
            DeclareLaunchArgument(
                'livox_xfer_format',
                default_value='0',
                description='0=PointCloud2 (Nav2/laserscan), 1=CustomMsg (FAST-LIO)',
            ),
            DeclareLaunchArgument(
                'livox_frame_id',
                default_value='livox_link',
                description='frame_id published by livox_ros_driver2',
            ),
            DeclareLaunchArgument(
                'use_laserscan',
                default_value='true',
                description='Start pointcloud_to_laserscan when xfer_format=0',
            ),
            OpaqueFunction(function=_setup),
        ]
    )
