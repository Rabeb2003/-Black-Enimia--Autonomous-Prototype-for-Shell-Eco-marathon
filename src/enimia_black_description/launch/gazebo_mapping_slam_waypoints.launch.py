import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _is_true(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _save_map_after_delay(context, *args, **kwargs):
    save_map = LaunchConfiguration('save_map').perform(context)
    if not _is_true(save_map):
        return []

    map_output = LaunchConfiguration('map_output').perform(context)
    save_delay = LaunchConfiguration('save_delay').perform(context)

    try:
        delay = float(save_delay)
    except ValueError:
        delay = 120.0

    output_dir = os.path.dirname(map_output)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    return [
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
    ]


def generate_launch_description():
    pkg = get_package_share_directory('enimia_black_description')
    workspace_root = os.environ.get(
        'ENIMIA_BLACK_WORKSPACE',
        '/home/rabeb/Downloads/enimia_black',
    )
    maps_dir = os.path.join(workspace_root, 'maps')
    os.makedirs(maps_dir, exist_ok=True)

    default_world = os.path.join(
        pkg,
        'world',
        'sem_circuit_2026_realistic_new.world',
    )
    gazebo_launch_file = os.path.join(
        pkg,
        'launch',
        'gazebo_classic_sem_circuit.launch.py',
    )
    slam_params_default = os.path.join(
        pkg,
        'config',
        'slam_toolbox_mapping.yaml',
    )
    rviz_config_default = os.path.join(
        pkg,
        'rviz',
        'mapping_slam_waypoints.rviz',
    )
    default_loaded_waypoints = os.path.join(
        pkg,
        'config',
        'silesia_ring_gpx_local_waypoints.csv',
    )
    default_waypoints_output = os.path.join(
        maps_dir,
        'recorded_waypoints.csv',
    )
    default_map_output = os.path.join(
        maps_dir,
        'silesia_ring_map',
    )
    default_calculation_config = os.path.join(
        workspace_root,
        'src',
        'waypoints_calculations',
        'config',
        'waypointsConfig.yaml',
    )

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    mapping_rviz = LaunchConfiguration('mapping_rviz')
    teleop = LaunchConfiguration('teleop')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_timeout = LaunchConfiguration('spawn_timeout')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    record_waypoints = LaunchConfiguration('record_waypoints')
    waypoints_interval = LaunchConfiguration('waypoints_interval')
    waypoints_output = LaunchConfiguration('waypoints_output')
    load_waypoints = LaunchConfiguration('load_waypoints')
    waypoints_file = LaunchConfiguration('waypoints_file')
    follow_waypoints = LaunchConfiguration('follow_waypoints')
    calculation_params_file = LaunchConfiguration('calculation_params_file')

    load_or_follow = IfCondition(PythonExpression([
        '"', load_waypoints, '".lower() in ["true", "1", "yes", "on"] or ',
        '"', follow_waypoints, '".lower() in ["true", "1", "yes", "on"]',
    ]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': verbose,
            'rviz': 'false',
            'teleop': teleop,
            'use_sim_time': use_sim_time,
            'spawn_timeout': spawn_timeout,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items(),
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        output='screen',
        condition=IfCondition(mapping_rviz),
    )

    waypoints_creator = Node(
        package='waypoints_niagara_creator',
        executable='waypoints_creator_node',
        name='waypoints_creator_node',
        parameters=[{
            'file_path': waypoints_output,
            'interval': ParameterValue(waypoints_interval, value_type=float),
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        output='screen',
        condition=IfCondition(record_waypoints),
    )

    waypoints_loader = Node(
        package='waypoints_niagara_loader',
        executable='waypoints_loader_node',
        name='waypoints_loader_node',
        parameters=[{
            'file_path': waypoints_file,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        output='screen',
        condition=load_or_follow,
    )

    waypoint_follower = Node(
        package='waypoints_calculations',
        executable='waypoints_calculations',
        name='waypoints_calculations',
        parameters=[
            calculation_params_file,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        output='screen',
        condition=IfCondition(follow_waypoints),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo Classic world file to load',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Launch gzclient. Disabled by default for mapping stability',
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='true',
            description='Print verbose Gazebo output',
        ),
        DeclareLaunchArgument(
            'mapping_rviz',
            default_value='true',
            description='Launch RViz2 mapping view',
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Launch keyboard teleoperation on /cmd_vel. Prefer a separate terminal',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'spawn_timeout',
            default_value='120.0',
            description='Seconds to wait for the Gazebo spawn service',
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Initial robot x position',
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Initial robot y position',
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.02',
            description='Initial robot z position',
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial robot yaw angle',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_default,
            description='SLAM Toolbox parameter file',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_default,
            description='RViz2 mapping configuration',
        ),
        DeclareLaunchArgument(
            'record_waypoints',
            default_value='true',
            description='Record /odom waypoints while driving',
        ),
        DeclareLaunchArgument(
            'waypoints_interval',
            default_value='1.0',
            description='Minimum distance in meters between recorded waypoints',
        ),
        DeclareLaunchArgument(
            'waypoints_output',
            default_value=default_waypoints_output,
            description='CSV written by waypoints_niagara_creator',
        ),
        DeclareLaunchArgument(
            'load_waypoints',
            default_value='false',
            description='Load and display waypoints_file',
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_loaded_waypoints,
            description='CSV loaded by waypoints_niagara_loader',
        ),
        DeclareLaunchArgument(
            'follow_waypoints',
            default_value='false',
            description='Run waypoint follower; disable teleop when using this',
        ),
        DeclareLaunchArgument(
            'calculation_params_file',
            default_value=default_calculation_config,
            description='Parameters for waypoints_calculations',
        ),
        DeclareLaunchArgument(
            'save_map',
            default_value='false',
            description='Save /map once after save_delay seconds',
        ),
        DeclareLaunchArgument(
            'save_delay',
            default_value='120.0',
            description='Delay before auto-saving map when save_map is true',
        ),
        DeclareLaunchArgument(
            'map_output',
            default_value=default_map_output,
            description='Map output prefix for nav2_map_server map_saver_cli',
        ),
        gazebo,
        slam_toolbox,
        waypoints_creator,
        waypoints_loader,
        waypoint_follower,
        rviz_node,
        OpaqueFunction(function=_save_map_after_delay),
    ])
