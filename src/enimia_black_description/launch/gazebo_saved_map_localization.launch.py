import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('enimia_black_description')
    workspace_root = os.environ.get(
        'ENIMIA_BLACK_WORKSPACE',
        '/home/rabeb/Downloads/enimia_black',
    )
    maps_dir = os.path.join(workspace_root, 'maps')

    default_world = os.path.join(
        pkg,
        'world',
        'sem_circuit_2026_realistic_new.world',
    )
    default_map = os.path.join(maps_dir, 'silesia_ring_map.yaml')
    default_waypoints = os.path.join(maps_dir, 'recorded_waypoints.csv')
    default_rviz_config = os.path.join(
        pkg,
        'rviz',
        'mapping_slam_waypoints.rviz',
    )
    default_amcl_params = os.path.join(
        pkg,
        'config',
        'amcl_saved_map.yaml',
    )
    default_calculation_config = os.path.join(
        workspace_root,
        'src',
        'waypoints_calculations',
        'config',
        'waypointsConfig.yaml',
    )
    gazebo_launch_file = os.path.join(
        pkg,
        'launch',
        'gazebo_classic_sem_circuit.launch.py',
    )

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    localization_rviz = LaunchConfiguration('localization_rviz')
    teleop = LaunchConfiguration('teleop')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_timeout = LaunchConfiguration('spawn_timeout')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    map_yaml = LaunchConfiguration('map')
    rviz_config = LaunchConfiguration('rviz_config')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    load_waypoints = LaunchConfiguration('load_waypoints')
    waypoints_file = LaunchConfiguration('waypoints_file')
    record_waypoints = LaunchConfiguration('record_waypoints')
    waypoints_output = LaunchConfiguration('waypoints_output')
    waypoints_interval = LaunchConfiguration('waypoints_interval')
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

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'set_initial_pose': True,
                'initial_pose.x': ParameterValue(x, value_type=float),
                'initial_pose.y': ParameterValue(y, value_type=float),
                'initial_pose.yaw': ParameterValue(yaw, value_type=float),
            },
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        output='screen',
        condition=IfCondition(localization_rviz),
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
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('localization_rviz', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('spawn_timeout', default_value='120.0'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        DeclareLaunchArgument('amcl_params_file', default_value=default_amcl_params),
        DeclareLaunchArgument('load_waypoints', default_value='true'),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument('record_waypoints', default_value='false'),
        DeclareLaunchArgument(
            'waypoints_output',
            default_value=os.path.join(maps_dir, 'recorded_waypoints_new.csv'),
        ),
        DeclareLaunchArgument('waypoints_interval', default_value='1.0'),
        DeclareLaunchArgument('follow_waypoints', default_value='false'),
        DeclareLaunchArgument(
            'calculation_params_file',
            default_value=default_calculation_config,
        ),
        gazebo,
        map_server,
        amcl,
        lifecycle_manager,
        waypoints_loader,
        waypoints_creator,
        waypoint_follower,
        rviz_node,
    ])
