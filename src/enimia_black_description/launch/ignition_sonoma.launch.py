import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('enimia_black_description')
    
    # Chemin vers le fichier xacro avec capteurs
    xacro_file = os.path.join(pkg, 'urdf', 'enimia_black_sensors.xacro')
    
    # Conversion xacro -> URDF
    from xacro import process_file
    robot_description = process_file(xacro_file).toxml()

    # Chemin vers le fichier world Gazebo Classic
    world_file = os.path.join(pkg, 'world', 'sem_circuit_2026_realistic_new.world')

    # Gazebo Classic avec monde personnalisé via gazebo_ros package
    gazebo = ExecuteProcess(
        cmd=['gazebo', world_file, '--verbose'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # Spawn du robot dans Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'enimia_black'],
        output='screen'
    )

    # RViz2
    rviz_config = os.path.join(pkg, 'rviz', 'display_sensors.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Nav2 Configuration
    nav2_params_file = os.path.join(pkg, 'config', 'nav2_autonomous_params.yaml')
    nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={},
        convert_types=True
    )

    # Lifecycle nodes
    lifecycle_nodes = ['map_server', 'amcl', 'controller_server',
                      'planner_server', 'behavior_server', 'bt_navigator',
                      'waypoint_follower', 'velocity_smoother']

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[nav2_params],
        output='screen'
    )

    # AMCL
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        parameters=[nav2_params],
        output='screen'
    )

    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        parameters=[nav2_params],
        output='screen'
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        parameters=[nav2_params],
        output='screen'
    )

    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        parameters=[nav2_params],
        output='screen'
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        parameters=[nav2_params],
        output='screen'
    )

    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        parameters=[nav2_params],
        output='screen'
    )

    # Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        parameters=[nav2_params],
        output='screen'
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}],
        output='screen'
    )

    return LaunchDescription([
        # Gazebo Classic
        gazebo,
        
        # Robot State Publisher
        robot_state_publisher,
        
        # Joint State Publisher
        joint_state_publisher,
        
        # Attendre que Gazebo soit prêt avant de spawner le robot
        TimerAction(period=5.0, actions=[spawn_robot]),
        
        # RViz2
        rviz_node,
        
        # Nav2 Lifecycle Manager
        lifecycle_manager,
        
        # Nav2 Nodes
        map_server,
        amcl_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
    ])
