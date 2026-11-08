import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('enimia_black_description')
    
    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'world', 'sem_circuit_2026_realistic_new.world'),
        description='Path to Gazebo world file'
    )
    
    # Chemin vers le fichier xacro avec capteurs
    xacro_file = os.path.join(pkg, 'urdf', 'enimia_black_sensors.xacro')
    
    # Conversion xacro -> URDF
    from xacro import process_file
    robot_description = process_file(xacro_file).toxml()

    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world')],
        output='screen'
    )

    # Gazebo client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn du robot dans Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'enimia_black'],
        output='screen'
    )

    # Joint State Publisher (pour les joints non contrôlés)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    return LaunchDescription([
        world_arg,
        
        # Lancer Gazebo server
        gazebo_server,
        
        # Attendre que Gazebo soit prêt
        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_server,
                on_exit=[gazebo_client]
            )
        ),
        
        # Robot State Publisher
        robot_state_publisher,
        
        # Joint State Publisher
        joint_state_publisher,
        
        # Spawn du robot
        spawn_robot,
    ])
