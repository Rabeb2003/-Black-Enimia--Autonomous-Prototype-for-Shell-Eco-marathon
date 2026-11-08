import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # Package path
    enimia_pkg = get_package_share_directory('enimia_black_description')
    
    # Chemin vers le fichier xacro avec capteurs
    xacro_file = os.path.join(enimia_pkg, 'urdf', 'enimia_black_sensors.xacro')
    
    # Conversion xacro -> URDF
    from xacro import process_file
    robot_description = process_file(xacro_file).toxml()

    # World Gazebo avec circuit
    world_file = os.path.join(enimia_pkg, 'world', 'empty.world')
    
    # Gazebo server
    gazebo_server = Node(
        package='gazebo_ros',
        executable='gzserver',
        arguments=['-s', 'libgazebo_ros_init.so', 
                   '-s', 'libgazebo_ros_factory.so',
                   world_file],
        output='screen'
    )

    # Gazebo client (optionnel, pour visualisation)
    gazebo_client = Node(
        package='gazebo_ros',
        executable='gzclient',
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

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    return LaunchDescription([
        # Lancer Gazebo server
        gazebo_server,
        
        # Attendre que Gazebo soit prêt avant de lancer le client
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
