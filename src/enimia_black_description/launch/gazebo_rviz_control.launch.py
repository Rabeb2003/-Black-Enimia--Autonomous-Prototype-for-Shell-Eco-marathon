import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('enimia_black_description')
    
    # Chemin vers le fichier xacro avec capteurs
    xacro_file = os.path.join(pkg, 'urdf', 'enimia_black_sensors.xacro')
    
    # Conversion xacro -> URDF
    from xacro import process_file
    robot_description = process_file(xacro_file).toxml()

    # Ignition Gazebo (Fortress)
    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4'],
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

    # Spawn du robot dans Ignition
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-name', 'enimia_black', 
                   '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.21',
                   '-Y', '0'],
        output='screen'
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                   os.path.join(pkg, 'config', 'controllers.yaml')],
        output='screen'
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Ackermann Steering Controller Spawner
    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
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

    return LaunchDescription([
        # Ignition Gazebo
        ignition_gazebo,
        
        # Robot State Publisher
        robot_state_publisher,
        
        # Joint State Publisher
        joint_state_publisher,
        
        # Attendre que Ignition soit prêt avant de spawner le robot
        TimerAction(period=5.0, actions=[spawn_robot]),
        
        # Controller Manager
        controller_manager,
        
        # Attendre que le controller manager soit prêt
        TimerAction(period=3.0, actions=[
            joint_state_broadcaster_spawner,
            ackermann_controller_spawner
        ]),
        
        # RViz2
        rviz_node,
    ])
