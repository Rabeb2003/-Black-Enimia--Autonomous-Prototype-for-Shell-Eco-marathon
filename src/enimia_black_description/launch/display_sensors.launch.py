import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('enimia_black_description')
    
    # Chemin vers le fichier xacro avec capteurs
    xacro_file = os.path.join(pkg, 'urdf', 'enimia_black_sensors.xacro')
    
    # Conversion xacro -> URDF
    from xacro import process_file
    robot_description = process_file(xacro_file).toxml()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        
        # RViz2 avec configuration des capteurs
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'display_sensors.rviz')],
            output='screen'
        ),
    ])
