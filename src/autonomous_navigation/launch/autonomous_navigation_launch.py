import os
import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the package share directory
    package_share_directory = get_package_share_directory('lane_detection')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='lane_detection',                    
            executable='stop_sign_node',
            name='stop_sign_detector',
            output='screen',
            parameters=[{'package_share_directory': package_share_directory}]
        ),
        launch_ros.actions.Node(
            package='lane_detection',  
            executable='lane_detection_node',
            name='lane_detector',
            output='screen',
            parameters=[{'package_share_directory': package_share_directory}]
        ),

    ])
