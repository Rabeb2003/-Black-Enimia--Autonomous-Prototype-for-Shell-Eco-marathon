import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    enimia_pkg = get_package_share_directory('enimia_black_description')
    oak_pkg = get_package_share_directory('oak_d_camera')
    
    # Chemin vers le fichier xacro avec capteurs
    xacro_file = os.path.join(enimia_pkg, 'urdf', 'enimia_black_sensors.xacro')
    
    # Conversion xacro -> URDF
    from xacro import process_file
    robot_description = process_file(xacro_file).toxml()

    # World Gazebo
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

    # Gazebo client
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

    # OAK-D Camera launch (pour la vraie caméra physique)
    # Note: Pour la simulation Gazebo, les plugins Gazebo dans oak_d_lite.xacro
    # simulent déjà la caméra. Ce launch est pour utiliser la vraie caméra OAK-D.
    oak_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(oak_pkg, 'launch', 'camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'OAK-D-LITE',
            'tf_prefix': 'oak',
            'base_frame': 'oak_rgb_optical_frame',
            'parent_frame': 'oak_link',
            'cam_pos_x': '0.0',
            'cam_pos_y': '0.0',
            'cam_pos_z': '0.0',
            'cam_roll': '0.0',
            'cam_pitch': '0.0',
            'cam_yaw': '0.0',
            'use_rviz': 'false',  # RViz lancé séparément si besoin
            'use_depth': 'true',
            'use_pointcloud': 'true',
            'pc_color': 'true',
            'use_base': 'false'
        }.items()
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
        
        # OAK-D Camera (commenté pour simulation pure, décommenter pour vraie caméra)
        # oak_camera_launch,
    ])
