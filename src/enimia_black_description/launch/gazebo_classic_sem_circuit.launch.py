import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('enimia_black_description')
    pkg_prefix = get_package_prefix('enimia_black_description')

    default_world = os.path.join(
        pkg,
        'world',
        'new_circuit.world'
    )
    default_xacro_file = os.path.join(pkg, 'urdf', 'enimia_black_sensors.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'display_sensors.rviz')

    world = LaunchConfiguration('world')
    xacro_file = LaunchConfiguration('xacro_file')
    urdf_renderer = os.path.join(
        pkg_prefix,
        'lib',
        'enimia_black_description',
        'render_urdf.py',
    )
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    rviz = LaunchConfiguration('rviz')
    teleop = LaunchConfiguration('teleop')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    gazebo_master_uri = LaunchConfiguration('gazebo_master_uri')
    spawn_timeout = LaunchConfiguration('spawn_timeout')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    package_model_path = os.path.dirname(pkg)

    gazebo_env = {
        'GAZEBO_RESOURCE_PATH': '/usr/share/gazebo-11',
        'GAZEBO_MODEL_PATH': (
            f'{package_model_path}:'
            '/usr/share/gazebo-11/models'
        ),
        'GAZEBO_PLUGIN_PATH': (
            '/opt/ros/humble/lib:'
            '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'
        ),
        'OGRE_RESOURCE_PATH': '/usr/lib/x86_64-linux-gnu/OGRE-1.9.0',
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'QT_QPA_PLATFORM': 'xcb',
        'GAZEBO_MASTER_URI': gazebo_master_uri,
    }

    robot_description = ParameterValue(
        Command([
            'python3 ',
            urdf_renderer,
            ' ',
            xacro_file,
            ' publish_odom_tf:=',
            publish_odom_tf,
        ]),
        value_type=str,
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            world,
            PythonExpression([
                '"--verbose" if "true" == "', verbose, '" else ""'
            ]),
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_force_system.so',
            '-s', 'libgazebo_ros_state.so',
        ],
        output='screen',
        additional_env=gazebo_env,
    )

    gazebo_client = ExecuteProcess(
        cmd=[
            'gzclient',
            '--gui-client-plugin=libgazebo_ros_eol_gui.so',
            PythonExpression([
                '"--verbose" if "true" == "', verbose, '" else ""'
            ]),
        ],
        output='screen',
        additional_env=gazebo_env,
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'enimia_black',
            '-timeout', spawn_timeout,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
        output='screen',
    )

    # ros2_control publishes the measured joint states from Gazebo.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # The actual ENIMIA geometry is a three-wheel vehicle: two front steering
    # joints and one rear drive joint.  The stock Ackermann controller requires
    # two driven rear wheels, so use independent interface controllers plus the
    # tricycle_ackermann_controller node below for the kinematics.
    front_steering_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['front_steering_controller'],
        output='screen',
    )

    rear_wheel_velocity_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rear_wheel_velocity_controller'],
        output='screen',
    )

    tricycle_ackermann_controller = Node(
        package='enimia_black_description',
        executable='tricycle_ackermann_controller.py',
        name='tricycle_ackermann_controller',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'wheelbase': 1.704,
            'front_track': 0.567,
            'rear_wheel_radius': 0.18,
            'max_steering_angle': 0.52,
            'publish_odom_tf': ParameterValue(publish_odom_tf, value_type=bool),
        }],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz),
    )

    teleop_node = ExecuteProcess(
        cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(teleop),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo Classic world file to load',
        ),
        DeclareLaunchArgument(
            'xacro_file',
            default_value=default_xacro_file,
            description='Robot Xacro file; set enimia_black.urdf to test the base vehicle without sensors',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch gzclient',
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='true',
            description='Print verbose Gazebo output',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 with the sensor display configuration',
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Launch keyboard teleoperation on /cmd_vel',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='true',
            description='Let the tricycle Ackermann controller publish odom -> base_footprint TF',
        ),
        DeclareLaunchArgument(
            'gazebo_master_uri',
            default_value='http://127.0.0.1:11345',
            description='Gazebo master URI. Change the port if another Gazebo is still running.',
        ),
        DeclareLaunchArgument(
            'spawn_timeout',
            # The realistic SEM circuit can take more than two minutes to load
            # on a software-rendered Gazebo Classic setup.  ros2_control only
            # exists after the robot is inserted, so avoid aborting the spawn
            # just before controller_manager becomes available.
            default_value='300.0',
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
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=6.0, actions=[front_steering_spawner]),
        TimerAction(period=7.0, actions=[rear_wheel_velocity_spawner]),
        tricycle_ackermann_controller,
        rviz_node,
        teleop_node,
    ])
