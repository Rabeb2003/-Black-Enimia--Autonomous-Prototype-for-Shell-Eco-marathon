import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
        'new_circuit.world',
    )
    default_map = os.path.join(maps_dir, 'silesia_ring_map.yaml')
    default_rviz_map = os.path.join(maps_dir, 'silesia_ring_rviz.yaml')
    default_waypoints = os.path.join(maps_dir, 'recorded_waypoints.csv')
    default_rviz_config = os.path.join(
        pkg,
        'rviz',
        'nav2_autonomous.rviz',
    )
    default_nav2_params = os.path.join(pkg, 'config', 'nav2_autonomous_params.yaml')
    default_ekf_params = os.path.join(pkg, 'config', 'ekf_autonomous.yaml')
    default_nav_to_pose_bt = os.path.join(
        pkg, 'config', 'navigate_to_pose_ackermann.xml')
    default_nav_through_poses_bt = os.path.join(
        pkg, 'config', 'navigate_through_poses_ackermann.xml')
    gazebo_launch_file = os.path.join(
        pkg,
        'launch',
        'gazebo_classic_sem_circuit.launch.py',
    )

    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    rviz_map_yaml = LaunchConfiguration('rviz_map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    rviz = LaunchConfiguration('rviz')
    teleop = LaunchConfiguration('teleop')
    spawn_timeout = LaunchConfiguration('spawn_timeout')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    load_waypoints = LaunchConfiguration('load_waypoints')
    auto_start_waypoints = LaunchConfiguration('auto_start_waypoints')
    record_waypoints = LaunchConfiguration('record_waypoints')
    waypoints_file = LaunchConfiguration('waypoints_file')
    waypoints_output = LaunchConfiguration('waypoints_output')
    waypoints_interval = LaunchConfiguration('waypoints_interval')
    perception = LaunchConfiguration('perception')
    stop_line_detection = LaunchConfiguration('stop_line_detection')
    stop_sign_detection = LaunchConfiguration('stop_sign_detection')
    lane_detection = LaunchConfiguration('lane_detection')
    stop_max_fps = LaunchConfiguration('stop_max_fps')
    lane_max_fps = LaunchConfiguration('lane_max_fps')
    gazebo_master_uri = LaunchConfiguration('gazebo_master_uri')
    waypoint_start_delay = LaunchConfiguration('waypoint_start_delay')
    use_amcl = LaunchConfiguration('use_amcl')

    start_stop_line_detection = IfCondition(PythonExpression([
        '"', perception, '".lower() in ["true", "1", "yes", "on"] and ',
        '"', stop_line_detection, '".lower() in ["true", "1", "yes", "on"]',
    ]))
    start_stop_sign_detection = IfCondition(PythonExpression([
        '"', perception, '".lower() in ["true", "1", "yes", "on"] and ',
        '"', stop_sign_detection, '".lower() in ["true", "1", "yes", "on"]',
    ]))
    start_lane_detection = IfCondition(PythonExpression([
        '"', perception, '".lower() in ["true", "1", "yes", "on"] and ',
        '"', lane_detection, '".lower() in ["true", "1", "yes", "on"]',
    ]))
    start_amcl = IfCondition(PythonExpression([
        '"', use_amcl, '".lower() in ["true", "1", "yes", "on"]'
    ]))
    start_gazebo_pose_bridge = IfCondition(PythonExpression([
        '"', use_amcl, '".lower() not in ["true", "1", "yes", "on"]'
    ]))

    gazebo = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_file),
                launch_arguments={
                    'world': world,
                    'gui': gui,
                    'verbose': verbose,
                    'rviz': 'false',
                    'teleop': teleop,
                    'use_sim_time': use_sim_time,
                    # robot_localization is the only publisher of odom ->
                    # base_footprint in this launch.  The tricycle bridge
                    # still publishes raw wheel odometry on /odom.
                    'publish_odom_tf': 'false',
                    'gazebo_master_uri': gazebo_master_uri,
                    'spawn_timeout': spawn_timeout,
                    'x': x,
                    'y': y,
                    'z': z,
                    'yaw': yaw,
                }.items(),
            ),
        ],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {
                'yaml_filename': map_yaml,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    gazebo_pose_bridge = Node(
        package='enimia_black_description',
        executable='gazebo_pose_bridge.py',
        name='gazebo_pose_bridge',
        output='screen',
        parameters=[{
            'model_name': 'enimia_black',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_rate': 30.0,
            'initial_x': ParameterValue(x, value_type=float),
            'initial_y': ParameterValue(y, value_type=float),
            'initial_yaw': ParameterValue(yaw, value_type=float),
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=start_gazebo_pose_bridge,
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'set_initial_pose': True,
                'initial_pose.x': ParameterValue(x, value_type=float),
                'initial_pose.y': ParameterValue(y, value_type=float),
                'initial_pose.yaw': ParameterValue(yaw, value_type=float),
            },
        ],
        condition=start_amcl,
    )

    # Localization lifecycle manager (map_server, and amcl if enabled).
    # Specifically named 'lifecycle_manager_localization' so the Nav2 RViz plugin
    # discovers it and displays 'Localization: active'.
    localization_lifecycle_gt = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            # Do not configure the map before Gazebo publishes /clock. With
            # use_sim_time this can leave map_server indefinitely
            # "unconfigured" during a slow Gazebo Classic startup.
            'autostart': False,
            'node_names': ['map_server'],
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
        }],
        condition=start_gazebo_pose_bridge,
    )

    localization_lifecycle_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            # Started by localization_readiness_gate after /clock exists.
            'autostart': False,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
        }],
        condition=start_amcl,
    )

    # Start localization only after Gazebo produces simulated time.  Starting
    # map_server at time zero can leave it permanently unconfigured, which
    # removes both /map and the map -> odom transform from RViz.
    localization_readiness_gate = Node(
        package='enimia_black_description',
        executable='nav2_lifecycle_autostarter.py',
        name='localization_readiness_gate',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'lifecycle_manager': 'lifecycle_manager_localization',
            'required_topics': ['/clock'],
            'initial_delay': 2.0,
            'service_timeout': 360.0,
        }],
    )

    # AMCL may become active before Gazebo inserts the robot.  Republish the
    # configured pose after EKF odometry exists so AMCL always establishes the
    # map -> odom transform used by RViz and Nav2.
    amcl_initial_pose_publisher = Node(
        package='enimia_black_description',
        executable='amcl_initial_pose_publisher.py',
        name='amcl_initial_pose_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'x': ParameterValue(x, value_type=float),
            'y': ParameterValue(y, value_type=float),
            'yaw': ParameterValue(yaw, value_type=float),
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'retry_period': 2.0,
            'timeout': 360.0,
        }],
        condition=start_amcl,
    )

    # AMCL is still the normal localization source.  The guard only uses the
    # simulated P3D pose to re-seed AMCL after a large jump/stall and publishes
    # a stable pose for the small kR_in turn guide.
    amcl_localization_guard = Node(
        package='enimia_black_description',
        executable='amcl_localization_guard.py',
        name='amcl_localization_guard',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'ground_truth_topic': '/gazebo/ground_truth/odom',
            'amcl_pose_topic': '/amcl_pose',
            'guarded_pose_topic': '/localization/guarded_pose',
            'use_ground_truth_recovery': True,
            'position_error_threshold': 1.50,
            'yaw_error_threshold': 0.60,
            'amcl_timeout': 1.50,
            'startup_grace_period': 8.0,
            'recovery_cooldown': 3.0,
        }],
        condition=start_amcl,
    )

    # RViz receives a separate half-resolution map texture.  Nav2 continues
    # using ``map_yaml`` above at 0.05 m/cell.  This avoids a reproducible
    # RViz/OGRE crash seen when rendering the 4031 x 1939 source map on the
    # project workstation's graphics stack.
    rviz_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_rviz',
        output='screen',
        parameters=[{
            'yaml_filename': rviz_map_yaml,
            'topic_name': 'map_rviz',
            'frame_id': 'map',
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=IfCondition(rviz),
    )

    rviz_map_lifecycle = Node(
        package='enimia_black_description',
        executable='nav2_lifecycle_autostarter.py',
        name='rviz_map_lifecycle_gate',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'node_names': ['map_server_rviz'],
            'initial_delay': 1.0,
            'service_timeout': 120.0,
            'retry_delay': 2.0,
            'max_attempts': 5,
        }],
        condition=IfCondition(rviz),
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'default_nav_to_pose_bt_xml': default_nav_to_pose_bt,
                'default_nav_through_poses_bt_xml': default_nav_through_poses_bt,
            },
        ],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel_safe'),
        ],
    )

    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            # A separate readiness gate sends STARTUP only after Gazebo has
            # produced scan/odometry data and AMCL publishes map -> odom.
            'autostart': False,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
        }],
    )

    navigation_readiness_gate = Node(
        package='enimia_black_description',
        executable='nav2_lifecycle_autostarter.py',
        name='navigation_readiness_gate',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'lifecycle_manager': 'lifecycle_manager_navigation',
            'required_topics': ['/map', '/livox/scan', '/odometry/filtered'],
            'required_transforms': ['map->base_footprint'],
            # Gazebo Classic needs time to insert the sensor-rich robot and
            # the SEM world can be slow on a software-rendered workstation.
            'initial_delay': 2.0,
            'service_timeout': 360.0,
        }],
    )

    cmd_vel_safety_gate = Node(
        package='enimia_black_description',
        executable='cmd_vel_safety_gate.py',
        name='cmd_vel_safety_gate',
        output='screen',
        parameters=[{
            # Nav2 is first corrected by lane keeping, then this final gate
            # applies the yellow-line and STOP-sign braking envelopes before
            # /cmd_vel reaches the Ackermann bridge.
            'input_cmd_topic': '/cmd_vel_lane',
            'output_cmd_topic': '/cmd_vel',
            'stop_topic': '/stop_line/detected',
            'stop_distance_topic': '/stop_line/distance',
            'brake_active_topic': '/stop_line/braking',
            'stop_distance': 1.25,
            'max_brake_decel': 10.0,
            'stop_hold_time': 5.0,
            'stop_sign_topic': '/stop_sign/detected',
            'stop_sign_distance_topic': '/stop_sign/distance',
            'stop_sign_brake_active_topic': '/stop_sign/braking',
            'stop_sign_distance': 2.5,
            'localization_recovering_topic': '/localization/recovering',
            'pause_during_localization_recovery': True,
            'zero_publish_rate': 10.0,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    yellow_stop_line_detector = Node(
        package='enimia_black_description',
        executable='yellow_stop_line_detector.py',
        name='yellow_stop_line_detector',
        output='screen',
        parameters=[{
            'rgb_topic': '/oak/rgb/image_raw',
            'depth_topic': '/oak/depth/image_raw',
            'stop_topic': '/stop_line/detected',
            'distance_topic': '/stop_line/distance',
            'brake_active_topic': '/stop_line/braking',
            'output_image_topic': '/stop_line/output',
            'stop_distance': 1.25,
            'stop_duration': 5.0,
            'rearm_clear_duration': 1.0,
            'max_fps': ParameterValue(stop_max_fps, value_type=float),
            'publish_debug_image': True,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=start_stop_line_detection,
    )

    stop_sign_detector = Node(
        package='enimia_black_description',
        executable='stop_sign_node.py',
        name='stop_sign_detector',
        output='screen',
        parameters=[{
            'rgb_topic': '/oak/rgb/image_raw',
            'depth_topic': '/oak/depth/image_raw',
            'stop_topic': '/stop_sign/detected',
            'distance_topic': '/stop_sign/distance',
            'brake_active_topic': '/stop_sign/braking',
            'output_image_topic': '/stop_sign/output',
            'output_alias_topic': '/stip_sign_output',
            'model_file': 'best_stop.pt',
            # The braking envelope starts as soon as the sign is measured;
            # hard stop and the five-second countdown begin at 2.5 m.
            'stop_distance': 2.5,
            'stop_duration': 5.0,
            'rearm_distance': 4.0,
            'brake_detection_timeout': 0.5,
            'max_fps': 6.0,
            'publish_debug_images': True,
            'display': False,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=start_stop_sign_detection,
    )

    lane_detection_node = Node(
        package='enimia_black_description',
        executable='robust_lane_detector.py',
        name='lane_detection_node',
        output='screen',
        parameters=[{
            'image_topic': '/oak/rgb/image_raw',
            'output_image_topic': '/lane_detection/output',
            'error_topic': '/lane_detection/center_error',
            'heading_error_topic': '/lane_detection/heading_error',
            'valid_topic': '/lane_detection/valid',
            'max_fps': ParameterValue(lane_max_fps, value_type=float),
            'roi_top_ratio': 0.25,
            'roi_bottom_ratio': 0.90,
            'smoothing_alpha': 0.75,
        }],
        condition=start_lane_detection,
    )

    lane_keeping_controller = Node(
        package='enimia_black_description',
        executable='lane_keeping_controller.py',
        name='lane_keeping_controller',
        output='screen',
        parameters=[{
            'input_cmd_topic': '/cmd_vel_safe',
            'output_cmd_topic': '/cmd_vel_lane',
            'lane_error_topic': '/lane_detection/center_error',
            'heading_error_topic': '/lane_detection/heading_error',
            'lane_valid_topic': '/lane_detection/valid',
            'stop_topic': '/stop_line/detected',
            'stop_sign_topic': '/stop_sign/detected',
            # Never let a short AMCL reset make the world-specific first-turn
            # guide miss its entry.  Nav2 itself still consumes AMCL TF.
            'curve_pose_topic': '/localization/guarded_pose',
            'curve_fallback_pose_topic': '/odometry/filtered',
            'target_speed': 5.6,
            'max_speed': 6.0,
            'max_acceleration': 7.0,
            'max_deceleration': 10.0,
            'min_curve_speed': 3.8,
            # Mechanical 0.52 rad steering limit allows curvature up to 0.307 1/m.
            'max_curvature': 0.30,
            'kp': 1.15,
            'kd': 0.12,
            'kh': 0.85,
            'right_curve_gain': 1.70,
            'right_curve_bias': 0.055,
            'right_curve_threshold': 0.04,
            # Start only around road_R_jnt62, the surveyed physical entrance
            # to the first right bend.  This avoids premature slowdown.
            'kr_in_trigger_x': 85.3630,
            'kr_in_trigger_y': -5.5101,
            'kr_in_trigger_radius': 3.8,
            # kR_in is the inner kerb; this offset follows road centre.
            'kr_in_safety_offset': 2.90,
            'kr_in_lookahead': 2.4,
            'kr_in_speed': 4.2,
            'kr_in_heading_gain': 1.70,
            'kr_in_cross_track_gain': 1.25,
            'kr_in_pure_pursuit_weight': 0.45,
            'lane_timeout': 0.80,
            'command_timeout': 0.50,
            'publish_rate': 30.0,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=start_lane_detection,
    )

    waypoints_loader = Node(
        package='waypoints_niagara_loader',
        executable='waypoints_loader_node',
        name='waypoints_loader_node',
        output='screen',
        parameters=[{
            'file_path': waypoints_file,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=IfCondition(load_waypoints),
    )

    waypoint_goal_sender = Node(
        package='enimia_black_description',
        executable='nav2_waypoint_goal_sender.py',
        name='nav2_waypoint_goal_sender',
        output='screen',
        parameters=[{
            'waypoints_file': waypoints_file,
            'frame_id': 'map',
            'initial_delay': ParameterValue(waypoint_start_delay, value_type=float),
            'waypoint_stride': 5,
            'min_spacing': 1.5,
            'max_waypoints': 200,
            'required_active_nodes': ['controller_server', 'planner_server', 'bt_navigator'],
            'wait_for_active_timeout': 240.0,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=IfCondition(auto_start_waypoints),
    )

    waypoints_creator = Node(
        package='waypoints_niagara_creator',
        executable='waypoints_creator_node',
        name='waypoints_creator_node',
        output='screen',
        parameters=[{
            'file_path': waypoints_output,
            'interval': ParameterValue(waypoints_interval, value_type=float),
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=IfCondition(record_waypoints),
    )

    # Do not open RViz while the sensor-heavy Gazebo world is still loading.
    # In particular, AMCL owns map -> odom, the EKF owns
    # odom -> base_footprint, and robot_state_publisher owns the wheel TFs.
    # Waiting for each part avoids transient RViz errors such as
    # "Fixed Frame [map] does not exist".
    rviz_tf_gate = Node(
        package='enimia_black_description',
        executable='rviz_tf_gate.py',
        name='rviz_tf_gate',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'required_transforms': [
                'map->odom',
                'odom->base_footprint',
                'base_footprint->front_left_wheel_link',
                'base_footprint->front_right_wheel_link',
                'base_footprint->rear_wheel_link',
                'map->livox_link',
                'map->livox_optical_frame',
            ],
        }],
        condition=IfCondition(rviz),
    )

    # Start RViz as an explicit launch process rather than replacing the TF
    # gate process.  This makes its PID, exit code and OpenGL/Qt diagnostics
    # visible in the launch terminal.  The two environment variables match
    # the Gazebo Classic software-rendering workaround used on this machine.
    rviz_process = ExecuteProcess(
        cmd=[
            'rviz2', '-d', rviz_config,
            '--ros-args', '-p', 'use_sim_time:=true',
        ],
        output='screen',
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'QT_QPA_PLATFORM': 'xcb',
        },
    )

    def start_rviz_after_tf_gate(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(msg='TF ready: starting RViz2 with software OpenGL.'),
                rviz_process,
            ]
        return [
            LogInfo(msg=(
                'RViz2 was not started because the TF readiness gate exited '
                f'with code {event.returncode}.')),
        ]

    start_rviz_after_tf_gate_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_tf_gate,
            on_exit=start_rviz_after_tf_gate,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'rviz_map',
            default_value=default_rviz_map,
            description='Reduced-resolution map used only by RViz2',
        ),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params),
        DeclareLaunchArgument('ekf_params_file', default_value=default_ekf_params),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('spawn_timeout', default_value='300.0'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('load_waypoints', default_value='false'),
        DeclareLaunchArgument('auto_start_waypoints', default_value='false'),
        DeclareLaunchArgument('record_waypoints', default_value='false'),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument(
            'waypoints_output',
            default_value=os.path.join(maps_dir, 'recorded_waypoints_autonomous.csv'),
        ),
        DeclareLaunchArgument('waypoints_interval', default_value='1.0'),
        DeclareLaunchArgument(
            'perception', default_value='true',
            description='Start lane keeping and yellow stop-line perception',
        ),
        DeclareLaunchArgument(
            'stop_line_detection', default_value='true',
            description='Detect only the orange-yellow road stop line',
        ),
        DeclareLaunchArgument(
            'stop_sign_detection', default_value='true',
            description='Detect STOP panel, brake to 2.5 m, then hold for 5 seconds',
        ),
        DeclareLaunchArgument('lane_detection', default_value='true'),
        DeclareLaunchArgument('stop_max_fps', default_value='10.0'),
        DeclareLaunchArgument('lane_max_fps', default_value='15.0'),
        DeclareLaunchArgument('gazebo_master_uri', default_value='http://127.0.0.1:11345'),
        DeclareLaunchArgument('waypoint_start_delay', default_value='45.0'),
        DeclareLaunchArgument(
            'use_amcl',
            default_value='true',
            description='Use Livox scan matching (AMCL) to align the saved map and point cloud',
        ),
        gazebo,
        map_server,
        gazebo_pose_bridge,
        amcl,
        localization_lifecycle_gt,
        localization_lifecycle_amcl,
        localization_readiness_gate,
        amcl_initial_pose_publisher,
        amcl_localization_guard,
        rviz_map_server,
        rviz_map_lifecycle,
        ekf,
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        navigation_lifecycle,
        navigation_readiness_gate,
        cmd_vel_safety_gate,
        yellow_stop_line_detector,
        stop_sign_detector,
        lane_detection_node,
        lane_keeping_controller,
        waypoints_loader,
        waypoint_goal_sender,
        waypoints_creator,
        rviz_tf_gate,
        start_rviz_after_tf_gate_handler,
    ])
