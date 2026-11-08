cmake_minimum_required(VERSION 3.8)
project(enimia_black_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz config world models
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/gpx_to_waypoints_csv.py
  scripts/nav2_lifecycle_autostarter.py
  scripts/nav2_waypoint_goal_sender.py
  scripts/stop_sign_detector.py
  scripts/stop_sign_node.py
  scripts/stop_sign_clock.py
  scripts/yellow_stop_line_detector.py
  scripts/lane_keeper.py
  scripts/robust_lane_detector.py
  scripts/lane_keeping_controller.py
  scripts/cmd_vel_safety_gate.py
  scripts/stanley_controller.py
  scripts/steering_sync.py
  scripts/render_urdf.py
  scripts/tricycle_ackermann_controller.py
  scripts/amcl_initial_pose_publisher.py
  scripts/amcl_localization_guard.py
  scripts/rviz_tf_gate.py
  scripts/gazebo_pose_bridge.py
  scripts/sensor_frame_capture.py
  scripts/fastlio_odom_bridge.py
  scripts/wheel_teleop_odom.py
  scripts/livox_custom_to_pointcloud2.py
  scripts/component_container_depthai.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
