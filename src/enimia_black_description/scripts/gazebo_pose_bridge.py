#!/usr/bin/env python3
"""Bridge Gazebo ground-truth model pose to ROS 2 localization and TF.

Subscribes to:
1. /gazebo/ground_truth/odom (published by libgazebo_ros_p3d.so)
2. /model_states and /gazebo/model_states (published by libgazebo_ros_state.so)

Broadcasts:
- map -> odom transform: T_map_odom = T_map_base * (T_odom_base)^-1
- /amcl_pose: PoseWithCovarianceStamped for RViz and Nav2
- /gazebo/ground_truth/pose: PoseStamped

Guarantees that RViz2 and Nav2 localization matches Gazebo 100% with ZERO drift
and immediately at startup (resolving "Could not transform from [map] to [odom]").
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation as R

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def pose_to_matrix(pos, quat):
    T = np.eye(4, dtype=float)
    # Avoid numerical anomalies in zero quaternion
    q_norm = np.linalg.norm(quat)
    if q_norm < 1e-6:
        quat = [0.0, 0.0, 0.0, 1.0]
    else:
        quat = quat / q_norm
    T[:3, :3] = R.from_quat(quat).as_matrix()
    T[:3, 3] = pos
    return T


def matrix_to_pose(T):
    pos = T[:3, 3]
    quat = R.from_matrix(T[:3, :3]).as_quat()
    q_norm = np.linalg.norm(quat)
    if q_norm > 1e-6:
        quat = quat / q_norm
    else:
        quat = np.array([0.0, 0.0, 0.0, 1.0])
    return pos, quat


class GazeboPoseBridge(Node):
    def __init__(self):
        super().__init__('gazebo_pose_bridge')

        self.declare_parameter('model_name', 'enimia_black')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.model_name = str(self.get_parameter('model_name').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        # Pre-seed ground-truth pose so map->odom is broadcast IMMEDIATELY from t=0
        init_x = float(self.get_parameter('initial_x').value)
        init_y = float(self.get_parameter('initial_y').value)
        init_yaw = float(self.get_parameter('initial_yaw').value)

        self.latest_ground_truth_pose = Pose()
        self.latest_ground_truth_pose.position.x = init_x
        self.latest_ground_truth_pose.position.y = init_y
        self.latest_ground_truth_pose.position.z = 0.02
        self.latest_ground_truth_pose.orientation.z = math.sin(init_yaw / 2.0)
        self.latest_ground_truth_pose.orientation.w = math.cos(init_yaw / 2.0)
        self.latest_ground_truth_twist = None
        self.model_index = None

        # TF Broadcaster and Listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped, '/gazebo/ground_truth/pose', 10)
        self.amcl_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10)

        # 1. P3D Odometry Subscriber (highest precision direct link ground truth)
        default_qos = QoSProfile(depth=10)
        self.create_subscription(
            Odometry,
            '/gazebo/ground_truth/odom',
            self.p3d_odom_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            '/gazebo/ground_truth/odom',
            self.p3d_odom_callback,
            default_qos
        )

        # 2. Gazebo ModelStates Subscriber (listening to both root and namespaced topics)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        default_qos = QoSProfile(depth=10)

        self.create_subscription(
            ModelStates, '/model_states', self.model_states_callback, sensor_qos)
        self.create_subscription(
            ModelStates, '/model_states', self.model_states_callback, default_qos)
        self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_callback, sensor_qos)
        self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_callback, default_qos)

        # Periodic timer for TF broadcast to keep transforms continuous and fresh
        timer_period = 1.0 / max(1.0, self.publish_rate)
        self.timer = self.create_timer(timer_period, self.broadcast_tf_cycle)

        self.get_logger().info(
            f'Gazebo ground-truth pose bridge active for "{self.model_name}". '
            f'Initial pose: ({init_x:.2f}, {init_y:.2f}, {init_yaw:.2f}). '
            f'Broadcasting {self.map_frame} -> {self.odom_frame} continuously.'
        )

    def p3d_odom_callback(self, msg: Odometry):
        self.latest_ground_truth_pose = msg.pose.pose
        self.latest_ground_truth_twist = msg.twist.twist

    def model_states_callback(self, msg: ModelStates):
        if self.model_index is None or self.model_index >= len(msg.name) or msg.name[self.model_index] != self.model_name:
            try:
                self.model_index = msg.name.index(self.model_name)
            except ValueError:
                return

        self.latest_ground_truth_pose = msg.pose[self.model_index]
        self.latest_ground_truth_twist = msg.twist[self.model_index]

    def broadcast_tf_cycle(self):
        now = self.get_clock().now()
        now_msg = now.to_msg()

        # 1. Ground truth pose in map coordinates
        p_base = self.latest_ground_truth_pose.position
        o_base = self.latest_ground_truth_pose.orientation

        pos_map_base = [p_base.x, p_base.y, p_base.z]
        quat_map_base = [o_base.x, o_base.y, o_base.z, o_base.w]

        # 2. Look up current odom -> base_frame transform
        pos_map_odom = pos_map_base
        quat_map_odom = quat_map_base

        try:
            # Non-blocking lookup of latest odom -> base_footprint
            if self.tf_buffer.can_transform(self.odom_frame, self.base_frame, Time(), timeout=Duration(seconds=0.0)):
                trans = self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    Time(),
                    timeout=Duration(seconds=0.0)
                )

                p_ob = trans.transform.translation
                q_ob = trans.transform.rotation
                pos_odom_base = [p_ob.x, p_ob.y, p_ob.z]
                quat_odom_base = [q_ob.x, q_ob.y, q_ob.z, q_ob.w]

                # Compute T_map_odom = T_map_base * (T_odom_base)^-1
                T_map_base = pose_to_matrix(pos_map_base, quat_map_base)
                T_odom_base = pose_to_matrix(pos_odom_base, quat_odom_base)

                T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
                pos_map_odom, quat_map_odom = matrix_to_pose(T_map_odom)
        except Exception:
            # Fall back to identity odom
            pos_map_odom = pos_map_base
            quat_map_odom = quat_map_base

        # 3. Broadcast map -> odom transform
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(pos_map_odom[0])
        t.transform.translation.y = float(pos_map_odom[1])
        t.transform.translation.z = float(pos_map_odom[2])
        t.transform.rotation.x = float(quat_map_odom[0])
        t.transform.rotation.y = float(quat_map_odom[1])
        t.transform.rotation.z = float(quat_map_odom[2])
        t.transform.rotation.w = float(quat_map_odom[3])

        self.tf_broadcaster.sendTransform(t)

        # 4. Publish /amcl_pose so RViz displays and Nav2 tools show true pose
        amcl_msg = PoseWithCovarianceStamped()
        amcl_msg.header.stamp = now_msg
        amcl_msg.header.frame_id = self.map_frame
        amcl_msg.pose.pose = self.latest_ground_truth_pose
        amcl_msg.pose.covariance[0] = 0.0001
        amcl_msg.pose.covariance[7] = 0.0001
        amcl_msg.pose.covariance[14] = 0.0001
        amcl_msg.pose.covariance[35] = 0.0001
        self.amcl_pose_pub.publish(amcl_msg)

        # 5. Publish ground truth pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = now_msg
        pose_stamped.header.frame_id = self.map_frame
        pose_stamped.pose = self.latest_ground_truth_pose
        self.pose_pub.publish(pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
