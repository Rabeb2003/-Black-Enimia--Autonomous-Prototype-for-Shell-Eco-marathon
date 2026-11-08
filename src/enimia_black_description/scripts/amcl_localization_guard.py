#!/usr/bin/env python3
"""Keep AMCL recoverable in Gazebo without replacing normal LiDAR matching.

AMCL remains the source of the ``map -> odom`` transform.  In a simulator we
also have a precise Gazebo P3D pose.  This node uses it only as a recovery
reference: when AMCL jumps far away or stops publishing, it sends one
``/initialpose`` message so AMCL can resume scan matching at the right place.

It additionally publishes ``/localization/guarded_pose``.  The lane controller
uses this pose for small, world-specific safety guides (not for Nav2), so a
brief AMCL recovery cannot make the first right turn miss kR_in.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def normalized_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class AmclLocalizationGuard(Node):
    def __init__(self):
        super().__init__('amcl_localization_guard')

        self.declare_parameter('ground_truth_topic', '/gazebo/ground_truth/odom')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('guarded_pose_topic', '/localization/guarded_pose')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('use_ground_truth_recovery', True)
        self.declare_parameter('position_error_threshold', 1.50)
        self.declare_parameter('yaw_error_threshold', 0.60)
        self.declare_parameter('amcl_timeout', 1.50)
        self.declare_parameter('startup_grace_period', 8.0)
        self.declare_parameter('recovery_cooldown', 3.0)
        self.declare_parameter('publish_rate', 20.0)

        self.ground_truth_topic = str(
            self.get_parameter('ground_truth_topic').value)
        self.amcl_pose_topic = str(self.get_parameter('amcl_pose_topic').value)
        self.guarded_pose_topic = str(
            self.get_parameter('guarded_pose_topic').value)
        self.initial_pose_topic = str(
            self.get_parameter('initial_pose_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.use_ground_truth_recovery = bool(
            self.get_parameter('use_ground_truth_recovery').value)
        self.position_error_threshold = float(
            self.get_parameter('position_error_threshold').value)
        self.yaw_error_threshold = float(
            self.get_parameter('yaw_error_threshold').value)
        self.amcl_timeout = float(self.get_parameter('amcl_timeout').value)
        self.startup_grace_period = float(
            self.get_parameter('startup_grace_period').value)
        self.recovery_cooldown = float(
            self.get_parameter('recovery_cooldown').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        if (self.position_error_threshold <= 0.0 or
                self.yaw_error_threshold <= 0.0 or
                self.amcl_timeout <= 0.0 or self.recovery_cooldown <= 0.0):
            raise ValueError('localization-guard thresholds and periods must be positive')

        self.ground_truth_pose = None
        self.amcl_pose = None
        self.last_amcl_at = 0.0
        self.started_at = time.monotonic()
        self.last_recovery_at = -math.inf
        self.last_error = math.inf
        self.recovering = False

        self.guarded_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.guarded_pose_topic, 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.initial_pose_topic, 10)
        self.healthy_pub = self.create_publisher(Bool, '/localization/healthy', 10)
        self.recovering_pub = self.create_publisher(
            Bool, '/localization/recovering', 10)
        self.error_pub = self.create_publisher(Float32, '/localization/error', 10)

        self.create_subscription(
            Odometry, self.ground_truth_topic, self.ground_truth_callback, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_pose_topic,
            self.amcl_pose_callback, 20)
        self.create_timer(1.0 / max(1.0, publish_rate), self.timer_callback)

        self.get_logger().info(
            'AMCL localization guard ready: LiDAR/AMCL remains primary; '
            f'Gazebo recovery threshold is {self.position_error_threshold:.2f} m.')

    def ground_truth_callback(self, message):
        self.ground_truth_pose = message.pose.pose

    def amcl_pose_callback(self, message):
        self.amcl_pose = message.pose.pose
        self.last_amcl_at = time.monotonic()

    def pose_error(self):
        if self.ground_truth_pose is None or self.amcl_pose is None:
            return math.inf, math.inf
        dx = self.amcl_pose.position.x - self.ground_truth_pose.position.x
        dy = self.amcl_pose.position.y - self.ground_truth_pose.position.y
        position_error = math.hypot(dx, dy)
        yaw_error = abs(normalized_angle(
            yaw_from_quaternion(self.amcl_pose.orientation) -
            yaw_from_quaternion(self.ground_truth_pose.orientation)))
        return position_error, yaw_error

    def make_pose_message(self):
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.pose.pose = self.ground_truth_pose
        # P3D is exact in Gazebo.  A compact covariance lets AMCL recover
        # immediately, but remains non-zero for numerical stability.
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.01
        return message

    def timer_callback(self):
        now = time.monotonic()
        if self.ground_truth_pose is None:
            self.healthy_pub.publish(Bool(data=False))
            self.recovering_pub.publish(Bool(data=False))
            return

        guarded_pose = self.make_pose_message()
        self.guarded_pose_pub.publish(guarded_pose)

        position_error, yaw_error = self.pose_error()
        self.last_error = position_error
        amcl_fresh = ((now - self.last_amcl_at) <= self.amcl_timeout and
                      self.amcl_pose is not None)
        healthy = (amcl_fresh and position_error <= self.position_error_threshold
                   and yaw_error <= self.yaw_error_threshold)
        startup_complete = now - self.started_at >= self.startup_grace_period
        needs_recovery = startup_complete and not healthy

        self.healthy_pub.publish(Bool(data=healthy))
        self.recovering_pub.publish(Bool(data=needs_recovery))
        self.error_pub.publish(Float32(data=float(
            position_error if math.isfinite(position_error) else 99.0)))

        if (self.use_ground_truth_recovery and needs_recovery and
                now - self.last_recovery_at >= self.recovery_cooldown):
            self.initial_pose_pub.publish(guarded_pose)
            self.last_recovery_at = now
            self.get_logger().warn(
                'AMCL drift/loss detected '
                f'(position error {position_error:.2f} m, yaw error '
                f'{math.degrees(yaw_error):.1f} deg); re-seeding from Gazebo P3D.')


def main(args=None):
    rclpy.init(args=args)
    node = AmclLocalizationGuard()
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
