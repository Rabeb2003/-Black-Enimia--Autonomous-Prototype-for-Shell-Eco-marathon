#!/usr/bin/env python3
"""Reliably seed AMCL once Gazebo odometry becomes available.

AMCL can be active before Gazebo has inserted the robot.  In that sequence,
its configured initial pose may be consumed before an ``odom`` transform is
available, leaving it without the ``map -> odom`` transform.  This node
republishes the initial pose after ``odom -> base_footprint`` exists and stops
as soon as AMCL broadcasts ``map -> odom``.
"""

import math
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class AmclInitialPosePublisher(Node):
    def __init__(self):
        super().__init__('amcl_initial_pose_publisher')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('retry_period', 2.0)
        self.declare_parameter('timeout', 360.0)

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.retry_period = float(self.get_parameter('retry_period').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.cancelled = False

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def has_transform(self, target_frame, source_frame):
        return self.tf_buffer.can_transform(
            target_frame, source_frame, Time(),
            timeout=Duration(seconds=0.0))

    def make_initial_pose(self):
        message = PoseWithCovarianceStamped()
        # A zero stamp requests the latest available odom transform in TF2.
        # It avoids a harmless but noisy future-extrapolation race between the
        # Gazebo clock and the EKF's first odometry sample.
        message.header.stamp = Time().to_msg()
        message.header.frame_id = self.map_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        # x, y and yaw covariance (row-major 6x6 covariance matrix).
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        return message

    def run(self):
        deadline = time.monotonic() + self.timeout
        next_log = 0.0
        while rclpy.ok() and not self.cancelled and time.monotonic() < deadline:
            if self.has_transform(self.map_frame, self.odom_frame):
                self.get_logger().info(
                    'AMCL is publishing map -> odom; initial-pose helper finished.')
                return

            if self.has_transform(self.odom_frame, self.base_frame):
                self.publisher.publish(self.make_initial_pose())
                self.get_logger().info(
                    f'Published initial pose ({self.x:.2f}, {self.y:.2f}, '
                    f'{self.yaw:.2f}) in {self.map_frame}; waiting for AMCL.')
                time.sleep(max(0.2, self.retry_period))
                continue

            now = time.monotonic()
            if now >= next_log:
                self.get_logger().info(
                    f'Waiting for {self.odom_frame} -> {self.base_frame} '
                    'before seeding AMCL.')
                next_log = now + 5.0
            time.sleep(0.5)

        if not self.cancelled and rclpy.ok():
            self.get_logger().error(
                'Timed out waiting for AMCL to publish map -> odom.')

    def destroy_node(self):
        self.cancelled = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AmclInitialPosePublisher()
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
