#!/usr/bin/env python3
"""Wheel / teleop odometry for the real ENIMIA prototype.

Publishes TF odom -> base_footprint and /odom so RobotModel in RViz matches
teleoperation. The LiDAR stays fixed on the robot via URDF (livox_link) and
must NOT drive the robot pose.

Sources (priority):
  1) /cmd_vel integration (teleop) — default until wheel encoders are wired
  2) optional external /wheel_odom (nav_msgs/Odometry) if odom_source:=wheel

base_footprint stays on the ground plane (z = 0).
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class WheelTeleopOdom(Node):
    def __init__(self) -> None:
        super().__init__('wheel_teleop_odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_source', 'cmd_vel')  # cmd_vel | wheel
        self.declare_parameter('wheel_odom_topic', '/wheel_odom')
        self.declare_parameter('wheelbase', 1.704)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_map_to_odom', True)
        self.declare_parameter('rate_hz', 50.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_source = str(self.get_parameter('odom_source').value).lower()
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_map = bool(self.get_parameter('publish_map_to_odom').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vyaw = 0.0
        self._last_time = self.get_clock().now()

        self._tf_br = TransformBroadcaster(self)
        self._static_br = StaticTransformBroadcaster(self)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 30
        )

        if self.publish_map:
            alias = TransformStamped()
            alias.header.stamp = self.get_clock().now().to_msg()
            alias.header.frame_id = self.map_frame
            alias.child_frame_id = self.odom_frame
            alias.transform.rotation.w = 1.0
            self._static_br.sendTransform(alias)

        if self.odom_source == 'wheel':
            self.create_subscription(
                Odometry,
                self.get_parameter('wheel_odom_topic').value,
                self._on_wheel_odom,
                30,
            )
            self.get_logger().info(
                f'Using external wheel odom on '
                f'{self.get_parameter("wheel_odom_topic").value}'
            )
        else:
            self.create_subscription(
                Twist,
                self.get_parameter('cmd_vel_topic').value,
                self._on_cmd_vel,
                30,
            )
            rate = max(float(self.get_parameter('rate_hz').value), 1.0)
            self.create_timer(1.0 / rate, self._integrate)
            self.get_logger().info(
                'Teleop odom: integrating /cmd_vel -> /odom + '
                f'TF {self.odom_frame}->{self.base_frame} (z=0 ground). '
                'Replace with encoder /wheel_odom when available '
                '(odom_source:=wheel).'
            )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.vx = float(msg.linear.x)
        # Prefer angular.z; if only steering-like vy unused, keep yaw rate
        self.vyaw = float(msg.angular.z)

    def _on_wheel_odom(self, msg: Odometry) -> None:
        # Pass-through / re-stamp into odom->base_footprint at ground height
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose = msg.pose
        out.pose.pose.position.z = 0.0
        out.twist = msg.twist
        self.x = out.pose.pose.position.x
        self.y = out.pose.pose.position.y
        q = out.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._odom_pub.publish(out)
        if self.publish_tf:
            self._send_tf(out.header.stamp, self.x, self.y, self.yaw)

    def _integrate(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1.0e-9
        self._last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        # Bicycle / differential approximation for teleop Twist
        self.yaw += self.vyaw * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

        stamp = now.to_msg()
        half = self.yaw * 0.5
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0  # on the ground
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vyaw
        self._odom_pub.publish(odom)
        if self.publish_tf:
            self._send_tf(stamp, self.x, self.y, self.yaw)

    def _send_tf(self, stamp, x: float, y: float, yaw: float) -> None:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        half = yaw * 0.5
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)
        self._tf_br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = WheelTeleopOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
