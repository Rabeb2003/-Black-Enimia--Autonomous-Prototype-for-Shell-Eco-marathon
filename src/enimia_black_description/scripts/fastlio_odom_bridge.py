#!/usr/bin/env python3
"""Bridge FAST-LIO LiDAR odometry into the robot frame tree and /odom.

FAST-LIO publishes TF camera_init->body and /Odometry.
This node adds body->base_footprint (LiDAR mount) so RobotModel follows
LiDAR odometry, and republishes /odom for RViz/Nav2 consumers.

No wheel fusion: odometry is LiDAR-primary (FAST-LIO).
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def _xyz_rpy_to_quat(roll: float, pitch: float, yaw: float):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _rotate_vector(q, vx: float, vy: float, vz: float):
    ox, oy, oz, ow = q.x, q.y, q.z, q.w
    tx = 2.0 * (oy * vz - oz * vy)
    ty = 2.0 * (oz * vx - ox * vz)
    tz = 2.0 * (ox * vy - oy * vx)
    return (
        vx + ow * tx + (oy * tz - oz * ty),
        vy + ow * ty + (oz * tx - ox * tz),
        vz + ow * tz + (ox * ty - oy * tx),
    )


class FastLioOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__('fastlio_odom_bridge')

        self.declare_parameter('fastlio_odom_topic', '/Odometry')
        self.declare_parameter('output_odom_topic', '/odom')
        # ROS map frame (RViz Fixed Frame). Identity with FAST-LIO world.
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('fastlio_world_frame', 'camera_init')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('body_frame', 'body')
        self.declare_parameter('base_frame', 'base_footprint')
        # Livox on base_link (0.30,0,0.48) + base_link z above footprint (0.21)
        self.declare_parameter('body_to_base_xyz', [-0.30, 0.0, -0.69])
        self.declare_parameter('body_to_base_rpy', [0.0, 0.0, 0.0])

        self._in_topic = self.get_parameter('fastlio_odom_topic').value
        self._out_topic = self.get_parameter('output_odom_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._world_frame = self.get_parameter('fastlio_world_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._body_frame = self.get_parameter('body_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._xyz = [float(v) for v in self.get_parameter('body_to_base_xyz').value]
        rpy = [float(v) for v in self.get_parameter('body_to_base_rpy').value]

        self._static_br = StaticTransformBroadcaster(self)
        self._pub = self.create_publisher(Odometry, self._out_topic, 20)
        self.create_subscription(Odometry, self._in_topic, self._on_odom, 20)

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = _xyz_rpy_to_quat(rpy[0], rpy[1], rpy[2])

        def _identity(parent: str, child: str) -> TransformStamped:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            return t

        body_to_base = TransformStamped()
        body_to_base.header.stamp = stamp
        body_to_base.header.frame_id = self._body_frame
        body_to_base.child_frame_id = self._base_frame
        body_to_base.transform.translation.x = self._xyz[0]
        body_to_base.transform.translation.y = self._xyz[1]
        body_to_base.transform.translation.z = self._xyz[2]
        body_to_base.transform.rotation.x = qx
        body_to_base.transform.rotation.y = qy
        body_to_base.transform.rotation.z = qz
        body_to_base.transform.rotation.w = qw

        # map -> camera_init -> (FAST-LIO) body -> base_footprint
        # map -> odom (alias) for Nav2-style consumers
        statics = [
            _identity(self._map_frame, self._world_frame),
            _identity(self._map_frame, self._odom_frame),
            body_to_base,
        ]
        self._static_br.sendTransform(statics)
        self.get_logger().info(
            f'Frames: {self._map_frame}->{self._world_frame}, '
            f'{self._map_frame}->{self._odom_frame}, '
            f'{self._body_frame}->{self._base_frame}; '
            f'odom topic {self._in_topic} -> {self._out_topic}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        out.pose = msg.pose
        out.twist = msg.twist
        wx, wy, wz = _rotate_vector(msg.pose.pose.orientation, *self._xyz)
        out.pose.pose.position.x = msg.pose.pose.position.x + wx
        out.pose.pose.position.y = msg.pose.pose.position.y + wy
        out.pose.pose.position.z = msg.pose.pose.position.z + wz
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = FastLioOdomBridge()
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
