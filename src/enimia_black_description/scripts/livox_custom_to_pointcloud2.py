#!/usr/bin/env python3
"""Convert Livox CustomMsg (/livox/lidar) to sensor_msgs/PointCloud2.

Keeps the LiDAR cloud in livox_link so RViz shows it fixed on the prototype
(URDF), independent of FAST-LIO's camera_init/body tree.
"""

from __future__ import annotations

import array
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        'livox_ros_driver2 not sourced — cannot import CustomMsg'
    ) from exc


class LivoxCustomToPointCloud2(Node):
    def __init__(self) -> None:
        super().__init__('livox_custom_to_pointcloud2')
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/points')
        self.declare_parameter('frame_id', 'livox_link')

        self._frame_id = self.get_parameter('frame_id').value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, qos
        )
        self.create_subscription(
            CustomMsg,
            self.get_parameter('input_topic').value,
            self._on_msg,
            qos,
        )
        self.get_logger().info(
            f'Livox CustomMsg -> PointCloud2 on '
            f'{self.get_parameter("output_topic").value} '
            f'(frame={self._frame_id})'
        )

    def _on_msg(self, msg: CustomMsg) -> None:
        n = int(msg.point_num)
        # x y z intensity (float32) — 16 bytes/point
        buf = array.array('f', [0.0] * (n * 4))
        i = 0
        for p in msg.points:
            buf[i] = float(p.x)
            buf[i + 1] = float(p.y)
            buf[i + 2] = float(p.z)
            buf[i + 3] = float(p.reflectivity)
            i += 4

        cloud = PointCloud2()
        cloud.header = Header()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = self._frame_id or msg.header.frame_id or 'livox_link'
        cloud.height = 1
        cloud.width = n
        cloud.is_dense = False
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * n
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.data = buf.tobytes()
        self._pub.publish(cloud)


def main() -> None:
    rclpy.init()
    node = LivoxCustomToPointCloud2()
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
