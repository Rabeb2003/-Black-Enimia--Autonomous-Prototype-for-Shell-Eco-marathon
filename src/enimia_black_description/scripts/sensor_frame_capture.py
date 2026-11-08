#!/usr/bin/env python3
"""Capture RGB (+ depth) frames from OAK topics for offline / perception work."""

from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class SensorFrameCapture(Node):
    def __init__(self) -> None:
        super().__init__('sensor_frame_capture')

        self.declare_parameter('rgb_topic', '/oak/rgb/image_raw')
        # depthai RGBD publishes depth on stereo/image_raw (also aliased to depth/image_raw)
        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')
        self.declare_parameter(
            'output_dir',
            str(Path.home() / 'Downloads' / 'enimia_black' / 'captures'),
        )
        self.declare_parameter('save_hz', 2.0)
        self.declare_parameter('save_rgb', True)
        self.declare_parameter('save_depth', True)
        self.declare_parameter('show_windows', True)
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('max_frames', 0)  # 0 = unlimited

        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.save_hz = float(self.get_parameter('save_hz').value)
        self.save_rgb = bool(self.get_parameter('save_rgb').value)
        self.save_depth = bool(self.get_parameter('save_depth').value)
        self.show_windows = bool(self.get_parameter('show_windows').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.max_frames = int(self.get_parameter('max_frames').value)

        self.bridge = CvBridge()
        self._latest_rgb = None
        self._latest_depth = None
        self._saved = 0
        self._session = datetime.now().strftime('%Y%m%d_%H%M%S')

        rgb_dir = self.output_dir / self._session / 'rgb'
        depth_dir = self.output_dir / self._session / 'depth'
        if self.save_rgb:
            rgb_dir.mkdir(parents=True, exist_ok=True)
        if self.save_depth:
            depth_dir.mkdir(parents=True, exist_ok=True)
        self._rgb_dir = rgb_dir
        self._depth_dir = depth_dir

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, self.rgb_topic, self._on_rgb, qos)
        self.create_subscription(Image, self.depth_topic, self._on_depth, qos)

        period = 1.0 / max(self.save_hz, 0.1)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Capturing RGB={self.rgb_topic} depth={self.depth_topic} -> {self.output_dir}/{self._session}'
        )

    def _on_rgb(self, msg: Image) -> None:
        try:
            self._latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'RGB convert failed: {exc}')

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self._latest_depth = depth
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Depth convert failed: {exc}')

    def _depth_vis(self, depth: np.ndarray) -> np.ndarray:
        if depth.dtype == np.float32:
            # meters -> mm-ish scale for display
            scaled = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            scaled = np.clip(scaled * 1000.0, 0, 10000)
            depth_u16 = scaled.astype(np.uint16)
        elif depth.dtype == np.uint16:
            depth_u16 = depth
        else:
            depth_u16 = cv2.normalize(depth, None, 0, 65535, cv2.NORM_MINMAX).astype(np.uint16)

        vis = cv2.convertScaleAbs(depth_u16, alpha=0.03)
        return cv2.applyColorMap(vis, cv2.COLORMAP_TURBO)

    def _on_timer(self) -> None:
        stamp = datetime.now().strftime('%H%M%S_%f')[:-3]
        wrote = False

        if self.save_rgb and self._latest_rgb is not None:
            path = self._rgb_dir / f'rgb_{self._saved:06d}_{stamp}.jpg'
            cv2.imwrite(
                str(path),
                self._latest_rgb,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            wrote = True
            if self.show_windows:
                cv2.imshow('OAK RGB', self._latest_rgb)

        if self.save_depth and self._latest_depth is not None:
            raw_path = self._depth_dir / f'depth_{self._saved:06d}_{stamp}.png'
            # Keep raw depth when possible for algorithms.
            if self._latest_depth.dtype == np.uint16:
                cv2.imwrite(str(raw_path), self._latest_depth)
            elif self._latest_depth.dtype == np.float32:
                mm = np.clip(np.nan_to_num(self._latest_depth) * 1000.0, 0, 65535)
                cv2.imwrite(str(raw_path), mm.astype(np.uint16))
            else:
                cv2.imwrite(str(raw_path), self._latest_depth)

            vis = self._depth_vis(self._latest_depth)
            vis_path = self._depth_dir / f'depth_vis_{self._saved:06d}_{stamp}.jpg'
            cv2.imwrite(str(vis_path), vis)
            wrote = True
            if self.show_windows:
                cv2.imshow('OAK Depth', vis)

        if self.show_windows:
            cv2.waitKey(1)

        if wrote:
            self._saved += 1
            if self._saved == 1 or self._saved % 20 == 0:
                self.get_logger().info(f'Saved {self._saved} frame pair(s)')
            if self.max_frames > 0 and self._saved >= self.max_frames:
                self.get_logger().info('Reached max_frames — shutting down capture node')
                raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = SensorFrameCapture()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
