#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import math

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from ultralytics import YOLO


class StopSignNode(Node):
    def __init__(self):
        super().__init__('stop_sign_detector')

        self.declare_parameter('rgb_topic', '/oak/rgb/image_raw')
        self.declare_parameter('depth_topic', '/oak/depth/image_raw')
        self.declare_parameter('stop_topic', '/stop_detected')
        self.declare_parameter('distance_topic', '/stop_sign/distance')
        self.declare_parameter('brake_active_topic', '/stop_sign/braking')
        self.declare_parameter('output_image_topic', '/stop_sign/output')
        # Compatibility alias requested by the RViz setup.  The canonical
        # topic remains /stop_sign/output.
        self.declare_parameter('output_alias_topic', '/stip_sign_output')
        self.declare_parameter('depth_debug_topic', '/stop_sign/depth_debug')
        self.declare_parameter('model_file', 'best_stop.pt')
        self.declare_parameter('confidence', 0.55)
        self.declare_parameter('class_id', 0)
        self.declare_parameter('image_size', 416)
        self.declare_parameter('use_half', False)
        self.declare_parameter('stop_distance', 2.0)
        self.declare_parameter('stop_duration', 5.0)
        self.declare_parameter('rearm_distance', 3.5)
        self.declare_parameter('rearm_clear_duration', 1.0)
        self.declare_parameter('brake_detection_timeout', 0.5)
        self.declare_parameter('max_fps', 6.0)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_valid_distance', 0.25)
        self.declare_parameter('max_valid_distance', 15.0)
        self.declare_parameter('use_size_fallback', True)
        self.declare_parameter('real_stop_sign_width', 0.2)
        self.declare_parameter('focal_length_px', 386.0)
        self.declare_parameter('publish_debug_images', True)
        self.declare_parameter('display', False)

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_stamp = 0.0
        self.state = 'DRIVING'
        self.stop_started_at = 0.0
        self.clear_started_at = None
        self.last_process_time = 0.0
        self.last_detected_distance = float('inf')
        self.last_detection_at = -float('inf')

        self.confidence = float(self.get_parameter('confidence').value)
        self.class_id = int(self.get_parameter('class_id').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.use_half = bool(self.get_parameter('use_half').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.stop_duration = float(self.get_parameter('stop_duration').value)
        self.rearm_distance = float(self.get_parameter('rearm_distance').value)
        self.rearm_clear_duration = float(
            self.get_parameter('rearm_clear_duration').value)
        self.brake_detection_timeout = float(
            self.get_parameter('brake_detection_timeout').value)
        self.max_fps = float(self.get_parameter('max_fps').value)
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_valid_distance = float(self.get_parameter('min_valid_distance').value)
        self.max_valid_distance = float(self.get_parameter('max_valid_distance').value)
        self.use_size_fallback = bool(self.get_parameter('use_size_fallback').value)
        self.real_stop_sign_width = float(self.get_parameter('real_stop_sign_width').value)
        self.focal_length_px = float(self.get_parameter('focal_length_px').value)
        self.publish_debug_images = bool(self.get_parameter('publish_debug_images').value)
        self.display = bool(self.get_parameter('display').value)

        model_path = self.resolve_model_path(self.get_parameter('model_file').value)
        self.model = YOLO(model_path)
        self.get_logger().info(f'Stop sign model loaded: {model_path}')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        stop_topic = self.get_parameter('stop_topic').value
        distance_topic = self.get_parameter('distance_topic').value
        brake_active_topic = self.get_parameter('brake_active_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        output_alias_topic = self.get_parameter('output_alias_topic').value
        depth_debug_topic = self.get_parameter('depth_debug_topic').value

        self.create_subscription(Image, rgb_topic, self.rgb_callback, qos_profile)
        self.create_subscription(Image, depth_topic, self.depth_callback, qos_profile)
        self.stop_pub = self.create_publisher(Bool, stop_topic, 10)
        self.distance_pub = self.create_publisher(Float32, distance_topic, 10)
        self.brake_active_pub = self.create_publisher(Bool, brake_active_topic, 10)
        self.image_pub = self.create_publisher(Image, output_image_topic, 10)
        self.image_alias_pub = None
        if output_alias_topic and output_alias_topic != output_image_topic:
            self.image_alias_pub = self.create_publisher(Image, output_alias_topic, 10)
        self.depth_debug_pub = self.create_publisher(Image, depth_debug_topic, 10)
        self.create_timer(0.2, self.publish_stop_state)

        self.get_logger().info(
            f'Stop detection on {rgb_topic} + {depth_topic}, stop distance {self.stop_distance:.1f} m'
        )

    def resolve_model_path(self, model_file):
        candidates = [model_file]
        for pkg_name in ['enimia_black_description', 'lane_detection']:
            try:
                pkg_dir = get_package_share_directory(pkg_name)
                candidates.append(os.path.join(pkg_dir, 'models', model_file))
                candidates.append(os.path.join(pkg_dir, model_file))
            except Exception:
                pass
        candidates.extend([
            os.path.join(os.path.dirname(__file__), '..', 'models', model_file),
            os.path.join(os.path.dirname(__file__), '..', 'share', 'enimia_black_description', 'models', model_file),
        ])
        for candidate in candidates:
            path = os.path.abspath(candidate)
            if os.path.exists(path):
                return path
        raise FileNotFoundError(f'Stop sign model not found: {model_file}')

    def depth_callback(self, msg):
        try:
            if msg.encoding == '16UC1':
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth = depth_raw.astype(np.float32) * self.depth_scale
            elif msg.encoding == '32FC1':
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                depth = self.bridge.imgmsg_to_cv2(msg)
                if depth.dtype == np.uint16:
                    depth = depth.astype(np.float32) * self.depth_scale
                else:
                    depth = depth.astype(np.float32)
            self.latest_depth = depth
            self.latest_depth_stamp = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f'Depth conversion failed: {exc}')

    def rgb_callback(self, msg):
        now = time.monotonic()
        if self.max_fps > 0.0 and now - self.last_process_time < 1.0 / self.max_fps:
            return
        self.last_process_time = now

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            depth = self.latest_depth
            if depth is not None and image.shape[:2] != depth.shape[:2]:
                depth = cv2.resize(depth, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)

            annotated = image.copy()
            nearest_distance = self.detect_stop_sign(image, depth, annotated)
            # The safety gate receives an explicit braking state only while a
            # current RGB inference contains a real STOP panel.  Its distance
            # then defines the smooth approach envelope down to 2.5 m.
            self.last_detected_distance = nearest_distance
            self.last_detection_at = now
            self.distance_pub.publish(Float32(data=(
                float(nearest_distance) if np.isfinite(nearest_distance) else 0.0)))
            self.update_stop_state(now, nearest_distance)

            self.draw_status(annotated, self.state)
            self.publish_debug(annotated, depth)
        except Exception as exc:
            self.get_logger().error(f'Stop sign processing error: {exc}')

    def detect_stop_sign(self, image, depth, annotated):
        results = self.model.predict(
            image,
            conf=self.confidence,
            imgsz=self.image_size,
            half=self.use_half,
            verbose=False,
        )

        nearest_distance = float('inf')
        for result in results:
            for box in result.boxes:
                score = float(box.conf[0])
                label = int(box.cls[0])
                if label != self.class_id or score < self.confidence:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                distance = self.distance_from_depth(x1, y1, x2, y2, depth)
                if not np.isfinite(distance) and self.use_size_fallback:
                    distance = self.distance_from_size(x2 - x1, y2 - y1)

                if np.isfinite(distance):
                    nearest_distance = min(nearest_distance, distance)

                self.draw_detection(
                    annotated, x1, y1, x2, y2, score, distance,
                    np.isfinite(distance) and distance <= self.stop_distance)

        return nearest_distance

    def update_stop_state(self, now, nearest_distance):
        """Implement one stop per sign: brake, hold five seconds, re-arm.

        After the hold, the same panel remains visible for a moment.  The
        WAIT_CLEAR state prevents it from immediately retriggering while the
        car pulls away; re-arming requires the detected panel to be farther
        than ``rearm_distance`` (or absent) for a short stable interval.
        """
        if self.state == 'DRIVING':
            if np.isfinite(nearest_distance) and nearest_distance <= self.stop_distance:
                self.state = 'HOLD'
                self.stop_started_at = now
                self.launch_stop_clock_ui(self.stop_duration)
                self.get_logger().warn(
                    f'Stop sign at {nearest_distance:.2f} m: holding for '
                    f'{self.stop_duration:.1f} s')
            return

        if self.state == 'HOLD':
            if now - self.stop_started_at >= self.stop_duration:
                self.state = 'WAIT_CLEAR'
                self.clear_started_at = None
                self.get_logger().info('Stop complete: vehicle may pull away')
            return

        # WAIT_CLEAR
        clear = (not np.isfinite(nearest_distance) or
                 nearest_distance >= self.rearm_distance)
        if clear:
            if self.clear_started_at is None:
                self.clear_started_at = now
            elif now - self.clear_started_at >= self.rearm_clear_duration:
                self.state = 'DRIVING'
                self.clear_started_at = None
                self.get_logger().info('Stop detector re-armed for the next sign')
        else:
            self.clear_started_at = None

    def launch_stop_clock_ui(self, duration):
        try:
            candidates = [
                os.path.join(os.path.dirname(__file__), 'stop_sign_clock.py'),
                os.path.join(os.path.dirname(__file__), '..', 'scripts', 'stop_sign_clock.py'),
                '/home/rabeb/Downloads/enimia_black/src/enimia_black_description/scripts/stop_sign_clock.py',
                '/home/rabeb/Downloads/enimia_black/install/enimia_black_description/lib/enimia_black_description/stop_sign_clock.py',
            ]
            try:
                pkg_dir = get_package_share_directory('enimia_black_description')
                candidates.append(os.path.join(pkg_dir, 'scripts', 'stop_sign_clock.py'))
                candidates.append(os.path.join(pkg_dir, '..', 'lib', 'enimia_black_description', 'stop_sign_clock.py'))
            except Exception:
                pass

            env = os.environ.copy()
            if 'DISPLAY' not in env or not env['DISPLAY']:
                env['DISPLAY'] = ':0'
            env['QT_QPA_PLATFORM'] = 'xcb'

            for script_path in candidates:
                if os.path.exists(script_path):
                    subprocess.Popen(
                        [sys.executable, os.path.abspath(script_path), '--duration', str(duration)],
                        env=env,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        preexec_fn=os.setpgrp,
                    )
                    self.get_logger().info(f'Stop Sign 5s Countdown Clock UI opened successfully from: {script_path}')
                    return
            self.get_logger().error(f'Stop Sign Clock script not found in candidates: {candidates}')
        except Exception as e:
            self.get_logger().warn(f'Could not launch Stop Sign Clock UI: {e}')

    def distance_from_depth(self, x1, y1, x2, y2, depth):
        if depth is None:
            return float('inf')

        height, width = depth.shape[:2]
        cx = int(np.clip((x1 + x2) / 2.0, 0, width - 1))
        cy = int(np.clip((y1 + y2) / 2.0, 0, height - 1))
        roi = max(4, min(abs(x2 - x1), abs(y2 - y1)) // 5)

        xs = max(0, cx - roi)
        xe = min(width, cx + roi + 1)
        ys = max(0, cy - roi)
        ye = min(height, cy + roi + 1)
        depth_roi = depth[ys:ye, xs:xe]
        if depth_roi.size == 0:
            return float('inf')

        valid = depth_roi[np.isfinite(depth_roi)]
        valid = valid[(valid > self.min_valid_distance) & (valid < self.max_valid_distance)]
        if valid.size < 5:
            return float('inf')
        return float(np.median(valid))

    def distance_from_size(self, box_width, box_height):
        apparent_size = max(float(box_width), float(box_height))
        if apparent_size <= 1.0:
            return float('inf')
        distance = (self.real_stop_sign_width * self.focal_length_px) / apparent_size
        if distance < self.min_valid_distance or distance > self.max_valid_distance:
            return float('inf')
        return distance

    def draw_detection(self, image, x1, y1, x2, y2, score, distance, close_stop):
        color = (0, 0, 255) if close_stop else (0, 180, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        distance_text = f'{distance:.2f}m' if np.isfinite(distance) else 'no depth'
        cv2.putText(
            image,
            f'STOP {distance_text} {score:.2f}',
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

    def draw_status(self, image, status):
        if self.state == 'HOLD':
            now = time.monotonic()
            elapsed = now - self.stop_started_at
            remaining = max(0.0, self.stop_duration - elapsed)
            # Prominent Red & Yellow Stop Sign Clock Banner
            cv2.rectangle(image, (10, 10), (480, 85), (20, 20, 28), -1)
            cv2.rectangle(image, (10, 10), (480, 85), (0, 0, 255), 2)
            cv2.putText(
                image,
                "PANNEAU STOP - PROTOTYPE IMMOBILISE",
                (20, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 0, 255),
                2,
            )
            cv2.putText(
                image,
                f"CHRONO ARRET : {remaining:.1f} s / {self.stop_duration:.1f} s",
                (20, 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 255),
                2,
            )
            # Circular progress mini-gauge
            center_x, center_y = 440, 48
            radius = 24
            cv2.circle(image, (center_x, center_y), radius, (60, 60, 75), 4)
            angle = int((remaining / self.stop_duration) * 360)
            cv2.ellipse(image, (center_x, center_y), (radius, radius), -90, 0, angle, (0, 255, 120), 4)
        else:
            cv2.putText(
                image,
                status,
                (12, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
            )

    def publish_debug(self, annotated, depth):
        if self.publish_debug_images:
            debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.image_pub.publish(debug_msg)
            if self.image_alias_pub is not None:
                self.image_alias_pub.publish(debug_msg)
            if depth is not None:
                depth_visual = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
                depth_visual = np.clip(depth_visual, 0.0, 8.0)
                depth_visual = (depth_visual / 8.0 * 255).astype(np.uint8)
                depth_visual = cv2.applyColorMap(depth_visual, cv2.COLORMAP_JET)
                self.depth_debug_pub.publish(self.bridge.cv2_to_imgmsg(depth_visual, encoding='bgr8'))

        if self.display and os.getenv('DISPLAY'):
            cv2.imshow('Stop Sign Detection', annotated)
            cv2.waitKey(1)

    def publish_stop_state(self):
        # ``stop_detected`` is the hard-stop request.  ``braking`` is active
        # only on approach; it lets the command gate follow the physical
        # braking envelope before the vehicle reaches the 2.5 m stop point.
        self.stop_pub.publish(Bool(data=self.state == 'HOLD'))
        braking_approach = (
            self.state == 'DRIVING' and
            math.isfinite(self.last_detected_distance) and
            time.monotonic() - self.last_detection_at <=
            self.brake_detection_timeout)
        self.brake_active_pub.publish(Bool(data=braking_approach))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StopSignNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node.display and os.getenv('DISPLAY'):
                cv2.destroyAllWindows()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
