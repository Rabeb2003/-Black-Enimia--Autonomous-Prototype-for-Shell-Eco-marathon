#!/usr/bin/env python3
"""Detect the orange-yellow stop line painted in the SEM Gazebo circuit.

The active world contains three transverse markings: yellow start and finish
lines and one orange-yellow ``stop_line``.  A generic stop-sign network used
to stop for the red panel as well, which is not the intended behaviour.  This
node deliberately accepts only a wide, horizontal orange-yellow marking on
the road.  It therefore ignores the red STOP panel, cones, white lane lines,
and the bright-yellow start/finish markings.

The node keeps the same stop/braking interface used by ``cmd_vel_safety_gate``
so braking starts smoothly from the OAK-D depth measurement and the vehicle
holds only after reaching the painted line.
"""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class YellowStopLineDetector(Node):
    """Vision and depth detector for the circuit's orange-yellow stop line."""

    def __init__(self):
        super().__init__('yellow_stop_line_detector')

        self.declare_parameter('rgb_topic', '/oak/rgb/image_raw')
        self.declare_parameter('depth_topic', '/oak/depth/image_raw')
        self.declare_parameter('stop_topic', '/stop_line/detected')
        self.declare_parameter('distance_topic', '/stop_line/distance')
        self.declare_parameter('brake_active_topic', '/stop_line/braking')
        self.declare_parameter('output_image_topic', '/stop_line/output')
        self.declare_parameter('max_fps', 15.0)
        self.declare_parameter('stop_distance', 1.25)
        self.declare_parameter('stop_duration', 5.0)
        self.declare_parameter('rearm_clear_duration', 1.0)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('roi_top_ratio', 0.35)
        self.declare_parameter('roi_bottom_ratio', 0.96)
        self.declare_parameter('min_width_ratio', 0.40)
        self.declare_parameter('min_aspect_ratio', 3.0)
        self.declare_parameter('center_tolerance_ratio', 0.28)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_valid_distance', 0.25)
        self.declare_parameter('max_valid_distance', 15.0)
        self.declare_parameter('publish_debug_image', True)

        self.max_fps = float(self.get_parameter('max_fps').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.stop_duration = float(self.get_parameter('stop_duration').value)
        self.rearm_clear_duration = float(
            self.get_parameter('rearm_clear_duration').value)
        self.confirm_frames = max(1, int(self.get_parameter('confirm_frames').value))
        self.roi_top_ratio = float(self.get_parameter('roi_top_ratio').value)
        self.roi_bottom_ratio = float(self.get_parameter('roi_bottom_ratio').value)
        self.min_width_ratio = float(self.get_parameter('min_width_ratio').value)
        self.min_aspect_ratio = float(self.get_parameter('min_aspect_ratio').value)
        self.center_tolerance_ratio = float(
            self.get_parameter('center_tolerance_ratio').value)
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_valid_distance = float(
            self.get_parameter('min_valid_distance').value)
        self.max_valid_distance = float(
            self.get_parameter('max_valid_distance').value)
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value)

        self.bridge = CvBridge()
        self.latest_depth = None
        self.last_process_time = 0.0
        self.state = 'DRIVING'
        self.stop_started_at = 0.0
        self.clear_started_at = None
        self.consecutive_detections = 0

        self.stop_pub = self.create_publisher(
            Bool, self.get_parameter('stop_topic').value, 10)
        self.distance_pub = self.create_publisher(
            Float32, self.get_parameter('distance_topic').value, 10)
        self.brake_pub = self.create_publisher(
            Bool, self.get_parameter('brake_active_topic').value, 10)
        self.image_pub = self.create_publisher(
            Image, self.get_parameter('output_image_topic').value, 10)

        self.create_subscription(
            Image, self.get_parameter('rgb_topic').value,
            self.rgb_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self.publish_state)

        self.get_logger().info(
            'Yellow stop-line detector active: only the wide orange-yellow '
            'road marking can request a stop.')

    def depth_callback(self, msg):
        try:
            if msg.encoding == '16UC1':
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                self.latest_depth = depth.astype(np.float32) * self.depth_scale
            elif msg.encoding == '32FC1':
                self.latest_depth = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='32FC1').astype(np.float32)
            else:
                depth = self.bridge.imgmsg_to_cv2(msg)
                self.latest_depth = (
                    depth.astype(np.float32) * self.depth_scale
                    if depth.dtype == np.uint16 else depth.astype(np.float32))
        except Exception as exc:
            self.get_logger().warn(f'Depth conversion failed: {exc}')

    def rgb_callback(self, msg):
        now = time.monotonic()
        if self.max_fps > 0.0 and now - self.last_process_time < 1.0 / self.max_fps:
            return
        self.last_process_time = now

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'RGB conversion failed: {exc}')
            return

        candidate, mask = self.find_stop_line(image)
        distance = math.inf
        if candidate is not None:
            distance = self.distance_from_depth(candidate, image.shape[:2])
            if not math.isfinite(distance):
                distance = self.distance_from_image_row(candidate, image.shape[0])
            self.consecutive_detections += 1
        else:
            self.consecutive_detections = 0

        confirmed = self.consecutive_detections >= self.confirm_frames
        self.update_state(now, confirmed, distance)
        self.distance_pub.publish(Float32(data=(
            float(distance) if confirmed and math.isfinite(distance) else 0.0)))

        if self.publish_debug_image:
            debug = self.draw_debug(image, mask, candidate, distance, confirmed)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

    def find_stop_line(self, image):
        """Return the best road-line bounding box and its binary mask.

        Hue 5..25 in OpenCV HSV is orange/yellow-orange.  It excludes the
        world start/finish markings (bright yellow, about hue 30) while still
        tolerating Gazebo lighting on ``stop_line`` (RGB 1.0, 0.5, 0.0).
        """
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([5, 110, 100]), np.array([25, 255, 255]))

        top = int(np.clip(self.roi_top_ratio, 0.0, 0.95) * height)
        bottom = int(np.clip(self.roi_bottom_ratio, 0.05, 1.0) * height)
        roi_mask = np.zeros_like(mask)
        roi_mask[top:max(top + 1, bottom), :] = 255
        mask = cv2.bitwise_and(mask, roi_mask)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 3))
        small_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, small_kernel)

        best = None
        best_score = -math.inf
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_height <= 0:
                continue
            width_ratio = box_width / float(width)
            aspect_ratio = box_width / float(box_height)
            center_offset = abs((x + box_width / 2.0) - width / 2.0) / float(width)
            if (width_ratio < self.min_width_ratio or
                    aspect_ratio < self.min_aspect_ratio or
                    center_offset > self.center_tolerance_ratio):
                continue

            # Prefer a long marking that is close to the vehicle, without
            # accepting a yellow panel at the side of the road.
            score = width_ratio * 2.0 + (y + box_height) / float(height)
            if score > best_score:
                best_score = score
                best = (x, y, box_width, box_height)

        return best, mask

    def distance_from_depth(self, candidate, image_shape):
        """Use only depth pixels inside the yellow contour's bounding box."""
        if self.latest_depth is None:
            return math.inf

        image_height, image_width = image_shape
        depth = self.latest_depth
        if depth.shape[:2] != (image_height, image_width):
            depth = cv2.resize(depth, (image_width, image_height),
                               interpolation=cv2.INTER_NEAREST)

        x, y, width, height = candidate
        # The line itself is thin.  Sampling the middle third avoids mixing it
        # with the road too far before/after the line while retaining enough
        # valid depth pixels when Gazebo anti-aliases its edge.
        x0 = max(0, x + width // 6)
        x1 = min(image_width, x + 5 * width // 6)
        y0 = max(0, y + height // 4)
        y1 = min(image_height, y + 3 * height // 4 + 1)
        values = depth[y0:y1, x0:x1].reshape(-1)
        values = values[np.isfinite(values)]
        values = values[(values >= self.min_valid_distance) &
                        (values <= self.max_valid_distance)]
        return float(np.median(values)) if values.size >= 5 else math.inf

    def distance_from_image_row(self, candidate, image_height):
        """Conservative depth fallback if the OAK depth stream is unavailable."""
        _, y, _, height = candidate
        row_ratio = min(1.0, max(0.0, (y + height) / float(image_height)))
        # A lower line is nearer.  This is only an envelope fallback; normal
        # operation uses the measured depth above.
        return self.stop_distance + 5.0 * (1.0 - row_ratio)

    def update_state(self, now, detected, distance):
        if self.state == 'DRIVING':
            if detected and math.isfinite(distance) and distance <= self.stop_distance:
                self.state = 'HOLD'
                self.stop_started_at = now
                self.clear_started_at = None
                self.get_logger().warn(
                    f'Yellow stop line at {distance:.2f} m: holding for '
                    f'{self.stop_duration:.1f} s')
            return

        if self.state == 'HOLD':
            if now - self.stop_started_at >= self.stop_duration:
                self.state = 'WAIT_CLEAR'
                self.clear_started_at = None
                self.get_logger().info('Stop-line hold complete; waiting for line to clear')
            return

        # Do not re-trigger while the car is still over the same painted line.
        clear = not detected
        if clear:
            if self.clear_started_at is None:
                self.clear_started_at = now
            elif now - self.clear_started_at >= self.rearm_clear_duration:
                self.state = 'DRIVING'
                self.clear_started_at = None
                self.get_logger().info('Yellow stop-line detector re-armed')
        else:
            self.clear_started_at = None

    def publish_state(self):
        self.stop_pub.publish(Bool(data=self.state == 'HOLD'))
        # Braking only occurs while a confirmed yellow line was just processed.
        # The distance message is zero when no line is available, so the gate
        # safely passes normal lane-following commands unchanged.
        self.brake_pub.publish(Bool(data=self.state == 'DRIVING' and
                                    self.consecutive_detections >= self.confirm_frames))

    def draw_debug(self, image, mask, candidate, distance, confirmed):
        debug = image.copy()
        overlay = np.zeros_like(debug)
        overlay[mask > 0] = (0, 220, 255)
        active = mask > 0
        debug[active] = cv2.addWeighted(debug, 0.55, overlay, 0.45, 0)[active]

        if candidate is not None:
            x, y, width, height = candidate
            color = (0, 0, 255) if confirmed else (0, 180, 255)
            cv2.rectangle(debug, (x, y), (x + width, y + height), color, 3)
            distance_text = f'{distance:.2f} m' if math.isfinite(distance) else 'distance inconnue'
            cv2.putText(debug, f'LIGNE STOP JAUNE  {distance_text}',
                        (max(10, x), max(28, y - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, color, 2)

        status = {
            'DRIVING': 'RECHERCHE LIGNE STOP JAUNE',
            'HOLD': 'ARRET SUR LIGNE JAUNE',
            'WAIT_CLEAR': 'LIGNE PASSEE - ATTENTE DEGAGEMENT',
        }[self.state]
        cv2.rectangle(debug, (10, 10), (630, 52), (24, 24, 24), -1)
        cv2.putText(debug, status, (20, 38), cv2.FONT_HERSHEY_SIMPLEX,
                    0.72, (0, 220, 255) if self.state == 'DRIVING' else (0, 0, 255), 2)
        return debug


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YellowStopLineDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
