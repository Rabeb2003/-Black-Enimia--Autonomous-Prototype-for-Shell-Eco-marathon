#!/usr/bin/env python3
"""
Robust Lane and Track Center Detection for Autonomous Racing Simulation (ROS 2 Humble).
Designed specifically for circuit tracks (e.g., Silesia Ring) in Gazebo.

Features:
- Dual-cue perception: White paint markings + Asphalt-to-Grass / Kerb boundary segmentation.
- Multi-row lookahead scanning: Computes both lateral cross-track error (near field)
  and anticipatory road curvature / heading error (mid/far field).
- Temporal exponential filter to bridge dashed center-line gaps seamlessly.
- Uses `qos_profile_sensor_data` (BEST_EFFORT) for reliable communication with Gazebo cameras.
- Comprehensive RViz debug visualization image output.
"""

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class RobustLaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        # Parameters
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('output_image_topic', '/lane_detection/output')
        self.declare_parameter('error_topic', '/lane_detection/center_error')
        self.declare_parameter('heading_error_topic', '/lane_detection/heading_error')
        self.declare_parameter('valid_topic', '/lane_detection/valid')
        self.declare_parameter('max_fps', 30.0)
        self.declare_parameter('roi_top_ratio', 0.45)
        self.declare_parameter('roi_bottom_ratio', 0.90)
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('white_intensity_thresh', 165)
        self.declare_parameter('white_max_sat', 65)

        self.image_topic = self.get_parameter('image_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        self.error_topic = self.get_parameter('error_topic').value
        self.heading_error_topic = self.get_parameter('heading_error_topic').value
        self.valid_topic = self.get_parameter('valid_topic').value
        self.max_fps = float(self.get_parameter('max_fps').value)
        self.roi_top_ratio = float(self.get_parameter('roi_top_ratio').value)
        self.roi_bottom_ratio = float(self.get_parameter('roi_bottom_ratio').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.white_intensity_thresh = int(self.get_parameter('white_intensity_thresh').value)
        self.white_max_sat = int(self.get_parameter('white_max_sat').value)

        # CV Bridge
        self.bridge = CvBridge()

        # Publishers
        self.error_pub = self.create_publisher(Float32, self.error_topic, 10)
        self.heading_pub = self.create_publisher(Float32, self.heading_error_topic, 10)
        self.valid_pub = self.create_publisher(Bool, self.valid_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.output_image_topic, 10)

        # Gazebo camera plugin uses BEST_EFFORT QoS
        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # Internal state
        self.last_process_time = 0.0
        self.filtered_error = 0.0
        self.filtered_heading = 0.0
        self.last_track_width_px = None
        self.last_valid_time = 0.0
        self.dropout_hold_duration = 0.8  # seconds to hold track center across gaps

        self.get_logger().info(
            f'Robust Lane Detector initialized on {self.image_topic} (BEST_EFFORT QoS)'
        )

    def image_callback(self, msg: Image):
        now = time.monotonic()
        if self.max_fps > 0.0 and (now - self.last_process_time) < (1.0 / self.max_fps):
            return
        self.last_process_time = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        h, w = cv_img.shape[:2]
        valid, lat_error, heading_error, debug_img = self.process_frame(cv_img, h, w, now)

        # Publish results
        self.error_pub.publish(Float32(data=float(lat_error)))
        self.heading_pub.publish(Float32(data=float(heading_error)))
        self.valid_pub.publish(Bool(data=bool(valid)))

        # Publish debug visualization
        if debug_img is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().debug(f'debug image publish error: {e}')

    def process_frame(self, img, h, w, now):
        """Analyze road markings and track boundaries to estimate center error."""
        # Convert color spaces
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 1. White paint segmentation (center dashed line and side borders)
        white_mask = (gray >= self.white_intensity_thresh) & (hsv[:, :, 1] <= self.white_max_sat)

        # 2. Grass segmentation (Green area off the track)
        # Gazebo Grass: H in [32, 90], S >= 35, V >= 35
        grass_mask = (
            (hsv[:, :, 0] >= 30) & (hsv[:, :, 0] <= 90) &
            (hsv[:, :, 1] >= 35) &
            (hsv[:, :, 2] >= 35)
        )

        # 3. Red kerb segmentation (corner apexes)
        kerb_mask = (
            ((hsv[:, :, 0] <= 12) | (hsv[:, :, 0] >= 168)) &
            (hsv[:, :, 1] >= 50) &
            (hsv[:, :, 2] >= 50)
        )

        # Scan across 5 vertical slices inside the ROI
        y_top = int(h * self.roi_top_ratio)
        y_bottom = int(h * self.roi_bottom_ratio)
        scan_rows = np.linspace(y_top, y_bottom, 5).astype(int)

        slice_centers = []
        slice_widths = []
        debug_lines = []

        half_w = w // 2

        for y in scan_rows:
            row_grass = grass_mask[y, :] | kerb_mask[y, :]
            row_white = white_mask[y, :]

            # Find left road border (boundary where left grass/kerb ends)
            left_grass_indices = np.where(row_grass[:half_w])[0]
            left_border = int(left_grass_indices.max()) if len(left_grass_indices) > 0 else 0

            # Find right road border (boundary where right grass/kerb begins)
            right_grass_indices = np.where(row_grass[half_w:])[0]
            right_border = int(right_grass_indices.min() + half_w) if len(right_grass_indices) > 0 else (w - 1)

            # Find white line detections inside the detected road corridor
            road_white_indices = np.where(row_white[left_border:right_border])[0]
            center_white = None
            if len(road_white_indices) > 0:
                white_cols = road_white_indices + left_border
                # Check for center line candidates near the middle
                corridor_center = (left_border + right_border) / 2.0
                corridor_half = max((right_border - left_border) * 0.25, 20.0)
                mid_white = white_cols[np.abs(white_cols - corridor_center) < corridor_half]
                if len(mid_white) > 0:
                    center_white = float(np.median(mid_white))

            # Determine track center for this horizontal row
            row_center = None
            road_width = right_border - left_border

            has_left_grass = (left_border > 5)
            has_right_grass = (right_border < (w - 5))

            if has_left_grass and has_right_grass and (road_width > w * 0.25):
                # Both edges clearly visible
                row_center = (left_border + right_border) / 2.0
                slice_widths.append(road_width)
            elif center_white is not None:
                # Center dashed line clearly visible
                row_center = center_white
                if has_left_grass:
                    slice_widths.append((center_white - left_border) * 2.0)
                elif has_right_grass:
                    slice_widths.append((right_border - center_white) * 2.0)
            elif has_left_grass and self.last_track_width_px is not None:
                # Drifting left: only left border visible, estimate center using remembered width
                row_center = left_border + self.last_track_width_px / 2.0
            elif has_right_grass and self.last_track_width_px is not None:
                # Drifting right: only right border visible, estimate center using remembered width
                row_center = right_border - self.last_track_width_px / 2.0
            elif (left_border == 0 and right_border == w - 1) and self.last_track_width_px is not None:
                # Fully on track with wide corridor; hold last center
                pass

            if row_center is not None:
                row_center = float(np.clip(row_center, 0.0, float(w - 1)))
                slice_centers.append((y, row_center))

            debug_lines.append((y, left_border, right_border, row_center, center_white))

        # Update remembered track width
        if len(slice_widths) > 0:
            avg_width = float(np.median(slice_widths))
            if self.last_track_width_px is None:
                self.last_track_width_px = avg_width
            else:
                self.last_track_width_px = 0.2 * avg_width + 0.8 * self.last_track_width_px

        # Compute near-field center and far lookahead center
        valid = False
        raw_error = 0.0
        raw_heading = 0.0

        if len(slice_centers) >= 1:
            valid = True
            self.last_valid_time = now

            # Near center (lower slices)
            near_candidates = [c for (y, c) in slice_centers if y >= scan_rows[2]]
            if len(near_candidates) > 0:
                center_near = float(np.median(near_candidates))
            else:
                center_near = float(slice_centers[-1][1])

            # Far center (upper slices)
            far_candidates = [c for (y, c) in slice_centers if y <= scan_rows[2]]
            if len(far_candidates) > 0:
                center_far = float(np.median(far_candidates))
            else:
                center_far = center_near

            # Normalized lateral error in [-1, 1]:
            # > 0 means track center is to the RIGHT of car -> car must steer RIGHT
            # < 0 means track center is to the LEFT of car -> car must steer LEFT
            raw_error = (center_near - half_w) / float(half_w)
            raw_heading = (center_far - center_near) / float(half_w)

            # Exponential smoothing
            self.filtered_error = (
                self.smoothing_alpha * raw_error +
                (1.0 - self.smoothing_alpha) * self.filtered_error
            )
            self.filtered_heading = (
                self.smoothing_alpha * raw_heading +
                (1.0 - self.smoothing_alpha) * self.filtered_heading
            )
        else:
            # Brief dropout (e.g. crossing gap in dashed line with open horizon)
            time_since_valid = now - self.last_valid_time
            if time_since_valid < self.dropout_hold_duration:
                valid = True
                # Slowly decay error back towards 0
                self.filtered_error *= 0.95
                self.filtered_heading *= 0.95
            else:
                valid = False

        # Build debug image
        debug_img = self.draw_debug_overlay(img, debug_lines, y_top, y_bottom, valid, self.filtered_error, self.filtered_heading)

        return valid, self.filtered_error, self.filtered_heading, debug_img

    def draw_debug_overlay(self, img, debug_lines, y_top, y_bottom, valid, lat_err, head_err):
        """Draw a clear visual HUD for RViz."""
        debug = img.copy()
        h, w = debug.shape[:2]
        half_w = w // 2

        # Draw ROI boundary
        cv2.rectangle(debug, (10, y_top), (w - 10, y_bottom), (60, 60, 60), 1)

        # Draw vehicle bumper centerline in cyan
        cv2.line(debug, (half_w, y_top), (half_w, y_bottom + 30), (255, 255, 0), 2)

        # Draw detected borders and centers for each scan row
        for y, lb, rb, rc, cw in debug_lines:
            # Left border in bright blue
            if lb > 5:
                cv2.circle(debug, (lb, y), 5, (255, 120, 0), -1)
            # Right border in bright blue
            if rb < (w - 5):
                cv2.circle(debug, (rb, y), 5, (255, 120, 0), -1)
            # Center dashed line in white
            if cw is not None:
                cv2.circle(debug, (int(cw), y), 4, (255, 255, 255), -1)
            # Computed road center in bright yellow
            if rc is not None:
                cv2.circle(debug, (int(rc), y), 6, (0, 255, 255), -1)

        # Draw target track center line (interpolated)
        target_x = int(half_w + lat_err * half_w)
        cv2.line(debug, (target_x, y_top), (target_x, y_bottom), (0, 255, 255), 3)

        # Draw steering arrow from bumper center towards target center
        arrow_color = (0, 255, 0) if valid else (0, 0, 255)
        arrow_end_x = int(half_w + np.clip(lat_err * 200, -250, 250))
        cv2.arrowedLine(
            debug,
            (half_w, y_bottom + 25),
            (arrow_end_x, y_bottom + 25),
            arrow_color,
            3,
            tipLength=0.25,
        )

        # HUD Text Panel
        overlay = debug.copy()
        cv2.rectangle(overlay, (10, 10), (380, 100), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.7, debug, 0.3, 0, debug)

        status_text = "TRACK: LOCKED" if valid else "TRACK: SEARCHING"
        status_color = (0, 255, 0) if valid else (0, 165, 255)
        cv2.putText(debug, status_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2)
        cv2.putText(debug, f"LAT ERROR : {lat_err:+.3f}", (20, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(debug, f"HEADING   : {head_err:+.3f}", (20, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)

        return debug


def main(args=None):
    rclpy.init(args=args)
    node = RobustLaneDetector()
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
