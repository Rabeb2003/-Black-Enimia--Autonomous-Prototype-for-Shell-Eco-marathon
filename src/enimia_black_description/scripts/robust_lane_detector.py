#!/usr/bin/env python3
"""
Robust Lane & White Road Line Segmentation for Autonomous Racing Simulation (ROS 2 Humble).
Designed specifically for circuit tracks (e.g., Silesia Ring) in Gazebo.

Features:
- Continuous white line segmentation (dashed center line and solid side borders).
- Dual-cue perception: Adaptive white paint segmentation + asphalt-to-grass/kerb corridor detection.
- Multi-row lookahead scanning (9 scan lines, deep lookahead to 15-20m) for early curve anticipation.
- Instant steering response (high smoothing alpha for rapid curve turn-in).
- Rich visual HUD with segmented white lines overlay and drivable corridor polygon.
- Publishes segmented debug image (/lane_detection/output) and raw segmentation mask (/lane_detection/mask).
"""

import math
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
        self.declare_parameter('mask_image_topic', '/lane_detection/mask')
        self.declare_parameter('error_topic', '/lane_detection/center_error')
        self.declare_parameter('heading_error_topic', '/lane_detection/heading_error')
        self.declare_parameter('valid_topic', '/lane_detection/valid')
        self.declare_parameter('max_fps', 30.0)
        self.declare_parameter('roi_top_ratio', 0.25)
        self.declare_parameter('roi_bottom_ratio', 0.90)
        self.declare_parameter('smoothing_alpha', 0.75)
        self.declare_parameter('white_intensity_thresh', 160)
        self.declare_parameter('white_max_sat', 70)

        self.image_topic = self.get_parameter('image_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        self.mask_image_topic = self.get_parameter('mask_image_topic').value
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
        self.mask_pub = self.create_publisher(Image, self.mask_image_topic, 10)

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
        self.last_valid_center = None
        self.last_valid_time = 0.0
        self.dropout_hold_duration = 0.8

        self.get_logger().info(
            f'Robust Lane & White Line Segmentation initialized on {self.image_topic}'
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
        valid, lat_error, heading_error, debug_img, seg_mask = self.process_frame(cv_img, h, w, now)

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

        # Publish raw segmentation mask
        if seg_mask is not None:
            try:
                mask_msg = self.bridge.cv2_to_imgmsg(seg_mask, encoding='mono8')
                self.mask_pub.publish(mask_msg)
            except Exception as e:
                self.get_logger().debug(f'mask publish error: {e}')

    def process_frame(self, img, h, w, now):
        """Segment white road lines and calculate track center & heading error."""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 1. White paint segmentation (center dashed line and side borders)
        white_mask = (gray >= self.white_intensity_thresh) & (hsv[:, :, 1] <= self.white_max_sat)

        # 2. Grass segmentation (Green area off the track)
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

        # Filter out grass and kerbs from white lines
        white_mask = white_mask & (~grass_mask) & (~kerb_mask)

        # Morphological noise cleaning on white line segmentation mask
        white_u8 = (white_mask.astype(np.uint8) * 255)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        clean_white_mask = cv2.morphologyEx(white_u8, cv2.MORPH_OPEN, kernel)

        # Deep lookahead: scan across 9 rows for anticipatory curve detection
        y_top = int(h * self.roi_top_ratio)
        y_bottom = int(h * self.roi_bottom_ratio)
        scan_rows = np.linspace(y_top, y_bottom, 9).astype(int)

        slice_centers = []
        slice_widths = []
        debug_lines = []

        half_w = w // 2
        # Track the road from the bumper towards the horizon.  In the former
        # far-to-near pass every row was compared with the *straight* near
        # reference.  At the first right-hand bend this could select a distant
        # road/grass corridor instead of the white right border that continues
        # the current lane.  Updating the reference row by row makes the
        # preview follow the visible curve continuously.
        tracked_center = (
            self.last_valid_center if self.last_valid_center is not None
            else float(half_w)
        )

        for y in reversed(scan_rows):
            row_grass = (grass_mask[y, :] | kerb_mask[y, :]).astype(np.uint8)
            row_white = clean_white_mask[y, :] > 0

            # Find all continuous road intervals (non-grass / non-kerb)
            road_mask_row = (row_grass == 0).astype(np.uint8)
            diff = np.diff(np.pad(road_mask_row, (1, 1), 'constant', constant_values=0).astype(np.int32))
            starts = np.where(diff == 1)[0]
            ends = np.where(diff == -1)[0]

            best_left = 0
            best_right = w - 1
            min_dist = math.inf
            found_corridor = False

            for s, e in zip(starts, ends):
                width = e - s
                if width > int(w * 0.12):
                    mid = (s + e) / 2.0
                    dist = abs(mid - tracked_center)
                    if dist < min_dist:
                        min_dist = dist
                        best_left = int(s)
                        best_right = int(e)
                        found_corridor = True

            left_border = best_left
            right_border = best_right
            road_width = right_border - left_border

            # Segment white line markings in this corridor
            road_white_indices = np.where(row_white[left_border:right_border])[0]
            center_white = None
            if len(road_white_indices) > 0:
                white_cols = road_white_indices + left_border
                corridor_center = (left_border + right_border) / 2.0
                corridor_half = max(road_width * 0.30, 20.0)
                mid_white = white_cols[np.abs(white_cols - corridor_center) < corridor_half]
                if len(mid_white) > 0:
                    center_white = float(np.median(mid_white))

            # Determine track center for this horizontal row
            row_center = None
            if found_corridor and (road_width < (w - 10)):
                if center_white is not None:
                    row_center = 0.60 * center_white + 0.40 * ((left_border + right_border) / 2.0)
                else:
                    row_center = (left_border + right_border) / 2.0
                slice_widths.append(road_width)
            elif center_white is not None:
                row_center = center_white
            elif self.last_track_width_px is not None:
                if left_border > 10 and right_border >= (w - 10):
                    row_center = left_border + self.last_track_width_px / 2.0
                elif right_border < (w - 10) and left_border <= 10:
                    row_center = right_border - self.last_track_width_px / 2.0

            if row_center is not None:
                row_center = float(np.clip(row_center, 0.0, float(w - 1)))
                slice_centers.append((y, row_center))
                # Keep some inertia so segmentation speckles cannot make the
                # preview jump across the circuit, while allowing enough turn
                # entry to follow the right white line in the first bend.
                tracked_center = 0.65 * row_center + 0.35 * tracked_center

            debug_lines.append((y, left_border, right_border, row_center, center_white))

        # Update track width memory
        if len(slice_widths) > 0:
            avg_width = float(np.median(slice_widths))
            if self.last_track_width_px is None:
                self.last_track_width_px = avg_width
            else:
                self.last_track_width_px = 0.2 * avg_width + 0.8 * self.last_track_width_px

        valid = False
        raw_error = 0.0
        raw_heading = 0.0

        if len(slice_centers) >= 2:
            valid = True
            self.last_valid_time = now

            # Near-field center (lower 4 slices)
            near_candidates = [c for (y, c) in slice_centers if y >= scan_rows[len(scan_rows)//2]]
            center_near = float(np.median(near_candidates)) if near_candidates else float(slice_centers[-1][1])

            # Far-field center (upper 4 slices - preview deep into bend)
            far_candidates = [c for (y, c) in slice_centers if y <= scan_rows[len(scan_rows)//2]]
            center_far = float(np.median(far_candidates)) if far_candidates else center_near

            self.last_valid_center = center_near

            # Normalized error: > 0 means car must steer right, < 0 means left
            raw_error = (center_near - half_w) / float(half_w)
            raw_heading = (center_far - center_near) / float(half_w)

            # Responsive dynamic alpha: higher alpha when entering sharp bends
            effective_alpha = self.smoothing_alpha
            if abs(raw_heading) > 0.15 or abs(raw_error) > 0.20:
                effective_alpha = min(0.90, self.smoothing_alpha * 1.25)

            self.filtered_error = (
                effective_alpha * raw_error +
                (1.0 - effective_alpha) * self.filtered_error
            )
            self.filtered_heading = (
                effective_alpha * raw_heading +
                (1.0 - effective_alpha) * self.filtered_heading
            )
        else:
            time_since_valid = now - self.last_valid_time
            if time_since_valid < self.dropout_hold_duration:
                valid = True
                self.filtered_error *= 0.95
                self.filtered_heading *= 0.95
            else:
                valid = False

        # Build debug image with continuous segmented white lines overlay
        debug_img = self.draw_debug_overlay(
            img, clean_white_mask, debug_lines, slice_centers,
            y_top, y_bottom, valid, self.filtered_error, self.filtered_heading
        )

        return valid, self.filtered_error, self.filtered_heading, debug_img, clean_white_mask

    def draw_debug_overlay(self, img, clean_white_mask, debug_lines, slice_centers,
                           y_top, y_bottom, valid, lat_err, head_err):
        """Draw aesthetic continuous white line segmentation and driving corridor HUD."""
        debug = img.copy()
        h, w = debug.shape[:2]
        half_w = w // 2

        # 1. Overlay segmented white road lines in glowing bright green
        # Only in the active road region
        roi_mask = np.zeros((h, w), dtype=np.uint8)
        roi_mask[y_top:y_bottom, :] = 255
        active_white = cv2.bitwise_and(clean_white_mask, roi_mask)

        # Create colored overlay for segmented white lines
        seg_color_layer = np.zeros_like(debug)
        # Lime-green for white lines: (30, 255, 120)
        seg_color_layer[active_white > 0] = (30, 255, 120)

        # 2. Draw drivable track corridor polygon if borders available
        left_pts = [(lb, y) for (y, lb, rb, rc, cw) in debug_lines if lb > 5]
        right_pts = [(rb, y) for (y, lb, rb, rc, cw) in reversed(debug_lines) if rb < (w - 5)]
        if len(left_pts) >= 2 and len(right_pts) >= 2:
            corridor_poly = np.array(left_pts + right_pts, dtype=np.int32)
            cv2.fillPoly(seg_color_layer, [corridor_poly], (255, 160, 0))  # Translucent cyan/blue corridor

        # Alpha blend the segmented layers onto debug image (35% opacity)
        mask_any = (seg_color_layer > 0).any(axis=2)
        debug[mask_any] = cv2.addWeighted(debug, 0.65, seg_color_layer, 0.35, 0)[mask_any]

        # 3. Draw contour outlines around segmented white road markings for crisp visualization
        contours, _ = cv2.findContours(active_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, (0, 255, 200), 1)

        # 4. Draw track trajectory curve from slice centers
        if len(slice_centers) >= 2:
            pts = np.array([[int(cx), int(cy)] for (cy, cx) in slice_centers], dtype=np.int32)
            cv2.polylines(debug, [pts], isClosed=False, color=(0, 255, 255), thickness=3)

        # Draw vehicle bumper centerline in cyan
        cv2.line(debug, (half_w, y_top), (half_w, y_bottom + 30), (255, 255, 0), 2)

        # Draw detected borders and centers for each scan row
        for y, lb, rb, rc, cw in debug_lines:
            if lb > 5:
                cv2.circle(debug, (lb, y), 5, (255, 120, 0), -1)
            if rb < (w - 5):
                cv2.circle(debug, (rb, y), 5, (255, 120, 0), -1)
            if cw is not None:
                cv2.circle(debug, (int(cw), y), 4, (255, 255, 255), -1)
            if rc is not None:
                cv2.circle(debug, (int(rc), y), 6, (0, 255, 255), -1)

        # Draw steering arrow from bumper center towards target track path
        arrow_color = (0, 255, 0) if valid else (0, 0, 255)
        arrow_end_x = int(half_w + np.clip(lat_err * 220, -250, 250))
        cv2.arrowedLine(
            debug,
            (half_w, y_bottom + 25),
            (arrow_end_x, y_bottom + 25),
            arrow_color,
            3,
            tipLength=0.25,
        )

        # HUD Text Panel (Semi-transparent dark backdrop)
        overlay = debug.copy()
        cv2.rectangle(overlay, (10, 10), (450, 115), (15, 15, 20), -1)
        cv2.addWeighted(overlay, 0.75, debug, 0.25, 0, debug)

        status_text = "LIGNES BLANCHES : SEGMENTEES [OK]" if valid else "LIGNES BLANCHES : RECHERCHE..."
        status_color = (0, 255, 120) if valid else (0, 165, 255)
        cv2.putText(debug, status_text, (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
        cv2.putText(debug, f"ERREUR LATERALE : {lat_err:+.3f}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
        cv2.putText(debug, f"ERREUR CAP/VIRAGE: {head_err:+.3f}", (20, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (220, 220, 220), 2)
        turn_status = "VIRAGE DROITE" if lat_err > 0.10 else ("VIRAGE GAUCHE" if lat_err < -0.10 else "LIGNE DROITE")
        cv2.putText(debug, f"ETAT ROUTE : {turn_status}", (20, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

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
