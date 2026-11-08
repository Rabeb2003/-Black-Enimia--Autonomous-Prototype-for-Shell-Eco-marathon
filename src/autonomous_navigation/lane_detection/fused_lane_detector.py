#!/usr/bin/env python3
"""
Fused Hybrid Lane & Curve Detector for ENIMIA Black Autonomous Racing.
Tuned for Perfect Hairpin Negotiation:
1. Deep lookahead (roi_top_ratio=0.32 to 0.90):
   Captures curve arc smoothly and measures true track curvature.
2. Robust corridor segmentation (Asphalt vs Green Grass vs Red Kerb Apex).
3. Anti-grass & Anti-barrier Corridor Guard:
   Strong immediate repulsive centering when vehicle approaches right or left road borders.
4. Curvature & Heading estimation with dynamic smoothing.
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


class FusedLaneDetector(Node):
    def __init__(self):
        super().__init__('fused_lane_detector')

        # Parameters
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('output_image_topic', '/lane_detection/output')
        self.declare_parameter('error_topic', '/lane_detection/center_error')
        self.declare_parameter('heading_error_topic', '/lane_detection/heading_error')
        self.declare_parameter('curvature_topic', '/lane_detection/curvature')
        self.declare_parameter('valid_topic', '/lane_detection/valid')
        self.declare_parameter('max_fps', 30.0)
        self.declare_parameter('roi_top_ratio', 0.32)        # Deep preview to see curve arc smoothly
        self.declare_parameter('roi_bottom_ratio', 0.90)     # Near front bumper
        self.declare_parameter('smoothing_alpha', 0.75)
        self.declare_parameter('white_intensity_thresh', 165)
        self.declare_parameter('white_max_sat', 65)

        self.image_topic = self.get_parameter('image_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        self.error_topic = self.get_parameter('error_topic').value
        self.heading_error_topic = self.get_parameter('heading_error_topic').value
        self.curvature_topic = self.get_parameter('curvature_topic').value
        self.valid_topic = self.get_parameter('valid_topic').value
        self.max_fps = float(self.get_parameter('max_fps').value)
        self.roi_top_ratio = float(self.get_parameter('roi_top_ratio').value)
        self.roi_bottom_ratio = float(self.get_parameter('roi_bottom_ratio').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.white_intensity_thresh = int(self.get_parameter('white_intensity_thresh').value)
        self.white_max_sat = int(self.get_parameter('white_max_sat').value)

        self.bridge = CvBridge()

        # Publishers
        self.error_pub = self.create_publisher(Float32, self.error_topic, 10)
        self.heading_pub = self.create_publisher(Float32, self.heading_error_topic, 10)
        self.curvature_pub = self.create_publisher(Float32, self.curvature_topic, 10)
        self.valid_pub = self.create_publisher(Bool, self.valid_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.output_image_topic, 10)

        # Subscription to camera
        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # Internal state tracking
        self.last_process_time = 0.0
        self.filtered_error = 0.0
        self.filtered_heading = 0.0
        self.filtered_curvature = 0.0
        self.last_valid_time = 0.0
        self.dropout_hold_duration = 0.8
        self.last_valid_center = None

        self.get_logger().info(
            f'Fused Lane & Curve Detector initialized on {self.image_topic} (FPS: {self.max_fps})'
        )

    def image_callback(self, msg: Image):
        now = time.monotonic()
        if self.max_fps > 0.0 and (now - self.last_process_time) < (1.0 / self.max_fps):
            return
        self.last_process_time = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion error: {e}')
            return

        h, w = cv_img.shape[:2]
        valid, lat_error, heading_error, curvature, debug_img = self.process_frame(cv_img, h, w, now)

        # Publish telemetry
        self.error_pub.publish(Float32(data=float(lat_error)))
        self.heading_pub.publish(Float32(data=float(heading_error)))
        self.curvature_pub.publish(Float32(data=float(curvature)))
        self.valid_pub.publish(Bool(data=bool(valid)))

        # Publish visual HUD
        if debug_img is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().debug(f'debug image publish error: {e}')

    def process_frame(self, img, h, w, now):
        """
        Processes frame using multi-cue segmentation, bottom-to-top perspective tracking,
        and corridor boundary guards.
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 1. White paint segmentation
        white_mask = (gray >= self.white_intensity_thresh) & (hsv[:, :, 1] <= self.white_max_sat)

        # 2. Grass segmentation
        grass_mask = (
            (hsv[:, :, 0] >= 30) & (hsv[:, :, 0] <= 90) &
            (hsv[:, :, 1] >= 35) &
            (hsv[:, :, 2] >= 35)
        )

        # 3. Kerb segmentation (red/white rumble strips)
        kerb_mask = (
            ((hsv[:, :, 0] <= 12) | (hsv[:, :, 0] >= 168)) &
            (hsv[:, :, 1] >= 50) &
            (hsv[:, :, 2] >= 50)
        )

        # Clean white line mask
        clean_white_mask = white_mask & (~grass_mask) & (~kerb_mask)
        white_u8 = (clean_white_mask.astype(np.uint8) * 255)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        clean_white_mask = cv2.morphologyEx(white_u8, cv2.MORPH_OPEN, kernel)

        # Bottom-to-top scanning: 9 scan lines
        y_top = int(h * self.roi_top_ratio)
        y_bottom = int(h * self.roi_bottom_ratio)
        scan_rows = np.linspace(y_top, y_bottom, 9).astype(int)

        slice_centers = []
        debug_slices = []

        half_w = w // 2
        tracked_center = self.last_valid_center if self.last_valid_center is not None else float(half_w)

        # Calibrated track half-width function for 12m Silesia Ring track (at 1280x720)
        def expected_half_width(y_row):
            ratio = (y_row - y_top) / max(1.0, float(y_bottom - y_top))
            # Perspective scaling: 350px far field (~18m) to 1100px near bumper (~3.5m)
            return 350.0 + 750.0 * ratio

        right_border_near = None
        left_border_near = None

        # Scan from near to far (bottom to top)
        for y in reversed(scan_rows):
            row_grass = (grass_mask[y, :] | kerb_mask[y, :]).astype(np.uint8)
            row_white = clean_white_mask[y, :] > 0

            # Continuous asphalt intervals
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

            # Record near-field borders
            if y >= scan_rows[-2]:
                if right_border < (w - 10):
                    right_border_near = right_border
                if left_border > 10:
                    left_border_near = left_border

            # Extract white lines
            white_indices = np.where(row_white)[0]
            left_white = None
            center_white = None
            right_white = None

            if len(white_indices) > 0:
                diff_w = np.diff(white_indices)
                split_points = np.where(diff_w > 8)[0] + 1
                white_clusters = np.split(white_indices, split_points)

                for cluster in white_clusters:
                    if len(cluster) >= 2:
                        cx = float(np.mean(cluster))
                        hw = expected_half_width(y)
                        
                        # Silesia Ring: dashed centerline is near track center (+- 0.30 * hw or +- 200px)
                        dist_from_tracked = abs(cx - tracked_center)
                        if dist_from_tracked < max(180.0, hw * 0.30):
                            if center_white is None or dist_from_tracked < abs(center_white - tracked_center):
                                center_white = cx
                        elif cx < (tracked_center - max(200.0, hw * 0.45)):
                            if left_white is None or dist_from_tracked < abs(left_white - tracked_center):
                                left_white = cx
                        elif cx > (tracked_center + max(200.0, hw * 0.45)):
                            if right_white is None or dist_from_tracked < abs(right_white - tracked_center):
                                right_white = cx

            eff_left = left_white if left_white is not None else (left_border if left_border > 12 else None)
            eff_right = right_white if right_white is not None else (right_border if right_border < (w - 12) else None)

            # Determine row target center
            row_center = None
            hw = expected_half_width(y)

            # Center dashed line has highest authority if present
            if center_white is not None:
                if eff_left is not None and eff_right is not None:
                    # Blend center dashed line with road boundaries
                    road_mid = (eff_left + eff_right) / 2.0
                    row_center = 0.70 * center_white + 0.30 * road_mid
                else:
                    row_center = center_white
            elif eff_left is not None and eff_right is not None:
                row_center = (eff_left + eff_right) / 2.0
            elif eff_left is not None:
                row_center = eff_left + hw
            elif eff_right is not None:
                row_center = eff_right - hw
            elif found_corridor:
                row_center = (left_border + right_border) / 2.0

            if row_center is not None:
                row_center = float(np.clip(row_center, 25.0, float(w - 25)))
                slice_centers.append((y, row_center))
                tracked_center = 0.65 * row_center + 0.35 * tracked_center

            debug_slices.append((y, left_border, right_border, row_center, center_white))

        valid = False
        raw_error = 0.0
        raw_heading = 0.0
        calculated_curvature = 0.0

        if len(slice_centers) >= 2:
            valid = True
            self.last_valid_time = now

            # Near-field center (lower half)
            near_candidates = [c for (y, c) in slice_centers if y >= scan_rows[len(scan_rows)//2]]
            center_near = float(np.median(near_candidates)) if near_candidates else float(slice_centers[0][1])

            # Far-field center (upper half - captures hairpin arc smoothly)
            far_candidates = [c for (y, c) in slice_centers if y <= scan_rows[len(scan_rows)//2]]
            center_far = float(np.median(far_candidates)) if far_candidates else center_near

            self.last_valid_center = center_near

            raw_error = (center_near - half_w) / float(half_w)
            raw_heading = (center_far - center_near) / float(half_w)

            # Anti-grass / anti-barrier guard in near field
            if right_border_near is not None:
                right_clearance = right_border_near - half_w
                safe_margin = 150.0
                if right_clearance < safe_margin:
                    guard_bias = -0.40 * ((safe_margin - right_clearance) / safe_margin)
                    raw_error = min(raw_error, guard_bias)

            if left_border_near is not None:
                left_clearance = half_w - left_border_near
                safe_margin = 150.0
                if left_clearance < safe_margin:
                    guard_bias = +0.40 * ((safe_margin - left_clearance) / safe_margin)
                    raw_error = max(raw_error, guard_bias)

            calculated_curvature = float(np.clip(raw_heading * 0.40, -0.40, 0.40))

            alpha = self.smoothing_alpha
            self.filtered_error = alpha * raw_error + (1.0 - alpha) * self.filtered_error
            self.filtered_heading = alpha * raw_heading + (1.0 - alpha) * self.filtered_heading
            self.filtered_curvature = alpha * calculated_curvature + (1.0 - alpha) * self.filtered_curvature

        else:
            time_since_valid = now - self.last_valid_time
            if time_since_valid < self.dropout_hold_duration:
                valid = True
                self.filtered_error *= 0.95
                self.filtered_heading *= 0.95
                self.filtered_curvature *= 0.95
            else:
                valid = False

        # Build rich HUD
        debug_img = self.render_hud(
            img, clean_white_mask, debug_slices, slice_centers,
            y_top, y_bottom, valid, self.filtered_error, self.filtered_heading, self.filtered_curvature
        )

        return valid, self.filtered_error, self.filtered_heading, self.filtered_curvature, debug_img

    def render_hud(self, img, clean_white_mask, debug_slices, slice_centers,
                   y_top, y_bottom, valid, lat_err, head_err, curvature):
        """Renders HUD with road boundaries, track center line, and anti-grass margin."""
        hud = img.copy()
        h, w = hud.shape[:2]
        half_w = w // 2

        # 1. Overlay segmented white lines in neon green
        neon_green = np.zeros_like(hud)
        neon_green[:, :] = (0, 255, 128)
        mask_3ch = cv2.cvtColor(clean_white_mask, cv2.COLOR_GRAY2BGR)
        hud = np.where(mask_3ch > 0, cv2.addWeighted(hud, 0.40, neon_green, 0.60, 0), hud)

        # 2. Draw corridor boundaries & slice centers
        pts_center = []
        for y, lb, rb, rc, cw in debug_slices:
            cv2.line(hud, (lb, y), (lb + 8, y), (0, 0, 255), 2)
            cv2.line(hud, (rb - 8, y), (rb, y), (0, 0, 255), 2)
            if cw is not None:
                cv2.circle(hud, (int(cw), y), 3, (255, 255, 0), -1)
            if rc is not None:
                cv2.circle(hud, (int(rc), y), 4, (0, 255, 255), -1)
                pts_center.append((int(rc), int(y)))

        # Draw smooth center line connecting slice centers
        if len(pts_center) >= 2:
            pts_sorted = sorted(pts_center, key=lambda p: p[1])
            for i in range(1, len(pts_sorted)):
                cv2.line(hud, pts_sorted[i - 1], pts_sorted[i], (0, 220, 255), 3)

        # 3. Vehicle center vs Target center
        target_x = int(half_w + lat_err * half_w)
        target_x = max(10, min(w - 10, target_x))
        cv2.circle(hud, (half_w, y_bottom), 6, (255, 255, 255), 2)
        cv2.circle(hud, (target_x, y_bottom), 8, (0, 255, 0) if valid else (0, 0, 255), -1)
        cv2.line(hud, (half_w, y_bottom), (target_x, y_bottom), (0, 255, 255), 2)

        # 4. Telemetry Card
        card_w, card_h = 350, 95
        overlay = hud.copy()
        cv2.rectangle(overlay, (10, 10), (10 + card_w, 10 + card_h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.80, hud, 0.20, 0, hud)
        cv2.rectangle(hud, (10, 10), (10 + card_w, 10 + card_h), (0, 255, 200), 1)

        status_str = "CIRCUIT TRACKING" if valid else "SEARCH / HOLD"
        status_color = (0, 255, 128) if valid else (0, 140, 255)
        cv2.putText(hud, f"FUSED LANE: {status_str}", (20, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, status_color, 2)
        cv2.putText(hud, f"Lat Error: {lat_err:+.3f}  |  Heading: {head_err:+.3f}", (20, 56),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (240, 240, 240), 1)
        turn_type = "RIGHT TURN" if curvature > 0.04 else ("LEFT TURN" if curvature < -0.04 else "STRAIGHT")
        cv2.putText(hud, f"Curvature: {curvature:+.3f} [{turn_type}]", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 220, 255), 1)

        return hud


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FusedLaneDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
