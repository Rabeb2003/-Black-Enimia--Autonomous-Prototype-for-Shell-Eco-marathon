#!/usr/bin/env python3
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from ultralytics import YOLO


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('stop_topic', '/stop_detected')
        self.declare_parameter('output_image_topic', '/lane_detection/output')
        self.declare_parameter('cmd_vel_topic', '/lane_detection/cmd_vel')
        self.declare_parameter('error_topic', '/lane_detection/center_error')
        self.declare_parameter('valid_topic', '/lane_detection/valid')
        self.declare_parameter('model_file', 'best.torchscript')
        self.declare_parameter('publish_cmd_vel', False)
        self.declare_parameter('display', False)
        self.declare_parameter('max_fps', 8.0)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 3.0)
        self.declare_parameter('kp', 0.20)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.03)
        self.declare_parameter('lookahead_row_ratio', 0.75)
        self.declare_parameter('error_smoothing', 0.35)

        self.bridge = CvBridge()
        self.stop_detected = False
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_process_time = 0.0

        self.publish_cmd_vel = bool(self.get_parameter('publish_cmd_vel').value)
        self.display = bool(self.get_parameter('display').value)
        self.max_fps = float(self.get_parameter('max_fps').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.lookahead_row_ratio = float(
            self.get_parameter('lookahead_row_ratio').value)
        self.error_smoothing = float(
            self.get_parameter('error_smoothing').value)

        model_path = self.resolve_model_path(self.get_parameter('model_file').value)
        self.model = YOLO(model_path, task='segment')
        self.get_logger().info(f'Lane model loaded: {model_path}')

        image_topic = self.get_parameter('image_topic').value
        stop_topic = self.get_parameter('stop_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        error_topic = self.get_parameter('error_topic').value
        valid_topic = self.get_parameter('valid_topic').value

        # Gazebo cameras use sensor-data QoS (best effort).  A default
        # reliable subscription is incompatible with them on several DDS
        # implementations and silently prevents lane messages from arriving.
        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, stop_topic, self.stop_callback, 10)
        self.image_pub = self.create_publisher(Image, output_image_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.error_pub = self.create_publisher(Float32, error_topic, 10)
        self.valid_pub = self.create_publisher(Bool, valid_topic, 10)

        self.filtered_error = 0.0
        self.lane_width_px = None

        mode = 'cmd_vel enabled' if self.publish_cmd_vel else 'diagnostic only'
        self.get_logger().info(f'Lane detection on {image_topic} ({mode})')

    def resolve_model_path(self, model_file):
        package_dir = get_package_share_directory('lane_detection')
        candidates = [
            model_file,
            os.path.join(package_dir, 'models', model_file),
            os.path.join(os.path.dirname(__file__), '..', 'models', model_file),
        ]
        for candidate in candidates:
            path = os.path.abspath(candidate)
            if os.path.exists(path):
                return path
        raise FileNotFoundError(f'Lane model not found: {model_file}')

    def stop_callback(self, msg):
        self.stop_detected = msg.data

    def image_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.max_fps > 0.0 and now - self.last_process_time < 1.0 / self.max_fps:
            return
        self.last_process_time = now

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model.predict(image, verbose=False)
            
            # Always publish the annotated image for debugging
            if results and len(results) > 0:
                annotated = results[0].plot()
            else:
                annotated = image.copy()
                cv2.putText(annotated, "No detection", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

            if not results or results[0].masks is None:
                self.publish_zero_if_needed()
                self.publish_lane_measurement(False, self.filtered_error)
                return

            if results[0].masks.xy:
                valid, error = self.process_lanes(
                    results[0].masks.xy, image.shape[1], image.shape[0])
                self.publish_lane_measurement(valid, error)
            else:
                self.publish_lane_measurement(False, self.filtered_error)
                self.publish_zero_if_needed()

            if self.display and os.getenv('DISPLAY'):
                cv2.imshow('Lane Detection', annotated)
                cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().error(f'Lane image processing error: {exc}')

    def process_lanes(self, detections, image_width, image_height):
        """Return a normalised lane-centre error in [-1, 1].

        The lower part of each segmented lane is used as the look-ahead
        measurement.  This is markedly more stable than averaging the whole
        mask, whose far-away pixels dominate when the road curves.  When one
        side is briefly occluded, the last measured lane width lets the lane
        keeper continue to steer rather than immediately losing the road.
        """
        left_xs = []
        right_xs = []
        row = float(np.clip(
            self.lookahead_row_ratio, 0.25, 0.95) * image_height)
        band = max(12.0, 0.08 * image_height)

        for mask in detections:
            points = np.asarray(mask)
            if points.ndim != 2 or points.shape[1] < 2:
                continue

            # Use points near the desired look-ahead row.  A fallback to the
            # lower third also supports small/partially visible masks.
            near_row = points[np.abs(points[:, 1] - row) <= band]
            if near_row.size == 0:
                lower = points[points[:, 1] >= np.percentile(points[:, 1], 70)]
                near_row = lower if lower.size else points
            x_at_lookahead = float(np.median(near_row[:, 0]))
            if x_at_lookahead < image_width / 2.0:
                left_xs.append(x_at_lookahead)
            else:
                right_xs.append(x_at_lookahead)

        left_x = float(np.median(left_xs)) if left_xs else None
        right_x = float(np.median(right_xs)) if right_xs else None

        if left_x is not None and right_x is not None:
            measured_width = right_x - left_x
            if 0.15 * image_width <= measured_width <= 0.95 * image_width:
                if self.lane_width_px is None:
                    self.lane_width_px = measured_width
                else:
                    self.lane_width_px = (
                        0.25 * measured_width + 0.75 * self.lane_width_px)
            center_x = (left_x + right_x) / 2.0
        elif self.lane_width_px is not None and left_x is not None:
            center_x = left_x + self.lane_width_px / 2.0
        elif self.lane_width_px is not None and right_x is not None:
            center_x = right_x - self.lane_width_px / 2.0
        else:
            self.get_logger().debug('No usable lane centre detected')
            return False, self.filtered_error

        raw_error = float(np.clip(
            (center_x - image_width / 2.0) / (image_width / 2.0), -1.0, 1.0))
        alpha = float(np.clip(self.error_smoothing, 0.0, 1.0))
        self.filtered_error = alpha * raw_error + (1.0 - alpha) * self.filtered_error
        self.publish_lane_cmd(self.filtered_error)
        return True, self.filtered_error

    def publish_lane_measurement(self, valid, error):
        self.error_pub.publish(Float32(data=float(error)))
        self.valid_pub.publish(Bool(data=bool(valid)))

    def publish_lane_cmd(self, error):
        if not self.publish_cmd_vel:
            return

        twist = Twist()
        if self.stop_detected:
            self.cmd_pub.publish(twist)
            return

        derivative = error - self.prev_error
        self.integral += error
        control = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        twist.linear.x = self.linear_speed
        # ``control`` is a curvature command.  Twist angular.z is yaw rate.
        twist.angular.z = -float(np.clip(
            self.linear_speed * control,
            -self.max_angular_speed,
            self.max_angular_speed,
        ))
        self.cmd_pub.publish(twist)

    def publish_zero_if_needed(self):
        if self.publish_cmd_vel:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneDetectionNode()
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
