#!/usr/bin/env python3
"""
ROS2 wrapper for LaneAssist that publishes center_error and valid topics
compatible with the existing lane_keeping_controller for Nav2 integration.
"""
import os
import sys
import logging

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import cv2

from ament_index_python.packages import get_package_share_directory

# Add package share directory to path for imports
sys.path.insert(0, os.path.join(get_package_share_directory('lane_assist_ros'), 'src'))
from LaneDetection.detect import LaneDetection
from LaneKeeping.lanekeeping import LaneKeeping


class LaneAssistWrapper(Node):
    def __init__(self):
        super().__init__("lane_assist_wrapper")

        # Get config path
        config_path = os.path.join(get_package_share_directory('lane_assist_ros'), 'config.ini')
        self.get_logger().info(f"Config path: {config_path}")

        # ROS2 interfaces
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('output_image_topic', '/lane_detection/output')
        self.declare_parameter('error_topic', '/lane_detection/center_error')
        self.declare_parameter('valid_topic', '/lane_detection/valid')
        self.declare_parameter('camera', '455')
        self.declare_parameter('max_fps', 15.0)

        image_topic = self.get_parameter('image_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        error_topic = self.get_parameter('error_topic').value
        valid_topic = self.get_parameter('valid_topic').value
        camera = self.get_parameter('camera').value
        self.max_fps = self.get_parameter('max_fps').value

        # Publishers
        self.image_pub = self.create_publisher(Image, output_image_topic, 10)
        self.error_pub = self.create_publisher(Float32, error_topic, 10)
        self.valid_pub = self.create_publisher(Bool, valid_topic, 10)

        # Subscriber
        self.create_subscription(Image, image_topic, self.image_callback, 10)

        # LaneAssist components
        self.logger = logging.getLogger("lane_assist_ros")
        self.img = None
        self.width = None
        self.height = None
        self.ld = None
        self.lk = None
        self.last_process_time = 0.0

        self.get_logger().info(f"LaneAssist wrapper on {image_topic}")
        self.get_logger().info(f"Camera type: {camera}")

    def image_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.max_fps > 0.0 and now - self.last_process_time < 1.0 / self.max_fps:
            return
        self.last_process_time = now

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        self.img = img

        if self.width is None:
            self.width = img.shape[1]
            self.height = img.shape[0]

            # Initialize LaneAssist components
            camera = self.get_parameter('camera').value
            config_path = os.path.join(get_package_share_directory('lane_assist_ros'), 'config.ini')
            self.lk = LaneKeeping(self.width, self.height, self.logger, camera, config_path)
            self.ld = LaneDetection(self.width, self.height, camera, self.lk, config_path)

            # Disable visualization for ROS2
            self.ld.print_lanes = False
            self.ld.print_peaks = False
            self.ld.print_lane_certainty = False
            self.lk.print_desire_lane = False

            self.get_logger().info("LaneAssist initialized")

    def process_frame(self):
        if self.img is None or self.ld is None or self.lk is None:
            return

        frame = self.img.copy()

        try:
            # Lane detection
            results = self.ld.lanes_detection(frame)

            # Extract lane information
            trust_lk = results.get('trust_lk', False)
            trust_left = results.get('trust_left', False)
            trust_right = results.get('trust_right', False)
            left_coef = results.get('left_coef')
            right_coef = results.get('right_coef')

            # Lane keeping to get angle and debug image
            angle, out_frame = self.lk.lane_keeping(results)

            # Publish debug image
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(out_frame, "bgr8")
                self.image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"Debug image publish error: {e}")

            # Calculate center error from lane keeping angle
            # Angle is in degrees, convert to normalized error [-1, 1]
            # Positive angle = turn right, negative = turn left
            # We want: positive error = right of center, negative error = left of center
            if trust_lk and (trust_left or trust_right):
                # Convert angle to normalized error
                # Max steering angle is about 23 degrees (from config)
                max_angle = 23.0
                normalized_error = -angle / max_angle  # Invert sign for correct direction
                normalized_error = max(min(normalized_error, 1.0), -1.0)
                
                self.error_pub.publish(Float32(data=float(normalized_error)))
                self.valid_pub.publish(Bool(data=True))
            else:
                # No valid lanes detected
                self.error_pub.publish(Float32(data=0.0))
                self.valid_pub.publish(Bool(data=False))

        except Exception as e:
            self.get_logger().error(f"LaneAssist processing error: {e}")
            import traceback
            traceback.print_exc()
            # Publish invalid on error
            self.error_pub.publish(Float32(data=0.0))
            self.valid_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = LaneAssistWrapper()

    # Create timer for processing
    node.create_timer(0.05, node.process_frame)  # 20 Hz

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
