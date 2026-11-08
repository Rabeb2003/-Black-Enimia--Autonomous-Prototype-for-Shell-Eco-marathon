#!/usr/bin/env python3
"""
ROS2 Node wrapper for LaneAssist lane detection and lane keeping system
Subscribes to camera image topic and publishes steering commands
"""
import os
import sys
import logging
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

# Add package share directory to path for imports
from ament_index_python.packages import get_package_share_directory

# Import LaneAssist modules
sys.path.insert(0, os.path.join(get_package_share_directory('lane_assist_ros'), 'src'))
from LaneDetection.detect import LaneDetection
from LaneKeeping.lanekeeping import LaneKeeping


class LaneAssistNode(Node):
    def __init__(self):
        super().__init__("lane_assist_node")

        # Get config path
        config_path = os.path.join(get_package_share_directory('lane_assist_ros'), 'config.ini')
        self.get_logger().info(f"Config path: {config_path}")

        # ROS2 interfaces
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/oak/left/image_raw", self.image_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_debug = self.create_publisher(Image, "/lane_assist/debug_image", 10)

        # LaneAssist components
        self.logger = logging.getLogger("lane_assist_ros")
        self.img = None
        self.width = None
        self.height = None
        self.ld = None
        self.lk = None

        # Parameters
        self.camera = self.declare_parameter("camera", "455").value
        self.linear_speed = self.declare_parameter("linear_speed", 0.5).value
        self.steering_gain = self.declare_parameter("steering_gain", 0.02).value

        self.get_logger().info(f"Camera type: {self.camera}")
        self.get_logger().info(f"Linear speed: {self.linear_speed}")
        self.get_logger().info(f"Steering gain: {self.steering_gain}")

    def image_cb(self, msg):
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
            self.lk = LaneKeeping(self.width, self.height, self.logger, self.camera, config_path)
            self.ld = LaneDetection(self.width, self.height, self.camera, self.lk, config_path)

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

            # Lane keeping
            angle, out_frame = self.lk.lane_keeping(results)

            # Convert angle to ROS2 command
            cmd = Twist()
            cmd.linear.x = float(self.linear_speed)
            # Convert angle from degrees to radians and scale
            cmd.angular.z = float(angle * self.steering_gain)

            self.pub_cmd.publish(cmd)

            # Publish debug image
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(out_frame, "bgr8")
                self.pub_debug.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"Debug image publish error: {e}")

        except Exception as e:
            self.get_logger().error(f"LaneAssist processing error: {e}")
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = LaneAssistNode()

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
