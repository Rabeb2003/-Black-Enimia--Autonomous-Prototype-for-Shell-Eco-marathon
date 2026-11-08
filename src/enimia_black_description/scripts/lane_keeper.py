#!/usr/bin/env python3
"""
Lane Keeping Node using Computer Vision
Detects lane lines and computes steering commands for Ackermann steering
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


class LaneKeeper(Node):
    def __init__(self):
        super().__init__('lane_keeper')
        
        # Parameters
        self.declare_parameter('camera_topic', '/oak/rgb/image_raw')
        self.declare_parameter('cmd_vel_topic', '/ackermann_cmd')
        self.declare_parameter('target_speed', 0.5)  # m/s
        self.declare_parameter('max_steering_angle', 0.52)  # radians (~30 degrees)
        self.declare_parameter('roi_height', 0.6)  # Use bottom 60% of image
        self.declare_parameter('debug', True)
        
        self.camera_topic = self.get_parameter('camera_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.target_speed = self.get_parameter('target_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.roi_height = self.get_parameter('roi_height').value
        self.debug = self.get_parameter('debug').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        if self.debug:
            self.debug_image_pub = self.create_publisher(Image, '/lane_keeper/debug_image', 10)
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info('Lane Keeper Node initialized')
        self.get_logger().info(f'Target speed: {self.target_speed} m/s')
        self.get_logger().info(f'Max steering angle: {self.max_steering_angle} rad')
    
    def preprocess_image(self, image):
        """Preprocess image for lane detection"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Define ROI (Region of Interest)
        height, width = edges.shape
        roi_top = int(height * (1 - self.roi_height))
        roi = np.zeros_like(edges)
        roi[roi_top:, :] = edges[roi_top:, :]
        
        return roi
    
    def detect_lines(self, edges):
        """Detect lines using Hough Transform"""
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=50,
            minLineLength=50,
            maxLineGap=50
        )
        return lines
    
    def separate_lines(self, lines, image_width):
        """Separate lines into left and right lanes"""
        left_lines = []
        right_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Calculate slope
                if x2 - x1 == 0:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                
                # Filter lines based on slope
                # Left lane: positive slope, Right lane: negative slope
                if 0.3 < slope < 2.0:
                    left_lines.append(line[0])
                elif -2.0 < slope < -0.3:
                    right_lines.append(line[0])
        
        return left_lines, right_lines
    
    def average_lines(self, lines):
        """Average multiple lines to get a single line"""
        if not lines:
            return None
        
        # Average x and y coordinates
        lines = np.array(lines)
        x_coords = np.mean(lines[:, [0, 2]], axis=1)
        y_coords = np.mean(lines[:, [1, 3]], axis=1)
        
        # Fit a line through the averaged points
        if len(x_coords) > 1:
            poly_fit = np.polyfit(y_coords, x_coords, 1)
            return poly_fit  # Returns [slope, intercept]
        return None
    
    def compute_steering(self, left_fit, right_fit, image_width):
        """Compute steering angle from lane fits"""
        steering_angle = 0.0
        
        if left_fit is not None and right_fit is not None:
            # Both lanes detected - compute center
            left_x = left_fit[0] * image_height + left_fit[1]
            right_x = right_fit[0] * image_height + right_fit[1]
            lane_center = (left_x + right_x) / 2
            image_center = image_width / 2
            
            # Compute error from center
            error = lane_center - image_center
            
            # Convert error to steering angle (simple proportional control)
            steering_angle = -error * 0.01  # Adjust gain as needed
            
        elif left_fit is not None:
            # Only left lane detected - steer right
            left_x = left_fit[0] * image_height + left_fit[1]
            error = left_x - image_width * 0.3
            steering_angle = -error * 0.005
            
        elif right_fit is not None:
            # Only right lane detected - steer left
            right_x = right_fit[0] * image_height + right_fit[1]
            error = right_x - image_width * 0.7
            steering_angle = -error * 0.005
        
        # Clamp steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        return steering_angle
    
    def image_callback(self, msg):
        """Process camera images for lane detection"""
        global image_height
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_height, image_width = cv_image.shape[:2]
            
            # Preprocess
            edges = self.preprocess_image(cv_image)
            
            # Detect lines
            lines = self.detect_lines(edges)
            
            # Separate left and right lanes
            left_lines, right_lines = self.separate_lines(lines, image_width)
            
            # Average lines
            left_fit = self.average_lines(left_lines)
            right_fit = self.average_lines(right_lines)
            
            # Compute steering angle
            steering_angle = self.compute_steering(left_fit, right_fit, image_width)
            
            # Create command message
            cmd_msg = TwistStamped()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'base_link'
            cmd_msg.twist.linear.x = self.target_speed
            cmd_msg.twist.angular.z = steering_angle
            
            # Publish command
            self.cmd_vel_pub.publish(cmd_msg)
            
            # Debug visualization
            if self.debug:
                debug_image = cv_image.copy()
                
                # Draw detected lines
                if left_lines:
                    for line in left_lines:
                        x1, y1, x2, y2 = line
                        cv2.line(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                if right_lines:
                    for line in right_lines:
                        x1, y1, x2, y2 = line
                        cv2.line(debug_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                
                # Draw averaged lines
                if left_fit is not None:
                    y1 = int(image_height * 0.6)
                    y2 = image_height
                    x1 = int(left_fit[0] * y1 + left_fit[1])
                    x2 = int(left_fit[0] * y2 + left_fit[1])
                    cv2.line(debug_image, (x1, y1), (x2, y2), (0, 255, 255), 3)
                
                if right_fit is not None:
                    y1 = int(image_height * 0.6)
                    y2 = image_height
                    x1 = int(right_fit[0] * y1 + right_fit[1])
                    x2 = int(right_fit[0] * y2 + right_fit[1])
                    cv2.line(debug_image, (x1, y1), (x2, y2), (0, 255, 255), 3)
                
                # Display steering angle
                cv2.putText(debug_image, f'Steering: {steering_angle:.3f} rad',
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Publish debug image
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneKeeper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
