#!/usr/bin/env python3
"""
Stop Sign Detection Node using YOLO
Detects stop signs from camera images and publishes detection results
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


class StopSignDetector(Node):
    def __init__(self):
        super().__init__('stop_sign_detector')
        
        # Parameters
        self.declare_parameter('camera_topic', '/oak/rgb/image_raw')
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('stop_distance_threshold', 5.0)  # meters
        
        camera_topic = self.get_parameter('camera_topic').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.stop_distance_threshold = self.get_parameter('stop_distance_threshold').value
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # YOLO model (using OpenCV DNN with pre-trained model)
        # Note: You need to download YOLO weights and config files
        self.net = None
        self.load_yolo_model()
        
        # Publishers
        self.stop_detected_pub = self.create_publisher(Bool, '/stop_sign/detected', 10)
        self.stop_distance_pub = self.create_publisher(Float32, '/stop_sign/distance', 10)
        self.debug_image_pub = self.create_publisher(Image, '/stop_sign/debug_image', 10)
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info('Stop Sign Detector Node initialized')
        self.get_logger().info(f'Listening on: {camera_topic}')
    
    def load_yolo_model(self):
        """Load YOLO model for stop sign detection"""
        try:
            # Using OpenCV's Haar Cascade as fallback if YOLO not available
            # For production, replace with YOLOv8 or YOLOv5
            self.stop_sign_cascade = cv2.CascadeClassifier(
                cv2.data.haarcascades + 'haarcascade_stop_sign.xml'
            )
            if self.stop_sign_cascade.empty():
                # Create a simple color-based detector as fallback
                self.stop_sign_cascade = None
                self.get_logger().warn('Haar cascade not found, using color-based detection')
            else:
                self.get_logger().info('Loaded Haar Cascade for stop sign detection')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.stop_sign_cascade = None
    
    def detect_stop_sign_color(self, image):
        """Simple color-based stop sign detection (red octagon)"""
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Red color range (two ranges for red in HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                # Check aspect ratio (stop signs are roughly square/octagonal)
                aspect_ratio = float(w) / h
                if 0.7 < aspect_ratio < 1.3:
                    detections.append({
                        'bbox': (x, y, w, h),
                        'confidence': min(1.0, area / 5000.0)  # Simple confidence based on size
                    })
        
        return detections
    
    def image_callback(self, msg):
        """Process camera images for stop sign detection"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect stop signs
            if self.stop_sign_cascade is not None:
                # Use Haar cascade
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                signs = self.stop_sign_cascade.detectMultiScale(
                    gray,
                    scaleFactor=1.1,
                    minNeighbors=5,
                    minSize=(30, 30)
                )
                detections = [{'bbox': (x, y, w, h), 'confidence': 0.8} for (x, y, w, h) in signs]
            else:
                # Use color-based detection
                detections = self.detect_stop_sign_color(cv_image)
            
            # Process detections
            stop_detected = False
            min_distance = float('inf')
            
            for detection in detections:
                x, y, w, h = detection['bbox']
                confidence = detection['confidence']
                
                if confidence >= self.detection_threshold:
                    stop_detected = True
                    
                    # Estimate distance based on bounding box size
                    # This is a rough estimate - calibrate for your camera
                    bbox_area = w * h
                    estimated_distance = 10000.0 / (bbox_area + 1.0)  # Simple inverse relationship
                    
                    if estimated_distance < min_distance:
                        min_distance = estimated_distance
                    
                    # Draw bounding box for debug
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(cv_image, f'Stop: {confidence:.2f}', (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Publish detection results
            stop_msg = Bool()
            stop_msg.data = stop_detected and (min_distance < self.stop_distance_threshold)
            self.stop_detected_pub.publish(stop_msg)
            
            distance_msg = Float32()
            distance_msg.data = min_distance if min_distance != float('inf') else 0.0
            self.stop_distance_pub.publish(distance_msg)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
