#!/usr/bin/env python3
"""
Stanley Controller for Lane Keeping
Implements the Stanley lateral control algorithm for autonomous steering
Reference: Hoffmann et al. (2007) - "Autonomous Automobile Trajectory Tracking"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import numpy as np
import math


class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        
        # Parameters
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_stanley')
        self.declare_parameter('steering_topic', '/steering_angle')
        self.declare_parameter('k_gain', 1.0)  # Cross-track error gain
        self.declare_parameter('k_soft', 1.0)  # Softening constant
        self.declare_parameter('lookahead_dist', 2.0)  # Lookahead distance
        self.declare_parameter('min_velocity', 0.5)  # Minimum velocity
        self.declare_parameter('max_velocity', 2.0)  # Maximum velocity
        self.declare_parameter('max_steering_angle', 0.52)  # Max steering angle (rad)
        self.declare_parameter('wheelbase', 1.7)  # Vehicle wheelbase
        
        self.path_topic = self.get_parameter('path_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.steering_topic = self.get_parameter('steering_topic').value
        self.k_gain = self.get_parameter('k_gain').value
        self.k_soft = self.get_parameter('k_soft').value
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.min_velocity = self.get_parameter('min_velocity').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.wheelbase = self.get_parameter('wheelbase').value
        
        # State variables
        self.current_path = None
        self.current_pose = None
        self.current_velocity = 0.0
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            self.cmd_vel_topic,
            10
        )
        
        self.steering_pub = self.create_publisher(
            Float32,
            self.steering_topic,
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        self.get_logger().info('Stanley Controller initialized')
        self.get_logger().info(f'K gain: {self.k_gain}, Max steering: {self.max_steering_angle} rad')
    
    def path_callback(self, msg):
        """Store the reference path"""
        self.current_path = msg
    
    def pose_callback(self, msg):
        """Store current vehicle pose"""
        self.current_pose = msg
    
    def get_closest_point(self, path, pose):
        """Find the closest point on the path to the current pose"""
        if not path.poses:
            return None, float('inf')
        
        min_dist = float('inf')
        closest_idx = 0
        
        for i, waypoint in enumerate(path.poses):
            dx = waypoint.pose.position.x - pose.pose.position.x
            dy = waypoint.pose.position.y - pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx, min_dist
    
    def get_path_heading(self, path, idx):
        """Calculate the heading of the path at a given index"""
        if idx >= len(path.poses) - 1:
            idx = len(path.poses) - 2
        
        p1 = path.poses[idx].pose.position
        p2 = path.poses[idx + 1].pose.position
        
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        
        return math.atan2(dy, dx)
    
    def get_cross_track_error(self, path, pose, closest_idx):
        """Calculate cross-track error (lateral distance from path)"""
        if closest_idx >= len(path.poses) - 1:
            return 0.0
        
        # Get path segment
        p1 = path.poses[closest_idx].pose.position
        p2 = path.poses[closest_idx + 1].pose.position
        
        # Vector from p1 to p2
        path_vec_x = p2.x - p1.x
        path_vec_y = p2.y - p1.y
        path_length = math.sqrt(path_vec_x**2 + path_vec_y**2)
        
        if path_length == 0:
            return 0.0
        
        # Normalize path vector
        path_unit_x = path_vec_x / path_length
        path_unit_y = path_vec_y / path_length
        
        # Vector from p1 to vehicle
        vehicle_vec_x = pose.pose.position.x - p1.x
        vehicle_vec_y = pose.pose.position.y - p1.y
        
        # Cross-track error (perpendicular distance)
        cte = vehicle_vec_x * (-path_unit_y) + vehicle_vec_y * path_unit_x
        
        return cte
    
    def get_vehicle_heading(self, pose):
        """Get current vehicle heading from pose quaternion"""
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def compute_steering_angle(self, path, pose):
        """Compute steering angle using Stanley controller"""
        # Find closest point
        closest_idx, min_dist = self.get_closest_point(path, pose)
        
        if closest_idx is None:
            return 0.0
        
        # Get path heading at closest point
        path_heading = self.get_path_heading(path, closest_idx)
        
        # Get vehicle heading
        vehicle_heading = self.get_vehicle_heading(pose)
        
        # Heading error
        heading_error = path_heading - vehicle_heading
        
        # Normalize heading error to [-pi, pi]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        # Cross-track error
        cte = self.get_cross_track_error(path, pose, closest_idx)
        
        # Stanley controller formula
        # delta = psi_error + arctan(k * cte / (v + k_soft))
        velocity = max(self.current_velocity, self.min_velocity)
        steering_angle = heading_error + math.atan2(self.k_gain * cte, velocity + self.k_soft)
        
        # Clamp steering angle
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        
        return steering_angle, cte, heading_error
    
    def compute_velocity(self, cte, steering_angle):
        """Compute target velocity based on lateral error and steering"""
        # Reduce velocity when lateral error is high or steering is large
        velocity_reduction = 0.0
        
        if abs(cte) > 0.5:
            velocity_reduction = 0.3
        elif abs(cte) > 0.3:
            velocity_reduction = 0.15
        
        if abs(steering_angle) > 0.4:
            velocity_reduction = max(velocity_reduction, 0.2)
        
        target_velocity = self.max_velocity * (1.0 - velocity_reduction)
        target_velocity = max(self.min_velocity, target_velocity)
        
        return target_velocity
    
    def control_loop(self):
        """Main control loop"""
        if self.current_path is None or self.current_pose is None:
            return
        
        # Compute steering angle
        steering_angle, cte, heading_error = self.compute_steering_angle(
            self.current_path,
            self.current_pose
        )
        
        # Compute velocity
        target_velocity = self.compute_velocity(cte, steering_angle)
        self.current_velocity = target_velocity
        
        # Create command message
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link'
        cmd_msg.twist.linear.x = target_velocity
        cmd_msg.twist.angular.z = steering_angle
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_msg)
        
        # Publish steering angle for debugging
        steering_msg = Float32()
        steering_msg.data = steering_angle
        self.steering_pub.publish(steering_msg)
        
        # Log info periodically
        if self.get_clock().now().nanoseconds % 500000000 < 50000000:  # Every 0.5s
            self.get_logger().debug(
                f'Steering: {steering_angle:.3f} rad, CTE: {cte:.3f} m, '
                f'Heading error: {heading_error:.3f} rad, Velocity: {target_velocity:.2f} m/s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
