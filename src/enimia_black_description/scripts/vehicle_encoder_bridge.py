#!/usr/bin/env python3
"""
Vehicle Encoder & Wheel Odometry Telemetry Bridge for ENIMIA Black.
Role:
1. Steering Wheel Encoder:
   - Reads front left & right steering joints ('front_left_steer_joint', 'front_right_steer_joint')
   - Computes exact steering wheel rotation angle in degrees and radians.
   - Publishes:
       /encoder/steering_angle_deg   (Float32, in degrees: +left, -right)
       /encoder/steering_angle_rad   (Float32, in radians)
       /encoder/steering_status      (String, human-readable e.g. "VOLANT: DROITE 8.5 deg")

2. Wheel Encoder & Odometry:
   - Reads rear drive joint ('rear_wheel_joint') position and velocity.
   - Computes:
       * Cumulative wheel angle (0 to 360 deg per rotation)
       * Total wheel revolutions
       * Real-time linear speed (m/s and km/h)
       * Encoder-based distance traveled
   - Publishes:
       /encoder/wheel_angle_deg      (Float32, 0 to 360 degrees)
       /encoder/wheel_angle_rad      (Float32, 0 to 2*pi radians)
       /encoder/wheel_revolutions    (Float32, total turns of the wheel)
       /encoder/wheel_speed_kmh      (Float32, speed in km/h)
       /encoder/wheel_odometry       (Odometry, wheel encoder odom message)
"""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String


class VehicleEncoderBridge(Node):
    def __init__(self):
        super().__init__('vehicle_encoder_bridge')

        # Parameters
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('wheel_radius', 0.18)      # 18 cm rear wheel radius
        self.declare_parameter('wheelbase', 1.704)
        self.declare_parameter('publish_rate', 50.0)

        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)

        # Publishers - Steering Wheel Encoder
        self.pub_steer_deg = self.create_publisher(Float32, '/encoder/steering_angle_deg', 10)
        self.pub_steer_rad = self.create_publisher(Float32, '/encoder/steering_angle_rad', 10)
        self.pub_steer_status = self.create_publisher(String, '/encoder/steering_status', 10)

        # Publishers - Wheel Encoder & Odometry
        self.pub_wheel_deg = self.create_publisher(Float32, '/encoder/wheel_angle_deg', 10)
        self.pub_wheel_rad = self.create_publisher(Float32, '/encoder/wheel_angle_rad', 10)
        self.pub_wheel_rev = self.create_publisher(Float32, '/encoder/wheel_revolutions', 10)
        self.pub_wheel_kmh = self.create_publisher(Float32, '/encoder/wheel_speed_kmh', 10)
        self.pub_wheel_odom = self.create_publisher(Odometry, '/encoder/wheel_odometry', 10)

        # Subscription to joint states
        self.create_subscription(JointState, self.joint_states_topic, self.joint_state_callback, 10)

        # State tracking
        self.total_distance = 0.0
        self.initial_wheel_pos = None
        self.last_wheel_pos = 0.0

        self.get_logger().info(
            f'Vehicle Encoder Bridge active: Reading {self.joint_states_topic} -> Publishing /encoder/ telemetry'
        )

    def joint_state_callback(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        # 1. Steering Wheel Encoder Processing
        has_left_steer = 'front_left_steer_joint' in name_to_idx
        has_right_steer = 'front_right_steer_joint' in name_to_idx

        if has_left_steer and has_right_steer:
            idx_l = name_to_idx['front_left_steer_joint']
            idx_r = name_to_idx['front_right_steer_joint']
            pos_l = msg.position[idx_l] if idx_l < len(msg.position) else 0.0
            pos_r = msg.position[idx_r] if idx_r < len(msg.position) else 0.0

            # Average steering angle (Ackermann center angle)
            avg_steer_rad = float((pos_l + pos_r) / 2.0)
            avg_steer_deg = float(math.degrees(avg_steer_rad))

            self.pub_steer_rad.publish(Float32(data=avg_steer_rad))
            self.pub_steer_deg.publish(Float32(data=avg_steer_deg))

            # Format status direction
            if abs(avg_steer_deg) < 0.5:
                direction_str = "CENTRE"
            elif avg_steer_deg > 0:
                direction_str = f"GAUCHE {abs(avg_steer_deg):.1f} deg"
            else:
                direction_str = f"DROITE {abs(avg_steer_deg):.1f} deg"

            self.pub_steer_status.publish(String(data=f"ENCODEUR DIRECTION: {direction_str}"))

        # 2. Wheel Encoder Processing
        if 'rear_wheel_joint' in name_to_idx:
            idx_w = name_to_idx['rear_wheel_joint']
            pos_w = msg.position[idx_w] if idx_w < len(msg.position) else 0.0
            vel_w = msg.velocity[idx_w] if idx_w < len(msg.velocity) else 0.0

            if self.initial_wheel_pos is None:
                self.initial_wheel_pos = pos_w

            rel_pos = pos_w - self.initial_wheel_pos
            total_revolutions = rel_pos / (2.0 * math.pi)

            # Instantaneous wheel angle in current revolution (0 to 360 deg)
            current_rev_rad = rel_pos % (2.0 * math.pi)
            current_rev_deg = math.degrees(current_rev_rad)

            # Speed calculation
            speed_mps = vel_w * self.wheel_radius
            speed_kmh = speed_mps * 3.6

            # Distance
            distance_m = rel_pos * self.wheel_radius

            self.pub_wheel_deg.publish(Float32(data=float(current_rev_deg)))
            self.pub_wheel_rad.publish(Float32(data=float(current_rev_rad)))
            self.pub_wheel_rev.publish(Float32(data=float(total_revolutions)))
            self.pub_wheel_kmh.publish(Float32(data=float(speed_kmh)))

            # Publish Odometry message
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.twist.twist.linear.x = float(speed_mps)
            self.pub_wheel_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VehicleEncoderBridge()
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
