#!/usr/bin/env python3
"""
LiDAR Obstacle & Barrier Monitor for ENIMIA Black.
Role:
1. Subscribes to /livox/scan (sensor_msgs/LaserScan).
2. Continuously monitors obstacles (barriers, cones, track walls) in 3 sectors:
   - Front Sector: [-22 deg, +22 deg] (Collision avoidance)
   - Right Sector: [-85 deg, -22 deg] (Barrier/cone clearance on right side)
   - Left Sector:  [+22 deg, +85 deg] (Barrier/cone clearance on left side)
3. Computes:
   - Minimum obstacle distance in forward direction.
   - Proactive barrier repulsive steering bias (margin: 2.40m) to prevent cutting inside barriers.
   - Emergency brake signal if an obstacle is closer than 1.30m.
4. Publishes:
   - /lidar/obstacle_distance       (Float32, in meters)
   - /lidar/barrier_right_distance  (Float32, in meters)
   - /lidar/barrier_left_distance   (Float32, in meters)
   - /lidar/barrier_steer_bias      (Float32, corrective steering bias)
   - /lidar/emergency_brake         (Bool, True if collision imminent < 1.3m)
   - /lidar/obstacle_alert          (String)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


class LidarObstacleMonitor(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_monitor')

        # Parameters
        self.declare_parameter('scan_topic', '/livox/scan')
        self.declare_parameter('front_angle_deg', 22.0)
        self.declare_parameter('side_max_angle_deg', 85.0)
        self.declare_parameter('danger_distance', 1.30)      # Hard emergency stop distance (m)
        self.declare_parameter('warning_distance', 3.80)     # Warning alert distance (m)
        self.declare_parameter('barrier_safe_margin', 2.40)  # Proactive clearance from track barriers (m)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.front_angle_rad = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.side_max_rad = math.radians(float(self.get_parameter('side_max_angle_deg').value))
        self.danger_distance = float(self.get_parameter('danger_distance').value)
        self.warning_distance = float(self.get_parameter('warning_distance').value)
        self.barrier_safe_margin = float(self.get_parameter('barrier_safe_margin').value)

        # Publishers
        self.pub_dist = self.create_publisher(Float32, '/lidar/obstacle_distance', 10)
        self.pub_right_dist = self.create_publisher(Float32, '/lidar/barrier_right_distance', 10)
        self.pub_left_dist = self.create_publisher(Float32, '/lidar/barrier_left_distance', 10)
        self.pub_steer_bias = self.create_publisher(Float32, '/lidar/barrier_steer_bias', 10)
        self.pub_brake = self.create_publisher(Bool, '/lidar/emergency_brake', 10)
        self.pub_alert = self.create_publisher(String, '/lidar/obstacle_alert', 10)

        # Subscription to LiDAR
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f'LiDAR Obstacle Monitor started on {self.scan_topic} (Safe Margin: {self.barrier_safe_margin} m)')

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)

        # Filter out invalid / out-of-range measurements
        valid_mask = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        if not np.any(valid_mask):
            return

        # Compute angle for each ray
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # 1. Front Corridor (Collision Sector)
        front_mask = valid_mask & (np.abs(angles) <= self.front_angle_rad)
        min_front = float(np.min(ranges[front_mask])) if np.any(front_mask) else math.inf

        # 2. Right Barrier Sector (Negative angles in standard ROS frame)
        right_mask = valid_mask & (angles < -self.front_angle_rad) & (angles >= -self.side_max_rad)
        min_right = float(np.min(ranges[right_mask])) if np.any(right_mask) else math.inf

        # 3. Left Barrier Sector (Positive angles)
        left_mask = valid_mask & (angles > self.front_angle_rad) & (angles <= self.side_max_rad)
        min_left = float(np.min(ranges[left_mask])) if np.any(left_mask) else math.inf

        nearest_overall = min(min_front, min_right, min_left)

        # Emergency Brake Trigger
        emergency_brake = bool(min_front <= self.danger_distance)

        # Proactive Repulsive Barrier Steering Bias
        barrier_bias = 0.0
        if min_right < self.barrier_safe_margin:
            deflection = (self.barrier_safe_margin - min_right) / self.barrier_safe_margin
            barrier_bias += float(0.20 * deflection)  # Steer left (+z) away from right barrier

        if min_left < self.barrier_safe_margin:
            deflection = (self.barrier_safe_margin - min_left) / self.barrier_safe_margin
            barrier_bias -= float(0.20 * deflection)  # Steer right (-z) away from left barrier

        barrier_bias = float(np.clip(barrier_bias, -0.20, 0.20))

        # Alert String
        if emergency_brake:
            alert_str = f"DANGER COLLISION ({min_front:.2f} m) -> FREINAGE D'URGENCE"
        elif min_front <= self.warning_distance:
            alert_str = f"ATTENTION OBSTACLE DEVANT : {min_front:.2f} m"
        elif min_right <= self.barrier_safe_margin:
            alert_str = f"BARRIERE DROITE PROCHE : {min_right:.2f} m -> REPULSION GAUCHE"
        elif min_left <= self.barrier_safe_margin:
            alert_str = f"BARRIERE GAUCHE PROCHE : {min_left:.2f} m -> REPULSION DROITE"
        else:
            alert_str = f"VOIE DEGAGEE ({nearest_overall:.2f} m)"

        # Publish
        self.pub_dist.publish(Float32(data=float(min_front if math.isfinite(min_front) else 99.0)))
        self.pub_right_dist.publish(Float32(data=float(min_right if math.isfinite(min_right) else 99.0)))
        self.pub_left_dist.publish(Float32(data=float(min_left if math.isfinite(min_left) else 99.0)))
        self.pub_steer_bias.publish(Float32(data=barrier_bias))
        self.pub_brake.publish(Bool(data=emergency_brake))
        self.pub_alert.publish(String(data=alert_str))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarObstacleMonitor()
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
