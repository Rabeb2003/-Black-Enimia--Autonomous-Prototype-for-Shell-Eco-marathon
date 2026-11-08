#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fused Pure-Pursuit & Curvature-Adaptive Controller for lane_follower.
Key Improvements:
1) Correct Pure-Pursuit lateral geometry: delta = arctan2(2 * L * x_la, L_d^2)
2) Curvature-Adaptive Throttle (from line_follower_ws):
   Slows down automatically in turns and accelerates on straights.
3) Smooth rate limiter on steering and throttle.
"""
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32

class PurePursuitCtrl(Node):
    def __init__(self):
        super().__init__("pure_pursuit_ctrl")

        # Parameters
        self.declare_parameter("wheel_base", 1.704)       # ENIMIA Black wheelbase
        self.declare_parameter("max_steer_deg", 30.0)
        self.declare_parameter("max_throttle", 0.85)      # Top straight throttle
        self.declare_parameter("min_curve_throttle", 0.35) # Floor throttle in sharp bends
        self.declare_parameter("throttle_decay_exp", 1.3)

        self.L_m = float(self.get_parameter("wheel_base").value)
        self.max_steer = float(self.get_parameter("max_steer_deg").value)
        self.max_throttle = float(self.get_parameter("max_throttle").value)
        self.min_curve_throttle = float(self.get_parameter("min_curve_throttle").value)
        self.decay_exp = float(self.get_parameter("throttle_decay_exp").value)

        self.pub_steer = self.create_publisher(Float32, "/car/steering", 1)
        self.pub_throttle = self.create_publisher(Float32, "/car/throttle", 1)

        self.create_subscription(Point32, "/path/lookahead_point", self.cb_la, 1)
        self.last_steer = 0.0

        self.get_logger().info(f"Enhanced PurePursuitCtrl running (Wheelbase={self.L_m}m)")

    def cb_la(self, p: Point32):
        # p.y = forward lookahead distance [m]
        # p.x = lateral deviation from vehicle centerline [m]
        y_la = float(p.y)
        x_la = float(p.x)

        # Lookahead distance squared
        Ld_sq = x_la**2 + y_la**2
        if Ld_sq < 1.0e-4:
            return

        # Correct Pure Pursuit formula: delta = arctan(2 * L * x_la / L_d^2)
        steer_rad = np.arctan2(2.0 * self.L_m * x_la, Ld_sq)
        steer_deg = float(np.degrees(steer_rad))
        steer_deg = float(np.clip(steer_deg, -self.max_steer, self.max_steer))

        # Smooth steering rate
        steer_deg = 0.80 * steer_deg + 0.20 * self.last_steer
        self.last_steer = steer_deg

        # Curvature-Adaptive Throttle from line_follower_ws:
        # As steering angle approaches max_steer, throttle drops to min_curve_throttle
        steer_ratio = min(1.0, abs(steer_deg) / self.max_steer)
        throttle = self.max_throttle - (self.max_throttle - self.min_curve_throttle) * (steer_ratio ** self.decay_exp)
        throttle = float(np.clip(throttle, self.min_curve_throttle, self.max_throttle))

        self.pub_steer.publish(Float32(data=steer_deg))
        self.pub_throttle.publish(Float32(data=throttle))
        self.get_logger().debug(f"Steer={steer_deg: .2f}°, Throttle={throttle: .2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitCtrl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
