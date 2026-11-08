#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fused Symmetrical Path Planner with Single-Lane Extrapolation for lane_follower.
Key Improvements:
1) Evaluates BOTH left and right polynomials symmetrically.
2) Single-Lane Recovery in curves: if inside/outside lane exits the camera FOV,
   extrapolates the vehicle centerline using track width W (real_shift_distance * 2).
3) Normal curve offset: accounts for track curvature theta = arctan(R') in bends.
4) Publishes /path/lookahead_point (Point32: x=lateral [m], y=forward lookahead [m])
"""
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Point32

class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")

        self.declare_parameter("shift_distance", 0.425)  # Half track width (m)
        self.declare_parameter("lookahead_dist", 0.85)   # Default lookahead forward distance (m)

        self.real_shift_distance = float(self.get_parameter("shift_distance").value)
        self.nominal_la = float(self.get_parameter("lookahead_dist").value)
        self.x_la = self.nominal_la

        self.left_fit = None
        self.right_fit = None

        self.create_subscription(Float32MultiArray, "/lane/left_fit",
                                lambda m: setattr(self, "left_fit", m.data), 1)
        self.create_subscription(Float32MultiArray, "/lane/right_fit",
                                lambda m: setattr(self, "right_fit", m.data), 1)

        # Optional dynamic lookahead subscriber
        self.create_subscription(Float32, "/path/target_lookahead",
                                lambda m: setattr(self, "x_la", float(m.data)), 1)

        self.pub_la = self.create_publisher(Point32, "/path/lookahead_point", 1)
        self.timer = self.create_timer(1/30, self.loop)

    def eval_poly(self, coeffs, y):
        # Horner's method for arbitrary degree
        val = 0.0
        for c in coeffs:
            val = val * y + c
        return val

    def eval_poly_deriv(self, coeffs, y):
        n = len(coeffs)
        if n <= 1:
            return 0.0
        val = 0.0
        for i, c in enumerate(coeffs[:-1]):
            power = n - 1 - i
            val += c * power * (y ** (power - 1))
        return val

    def loop(self):
        p = Point32()
        p.z = 0.0
        y_eval = self.x_la
        p.y = y_eval

        # Case 1: Both Left and Right Lanes Detected -> True Centerline
        if self.left_fit is not None and self.right_fit is not None:
            x_l = self.eval_poly(self.left_fit, y_eval)
            x_r = self.eval_poly(self.right_fit, y_eval)
            p.x = float((x_l + x_r) / 2.0)
            self.pub_la.publish(p)

        # Case 2: Only Right Lane Detected (e.g. sharp left turn or right side tracking)
        elif self.right_fit is not None:
            x_r = self.eval_poly(self.right_fit, y_eval)
            r_prime = self.eval_poly_deriv(self.right_fit, y_eval)
            theta = np.arctan(r_prime)
            # Offset perpendicular to tangent
            y_shift = self.real_shift_distance * np.cos(theta)
            p.x = float(x_r + y_shift)
            self.pub_la.publish(p)

        # Case 3: Only Left Lane Detected (e.g. sharp right turn or left side tracking)
        elif self.left_fit is not None:
            x_l = self.eval_poly(self.left_fit, y_eval)
            l_prime = self.eval_poly_deriv(self.left_fit, y_eval)
            theta = np.arctan(l_prime)
            # Offset perpendicular to tangent towards right
            y_shift = -self.real_shift_distance * np.cos(theta)
            p.x = float(x_l + y_shift)
            self.pub_la.publish(p)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
