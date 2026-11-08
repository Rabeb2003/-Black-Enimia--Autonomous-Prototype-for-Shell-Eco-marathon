#!/usr/bin/env python3
"""
Lane Keeping Controller: Seamlessly blends Nav2 Goal Navigation with Real-Time Lane Centering.

Role:
- Subscribes to Nav2's smoothed velocity commands (`/cmd_vel_safe`).
- Subscribes to real-time lane cross-track error (`/lane_detection/center_error`)
  and preview heading error (`/lane_detection/heading_error`).
- Constrains Nav2's trajectory so the vehicle stays precisely centered in the circuit
  and never exits the track borders.
- Automatically regulates speed on sharp bends and large cross-track deviations to prevent skidding.
- Respects vehicle mechanical Ackermann curvature limits.
- Publishes to `/cmd_vel_lane`.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class LaneKeepingController(Node):
    def __init__(self):
        super().__init__('lane_keeping_controller')

        # Parameters
        self.declare_parameter('input_cmd_topic', '/cmd_vel_safe')
        self.declare_parameter('output_cmd_topic', '/cmd_vel_lane')
        self.declare_parameter('lane_error_topic', '/lane_detection/center_error')
        self.declare_parameter('heading_error_topic', '/lane_detection/heading_error')
        self.declare_parameter('lane_valid_topic', '/lane_detection/valid')
        self.declare_parameter('stop_topic', '/stop_detected')

        self.declare_parameter('target_speed', 10.0)      # Cruise speed if no Nav2 goal active
        self.declare_parameter('max_speed', 15.0)         # Max speed limit
        self.declare_parameter('min_curve_speed', 3.0)    # Floor speed on sharp bends
        self.declare_parameter('max_acceleration', 15.0)  # Acceleration limit
        self.declare_parameter('max_deceleration', 15.0)  # Deceleration limit
        self.declare_parameter('max_curvature', 0.20)     # Max Ackermann curvature (1/m)

        # Control gains
        self.declare_parameter('kp', 0.22)                # Proportional gain for lateral error
        self.declare_parameter('kd', 0.05)                # Derivative gain for lateral damping
        self.declare_parameter('kh', 0.08)                # Anticipatory lookahead heading gain

        self.declare_parameter('lane_timeout', 0.8)       # Seconds before lane signal considered lost
        self.declare_parameter('command_timeout', 0.5)    # Nav2 command freshness timeout
        self.declare_parameter('publish_rate', 30.0)

        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.lane_error_topic = self.get_parameter('lane_error_topic').value
        self.heading_error_topic = self.get_parameter('heading_error_topic').value
        self.lane_valid_topic = self.get_parameter('lane_valid_topic').value
        self.stop_topic = self.get_parameter('stop_topic').value

        self.target_speed = float(self.get_parameter('target_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_curve_speed = float(self.get_parameter('min_curve_speed').value)
        self.max_acceleration = float(self.get_parameter('max_acceleration').value)
        self.max_deceleration = float(self.get_parameter('max_deceleration').value)
        self.max_curvature = float(self.get_parameter('max_curvature').value)

        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.kh = float(self.get_parameter('kh').value)

        self.lane_timeout = float(self.get_parameter('lane_timeout').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        # State tracking
        self.latest_nav_cmd = Twist()
        self.last_nav_command_at = 0.0

        self.lane_error = 0.0
        self.previous_lane_error = 0.0
        self.heading_error = 0.0
        self.last_lane_at = 0.0
        self.lane_valid = False

        self.stop_detected = False
        self.last_output_speed = 0.0
        self.last_output_at = time.monotonic()

        # ROS 2 Subscriptions & Publishers
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.create_subscription(Twist, self.input_cmd_topic, self.nav_cmd_callback, 10)
        self.create_subscription(Float32, self.lane_error_topic, self.lane_error_callback, 10)
        self.create_subscription(Float32, self.heading_error_topic, self.heading_error_callback, 10)
        self.create_subscription(Bool, self.lane_valid_topic, self.lane_valid_callback, 10)
        self.create_subscription(Bool, self.stop_topic, self.stop_callback, 10)

        self.create_timer(1.0 / publish_rate, self.publish_command)

        self.get_logger().info(
            f'Lane Keeping Controller active: {self.input_cmd_topic} + {self.lane_error_topic} -> {self.output_cmd_topic}'
        )

    def nav_cmd_callback(self, msg: Twist):
        self.latest_nav_cmd = msg
        self.last_nav_command_at = time.monotonic()

    def lane_error_callback(self, msg: Float32):
        self.previous_lane_error = self.lane_error
        self.lane_error = float(max(-1.0, min(1.0, msg.data)))

    def heading_error_callback(self, msg: Float32):
        self.heading_error = float(max(-1.0, min(1.0, msg.data)))

    def lane_valid_callback(self, msg: Bool):
        self.lane_valid = bool(msg.data)
        if self.lane_valid:
            self.last_lane_at = time.monotonic()

    def stop_callback(self, msg: Bool):
        self.stop_detected = bool(msg.data)

    def publish_command(self):
        now = time.monotonic()
        command = Twist()

        # Emergency Stop
        if self.stop_detected:
            self.last_output_speed = 0.0
            self.last_output_at = now
            self.cmd_pub.publish(command)
            return

        lane_age = (now - self.last_lane_at) if self.last_lane_at > 0 else math.inf
        nav_fresh = (now - self.last_nav_command_at) <= self.command_timeout
        lane_active = (lane_age <= self.lane_timeout) and self.lane_valid

        # Case 1: Lane Keeping Active (with or without Nav2)
        if lane_active:
            # Determine base speed
            if nav_fresh and self.latest_nav_cmd.linear.x > 0.05:
                # Nav2 is actively navigating: follow its speed intent
                raw_speed = self.latest_nav_cmd.linear.x
                base_curvature = self.latest_nav_cmd.angular.z / max(raw_speed, 0.1)
            else:
                # Nav2 idle or stopped at goal
                if nav_fresh and abs(self.latest_nav_cmd.linear.x) <= 0.05:
                    # Explicit Nav2 stop command (e.g. goal reached): honor stop
                    raw_speed = 0.0
                    base_curvature = 0.0
                else:
                    # Autonomous lane cruise
                    raw_speed = self.target_speed
                    base_curvature = 0.0

            if raw_speed <= 0.05:
                command.linear.x = self.limit_speed(0.0, now)
                command.angular.z = 0.0
                self.cmd_pub.publish(command)
                return

            # Compute lateral steering correction
            # Note sign convention:
            # lane_error > 0 means lane center is to the RIGHT of car -> car must steer RIGHT (negative angular.z)
            # lane_error < 0 means lane center is to the LEFT of car -> car must steer LEFT (positive angular.z)
            d_error = (self.lane_error - self.previous_lane_error)
            correction = - (self.kp * self.lane_error + self.kd * d_error + self.kh * self.heading_error)

            # Blend Nav2 path intent with Lane Centering
            if abs(self.lane_error) > 0.35:
                # Critical off-center condition: prioritize lane recovery over Nav2 path
                blend_weight = min(1.0, (abs(self.lane_error) - 0.35) / 0.35)
                curvature = (1.0 - blend_weight) * base_curvature + correction
            else:
                curvature = base_curvature + correction

            # Bound curvature to vehicle physical Ackermann limit
            curvature = max(-self.max_curvature, min(self.max_curvature, curvature))

            # Dynamic speed regulation on bends and off-center deviations
            curve_ratio = abs(curvature) / self.max_curvature
            off_center_ratio = abs(self.lane_error)
            speed_factor = max(0.2, 1.0 - 0.50 * curve_ratio - 0.35 * off_center_ratio)
            speed = max(self.min_curve_speed, min(raw_speed * speed_factor, self.max_speed))

            command.linear.x = self.limit_speed(speed, now)
            command.angular.z = command.linear.x * curvature

        # Case 2: Perception temporarily degraded -> Fall back smoothly to Nav2
        elif nav_fresh:
            command = self.latest_nav_cmd
            command.linear.x = self.limit_speed(min(max(command.linear.x, 0.0), self.max_speed), now)

        # Case 3: Neither Lane nor Nav2 available -> Controlled stop
        else:
            command.linear.x = self.limit_speed(0.0, now)
            command.angular.z = 0.0

        self.cmd_pub.publish(command)

    def limit_speed(self, target_speed: float, now: float) -> float:
        """Enforce acceleration and deceleration boundaries smoothly."""
        dt = max(0.001, now - self.last_output_at)
        if target_speed >= self.last_output_speed:
            allowed_delta = self.max_acceleration * dt
            speed = min(target_speed, self.last_output_speed + allowed_delta)
        else:
            allowed_delta = self.max_deceleration * dt
            speed = max(target_speed, self.last_output_speed - allowed_delta)

        self.last_output_speed = speed
        self.last_output_at = now
        return speed


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneKeepingController()
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
