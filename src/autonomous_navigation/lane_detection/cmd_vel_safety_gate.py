#!/usr/bin/env python3
import time
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class CmdVelSafetyGate(Node):
    def __init__(self):
        super().__init__('cmd_vel_safety_gate')

        self.declare_parameter('input_cmd_topic', '/cmd_vel_lane')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('stop_topic', '/stop_detected')
        self.declare_parameter('stop_distance_topic', '/stop_sign/distance')
        self.declare_parameter('brake_active_topic', '/stop_sign/braking')
        self.declare_parameter('stop_distance', 2.5)
        self.declare_parameter('max_brake_decel', 15.0)
        self.declare_parameter('stop_hold_time', 5.0)
        self.declare_parameter('zero_publish_rate', 10.0)

        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.stop_topic = self.get_parameter('stop_topic').value
        self.stop_distance_topic = self.get_parameter('stop_distance_topic').value
        self.brake_active_topic = self.get_parameter('brake_active_topic').value
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.max_brake_decel = float(self.get_parameter('max_brake_decel').value)
        self.stop_hold_time = float(self.get_parameter('stop_hold_time').value)
        zero_publish_rate = float(self.get_parameter('zero_publish_rate').value)

        if self.stop_distance < 0.0 or self.max_brake_decel <= 0.0:
            raise ValueError('stop_distance must be >= 0 and max_brake_decel must be > 0')

        self.stop_until = 0.0
        self.stop_signal_active = False
        self.stop_announced = False
        self.last_stop_distance = math.inf
        self.brake_active = False

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.create_subscription(Twist, self.input_cmd_topic, self.cmd_callback, 10)
        self.create_subscription(Bool, self.stop_topic, self.stop_callback, 10)
        self.create_subscription(
            Float32, self.stop_distance_topic, self.stop_distance_callback, 10)
        self.create_subscription(
            Bool, self.brake_active_topic, self.brake_active_callback, 10)
        self.create_timer(1.0 / max(zero_publish_rate, 1.0), self.timer_callback)

        self.get_logger().info(
            f'Safety gate: {self.input_cmd_topic} -> {self.output_cmd_topic}, '
            f'stop at {self.stop_distance:.2f} m, then hold {self.stop_hold_time:.1f} s'
        )

    def stop_callback(self, msg):
        # Latch only the rising edge.  The detector deliberately publishes
        # True for the whole five-second stop, so extending the timer on every
        # message would otherwise keep the vehicle stopped indefinitely.
        if msg.data and not self.stop_signal_active:
            self.stop_until = time.monotonic() + self.stop_hold_time
            if not self.stop_announced:
                self.get_logger().warn('Stop detected: blocking cmd_vel')
            self.stop_announced = True
        self.stop_signal_active = bool(msg.data)

    def stop_distance_callback(self, msg):
        distance = float(msg.data)
        self.last_stop_distance = distance if math.isfinite(distance) and distance > 0.0 else math.inf

    def brake_active_callback(self, msg):
        self.brake_active = bool(msg.data)

    def is_stopped(self):
        return time.monotonic() < self.stop_until

    def cmd_callback(self, msg):
        if self.is_stopped():
            self.cmd_pub.publish(Twist())
            return
        self.cmd_pub.publish(self.apply_stop_braking(msg))

    def apply_stop_braking(self, msg):
        """Limit the speed so that the car reaches zero at stop_distance.

        v_max = sqrt(2 * a * remaining_distance) gives a smooth braking
        envelope.  At 15 m/s with 15 m/s² braking it starts around 10 m from
        the panel and is zero at 2.5 m, rather than cutting power too late.
        """
        if not self.brake_active or not math.isfinite(self.last_stop_distance):
            return msg

        remaining = max(0.0, self.last_stop_distance - self.stop_distance)
        allowed_speed = math.sqrt(2.0 * self.max_brake_decel * remaining)
        output = Twist()
        output.linear = msg.linear
        output.angular = msg.angular
        if msg.linear.x > allowed_speed:
            scale = allowed_speed / msg.linear.x if msg.linear.x > 1.0e-6 else 0.0
            output.linear.x = allowed_speed
            # Preserve the requested curvature while reducing yaw rate.
            output.angular.z = msg.angular.z * scale
        return output

    def timer_callback(self):
        if self.is_stopped():
            self.cmd_pub.publish(Twist())
            return
        if self.stop_announced:
            self.get_logger().info('Stop hold complete: passing cmd_vel again')
            self.stop_announced = False


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
