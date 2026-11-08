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
        self.declare_parameter('stop_topic', '/stop_line/detected')
        self.declare_parameter('stop_distance_topic', '/stop_line/distance')
        self.declare_parameter('brake_active_topic', '/stop_line/braking')
        self.declare_parameter('stop_distance', 2.5)
        self.declare_parameter('stop_sign_topic', '/stop_sign/detected')
        self.declare_parameter('stop_sign_distance_topic', '/stop_sign/distance')
        self.declare_parameter('stop_sign_brake_active_topic', '/stop_sign/braking')
        self.declare_parameter('stop_sign_distance', 2.5)
        self.declare_parameter(
            'localization_recovering_topic', '/localization/recovering')
        self.declare_parameter('pause_during_localization_recovery', True)
        self.declare_parameter('max_brake_decel', 15.0)
        self.declare_parameter('stop_hold_time', 5.0)
        self.declare_parameter('zero_publish_rate', 10.0)

        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.stop_topic = self.get_parameter('stop_topic').value
        self.stop_distance_topic = self.get_parameter('stop_distance_topic').value
        self.brake_active_topic = self.get_parameter('brake_active_topic').value
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.stop_sign_topic = self.get_parameter('stop_sign_topic').value
        self.stop_sign_distance_topic = self.get_parameter(
            'stop_sign_distance_topic').value
        self.stop_sign_brake_active_topic = self.get_parameter(
            'stop_sign_brake_active_topic').value
        self.stop_sign_distance = float(self.get_parameter('stop_sign_distance').value)
        self.localization_recovering_topic = self.get_parameter(
            'localization_recovering_topic').value
        self.pause_during_localization_recovery = bool(self.get_parameter(
            'pause_during_localization_recovery').value)
        self.max_brake_decel = float(self.get_parameter('max_brake_decel').value)
        self.stop_hold_time = float(self.get_parameter('stop_hold_time').value)
        zero_publish_rate = float(self.get_parameter('zero_publish_rate').value)

        if (self.stop_distance < 0.0 or self.stop_sign_distance < 0.0 or
                self.max_brake_decel <= 0.0):
            raise ValueError('stop distances must be >= 0 and max_brake_decel must be > 0')

        self.stop_until = 0.0
        self.stop_signal_active = False
        self.stop_sign_signal_active = False
        self.stop_announced = False
        self.last_stop_distance = math.inf
        self.last_stop_sign_distance = math.inf
        self.brake_active = False
        self.stop_sign_brake_active = False
        self.localization_recovering = False
        self.localization_pause_announced = False

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.create_subscription(Twist, self.input_cmd_topic, self.cmd_callback, 10)
        self.create_subscription(
            Bool, self.stop_topic,
            lambda msg: self.stop_callback(msg, 'yellow line'), 10)
        self.create_subscription(
            Float32, self.stop_distance_topic, self.stop_distance_callback, 10)
        self.create_subscription(
            Bool, self.brake_active_topic, self.brake_active_callback, 10)
        self.create_subscription(
            Bool, self.stop_sign_topic,
            lambda msg: self.stop_callback(msg, 'stop sign'), 10)
        self.create_subscription(
            Float32, self.stop_sign_distance_topic,
            self.stop_sign_distance_callback, 10)
        self.create_subscription(
            Bool, self.stop_sign_brake_active_topic,
            self.stop_sign_brake_active_callback, 10)
        self.create_subscription(
            Bool, self.localization_recovering_topic,
            self.localization_recovering_callback, 10)
        self.create_timer(1.0 / max(zero_publish_rate, 1.0), self.timer_callback)

        self.get_logger().info(
            f'Safety gate: {self.input_cmd_topic} -> {self.output_cmd_topic}, '
            f'yellow line at {self.stop_distance:.2f} m / stop sign at '
            f'{self.stop_sign_distance:.2f} m, hold {self.stop_hold_time:.1f} s'
        )

    def stop_callback(self, msg, source):
        # Latch only the rising edge.  The detector deliberately publishes
        # True for the whole five-second stop, so extending the timer on every
        # message would otherwise keep the vehicle stopped indefinitely.
        active = (self.stop_signal_active if source == 'yellow line'
                  else self.stop_sign_signal_active)
        if msg.data and not active:
            self.stop_until = max(
                self.stop_until, time.monotonic() + self.stop_hold_time)
            if not self.stop_announced:
                self.get_logger().warn(f'{source} stop detected: blocking cmd_vel')
            self.stop_announced = True
        if source == 'yellow line':
            self.stop_signal_active = bool(msg.data)
        else:
            self.stop_sign_signal_active = bool(msg.data)

    def stop_distance_callback(self, msg):
        distance = float(msg.data)
        self.last_stop_distance = distance if math.isfinite(distance) and distance > 0.0 else math.inf

    def brake_active_callback(self, msg):
        self.brake_active = bool(msg.data)

    def stop_sign_distance_callback(self, msg):
        distance = float(msg.data)
        self.last_stop_sign_distance = (
            distance if math.isfinite(distance) and distance > 0.0 else math.inf)

    def stop_sign_brake_active_callback(self, msg):
        self.stop_sign_brake_active = bool(msg.data)

    def localization_recovering_callback(self, msg):
        self.localization_recovering = bool(msg.data)
        if (self.pause_during_localization_recovery and
                self.localization_recovering and not self.localization_pause_announced):
            self.get_logger().warn(
                'AMCL recovery in progress: holding cmd_vel until localization is healthy')
            self.localization_pause_announced = True
        elif not self.localization_recovering:
            self.localization_pause_announced = False

    def is_stopped(self):
        return time.monotonic() < self.stop_until

    def cmd_callback(self, msg):
        if (self.is_stopped() or
                (self.pause_during_localization_recovery and
                 self.localization_recovering)):
            self.cmd_pub.publish(Twist())
            return
        self.cmd_pub.publish(self.apply_stop_braking(msg))

    def apply_stop_braking(self, msg):
        """Limit speed so the car reaches zero at the active stop distance.

        v_max = sqrt(2 * a * remaining_distance) gives a smooth braking
        envelope.  The painted line and stop-sign detector each contribute a
        depth measurement; the strictest active envelope always wins.
        """
        braking_limits = []
        if self.brake_active and math.isfinite(self.last_stop_distance):
            remaining = max(0.0, self.last_stop_distance - self.stop_distance)
            braking_limits.append(math.sqrt(2.0 * self.max_brake_decel * remaining))
        if (self.stop_sign_brake_active and
                math.isfinite(self.last_stop_sign_distance)):
            remaining = max(0.0, self.last_stop_sign_distance - self.stop_sign_distance)
            braking_limits.append(math.sqrt(2.0 * self.max_brake_decel * remaining))

        if not braking_limits:
            return msg

        allowed_speed = min(braking_limits)
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
        if (self.is_stopped() or
                (self.pause_during_localization_recovery and
                 self.localization_recovering)):
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
