#!/usr/bin/env python3
"""Ackermann kinematics for ENIMIA Black's 2-steer / 1-drive layout.

The ros2_control AckermannSteeringController cannot be used here because it
requires two traction joints.  This node converts ``/cmd_vel`` into commands
for the two front steering joints and the single rear traction joint while the
standard ros2_control forwarding controllers own the Gazebo interfaces.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


class TricycleAckermannController(Node):
    """Command and odometry bridge for the ENIMIA Black tricycle."""

    def __init__(self):
        super().__init__('tricycle_ackermann_controller')

        self.declare_parameter('wheelbase', 1.704)
        self.declare_parameter('front_track', 0.567)
        self.declare_parameter('rear_wheel_radius', 0.18)
        self.declare_parameter('max_steering_angle', 0.52)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.front_track = float(self.get_parameter('front_track').value)
        self.rear_wheel_radius = float(
            self.get_parameter('rear_wheel_radius').value)
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.publish_odom_tf = bool(
            self.get_parameter('publish_odom_tf').value)
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value

        if self.wheelbase <= 0.0 or self.front_track <= 0.0:
            raise ValueError('wheelbase and front_track must be positive')
        if self.rear_wheel_radius <= 0.0 or self.max_steering_angle <= 0.0:
            raise ValueError('rear_wheel_radius and max_steering_angle must be positive')
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')

        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/front_steering_controller/commands', 10)
        self.traction_pub = self.create_publisher(
            Float64MultiArray, '/rear_wheel_velocity_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self._cmd_vel_callback,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self._joint_states_callback,
            10,
        )

        self.command_linear = 0.0
        self.command_angular = 0.0
        self.last_command_time = self.get_clock().now()
        self.have_joint_feedback = False
        self.rear_wheel_velocity = 0.0
        self.left_steering = 0.0
        self.right_steering = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update_time = self.get_clock().now()

        self.create_timer(1.0 / publish_rate, self._update)
        self.get_logger().info(
            'ENIMIA Ackermann bridge ready: /cmd_vel -> front steering + rear drive')

    def _cmd_vel_callback(self, message):
        self.command_linear = message.linear.x
        self.command_angular = message.angular.z
        self.last_command_time = self.get_clock().now()

    def _joint_states_callback(self, message):
        index = {name: i for i, name in enumerate(message.name)}
        required = (
            'rear_wheel_joint',
            'front_left_steer_joint',
            'front_right_steer_joint',
        )
        if not all(name in index for name in required):
            return

        rear_index = index['rear_wheel_joint']
        if rear_index < len(message.velocity):
            self.rear_wheel_velocity = message.velocity[rear_index]
        else:
            return

        left_index = index['front_left_steer_joint']
        right_index = index['front_right_steer_joint']
        if left_index >= len(message.position) or right_index >= len(message.position):
            return
        self.left_steering = message.position[left_index]
        self.right_steering = message.position[right_index]
        self.have_joint_feedback = True

    def _limited_curvature(self, linear_velocity, angular_velocity):
        if abs(linear_velocity) < 1.0e-6:
            return 0.0

        requested_curvature = angular_velocity / linear_velocity
        # The inside front wheel reaches the mechanical stop first.
        minimum_radius = (
            self.wheelbase / math.tan(self.max_steering_angle)
            + self.front_track / 2.0
        )
        maximum_curvature = 1.0 / minimum_radius
        return max(-maximum_curvature, min(maximum_curvature, requested_curvature))

    def _steering_angles(self, curvature):
        if abs(curvature) < 1.0e-9:
            return 0.0, 0.0

        turn_radius = 1.0 / curvature
        left = math.atan(self.wheelbase / (turn_radius - self.front_track / 2.0))
        right = math.atan(self.wheelbase / (turn_radius + self.front_track / 2.0))
        return left, right

    def _update(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_command_time).nanoseconds / 1.0e9
        if elapsed > self.command_timeout:
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            linear_velocity = self.command_linear
            angular_velocity = self.command_angular

        curvature = self._limited_curvature(linear_velocity, angular_velocity)
        left_angle, right_angle = self._steering_angles(curvature)
        self.steering_pub.publish(Float64MultiArray(data=[left_angle, right_angle]))
        self.traction_pub.publish(Float64MultiArray(
            data=[linear_velocity / self.rear_wheel_radius]))

        self._publish_odometry(now)

    def _publish_odometry(self, now):
        dt = (now - self.last_update_time).nanoseconds / 1.0e9
        self.last_update_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        if self.have_joint_feedback:
            linear_velocity = self.rear_wheel_velocity * self.rear_wheel_radius
            # For a true Ackermann pair, averaging tan(delta_left/right) is
            # only an approximation.  The geometric centre radius is exact:
            # R / L = (cot(delta_left) + cot(delta_right)) / 2.
            left_tangent = math.tan(self.left_steering)
            right_tangent = math.tan(self.right_steering)
            if abs(left_tangent) < 1.0e-9 or abs(right_tangent) < 1.0e-9:
                angular_velocity = 0.0
            else:
                curvature = 2.0 / (
                    self.wheelbase * (
                        1.0 / left_tangent + 1.0 / right_tangent))
                angular_velocity = linear_velocity * curvature
        else:
            linear_velocity = 0.0
            angular_velocity = 0.0

        self.x += linear_velocity * math.cos(self.yaw) * dt
        self.y += linear_velocity * math.sin(self.yaw) * dt
        self.yaw += angular_velocity * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        half_yaw = self.yaw / 2.0
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = TricycleAckermannController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ros2 launch may already have shut down the global context after a
        # SIGINT.  Calling shutdown a second time raises RCLError and makes a
        # normal simulator stop look like a controller failure.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
