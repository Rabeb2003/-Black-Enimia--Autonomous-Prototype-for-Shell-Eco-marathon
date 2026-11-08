#!/usr/bin/env python3
"""Start RViz only after the navigation and robot TF tree is complete.

Gazebo Classic may need a long time to insert the SEM world robot.  Opening
RViz before AMCL, the EKF and joint_state_broadcaster are ready produces
misleading "map does not exist" and missing-wheel transform errors.  This
small launch-managed gate exits successfully once all required frames can be
resolved.  The launch file then starts RViz as its own visible process.
"""

import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class RvizTfGate(Node):
    def __init__(self):
        super().__init__('rviz_tf_gate')

        self.declare_parameter('required_transforms', ['map->base_footprint'])
        self.declare_parameter('log_period', 5.0)

        self.required_transforms = [
            str(transform) for transform in
            self.get_parameter('required_transforms').value if str(transform)
        ]
        self.log_period = float(self.get_parameter('log_period').value)
        self.last_log = 0.0
        self.ready = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.check_transforms)
        self.get_logger().info(
            'RViz will open after these transforms are available: '
            f'{self.required_transforms}')

    @staticmethod
    def split_transform(transform):
        try:
            target_frame, source_frame = [
                frame.strip() for frame in transform.split('->', 1)]
        except ValueError as error:
            raise ValueError(
                f'Invalid transform {transform!r}; use target->source') from error
        if not target_frame or not source_frame:
            raise ValueError(
                f'Invalid transform {transform!r}; frames cannot be empty')
        return target_frame, source_frame

    def check_transforms(self):
        if self.ready:
            return

        missing = []
        for transform in self.required_transforms:
            try:
                target_frame, source_frame = self.split_transform(transform)
            except ValueError as error:
                self.get_logger().error(str(error))
                self.timer.cancel()
                rclpy.shutdown()
                return

            if not self.tf_buffer.can_transform(
                    target_frame, source_frame, Time(),
                    timeout=Duration(seconds=0.0)):
                missing.append(transform)

        if missing:
            now = time.monotonic()
            if now - self.last_log >= self.log_period:
                self.get_logger().info(f'Waiting for TF before opening RViz: {missing}')
                self.last_log = now
            return

        self.ready = True
        self.timer.cancel()
        self.get_logger().info('TF tree is ready; requesting RViz2 launch.')
        # The launch file observes this clean exit and starts RViz as a real
        # ExecuteProcess, which preserves RViz diagnostics and process status.
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RvizTfGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    # ``rclpy.shutdown()`` may cause spin() either to return normally or to
    # raise ExternalShutdownException.  In both cases, readiness is the
    # authoritative result consumed by the launch OnProcessExit handler.
    return 0 if node.ready else 1


if __name__ == '__main__':
    raise SystemExit(main())
