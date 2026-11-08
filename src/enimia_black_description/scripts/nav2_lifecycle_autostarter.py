#!/usr/bin/env python3
import threading
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class LifecycleAutostarter(Node):
    def __init__(self):
        super().__init__('nav2_lifecycle_autostarter')

        self.declare_parameter('node_names', [''])
        self.declare_parameter('required_topics', [''])
        self.declare_parameter('required_transforms', [''])
        self.declare_parameter('lifecycle_manager', '')
        self.declare_parameter('initial_delay', 5.0)
        self.declare_parameter('service_timeout', 90.0)
        self.declare_parameter('retry_delay', 3.0)
        self.declare_parameter('max_attempts', 5)

        self.node_names = [
            str(name) for name in self.get_parameter('node_names').value
            if str(name)
        ]
        self.required_topics = [
            str(name) for name in self.get_parameter('required_topics').value
            if str(name)
        ]
        self.required_transforms = [
            str(transform) for transform in
            self.get_parameter('required_transforms').value
            if str(transform)
        ]
        self.lifecycle_manager = str(
            self.get_parameter('lifecycle_manager').value).strip('/')
        self.initial_delay = float(self.get_parameter('initial_delay').value)
        self.service_timeout = float(self.get_parameter('service_timeout').value)
        self.retry_delay = float(self.get_parameter('retry_delay').value)
        self.max_attempts = int(self.get_parameter('max_attempts').value)
        self.cancelled = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'Lifecycle readiness gate will start '
            f'{self.lifecycle_manager or self.node_names} after '
            f'{self.initial_delay:.1f}s'
        )
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def run(self):
        time.sleep(max(0.0, self.initial_delay))
        if self.required_topics and not self.wait_for_topics():
            self.get_logger().error(f'Missing required topics: {self.required_topics}')
            return
        if self.required_transforms and not self.wait_for_transforms():
            self.get_logger().error(
                f'Missing required transforms: {self.required_transforms}')
            return
        if self.lifecycle_manager:
            if self.start_lifecycle_manager():
                self.get_logger().info('Lifecycle manager startup complete')
            else:
                self.get_logger().error(
                    f'Could not start lifecycle manager {self.lifecycle_manager}')
            return
        for node_name in self.node_names:
            if self.cancelled or not rclpy.ok():
                return
            if not self.bringup_node(node_name):
                self.get_logger().error(f'Could not activate lifecycle node {node_name}')
                return
        self.get_logger().info('Lifecycle autostart complete')

    def wait_for_topics(self):
        self.get_logger().info(f'Waiting for topics before lifecycle activation: {self.required_topics}')
        deadline = time.monotonic() + self.service_timeout
        next_log = 0.0
        while rclpy.ok() and not self.cancelled and time.monotonic() < deadline:
            available = {name for name, _ in self.get_topic_names_and_types()}
            missing = [topic for topic in self.required_topics if topic not in available]
            if not missing:
                self.get_logger().info('Required topics are available')
                return True
            if time.monotonic() >= next_log:
                self.get_logger().info(f'Waiting for topics: {missing}')
                next_log = time.monotonic() + 5.0
            time.sleep(1.0)
        return False

    def wait_for_transforms(self):
        self.get_logger().info(
            f'Waiting for transforms before lifecycle activation: '
            f'{self.required_transforms}')
        deadline = time.monotonic() + self.service_timeout
        next_log = 0.0
        while rclpy.ok() and not self.cancelled and time.monotonic() < deadline:
            missing = []
            for transform in self.required_transforms:
                try:
                    target_frame, source_frame = [
                        frame.strip() for frame in transform.split('->', 1)]
                except ValueError:
                    self.get_logger().error(
                        f'Invalid transform {transform!r}; use target->source')
                    return False
                if not target_frame or not source_frame:
                    self.get_logger().error(
                        f'Invalid transform {transform!r}; frames cannot be empty')
                    return False
                if not self.tf_buffer.can_transform(
                        target_frame, source_frame, Time(),
                        timeout=Duration(seconds=0.1)):
                    missing.append(transform)
            if not missing:
                self.get_logger().info('Required transforms are available')
                return True
            if time.monotonic() >= next_log:
                self.get_logger().info(f'Waiting for transforms: {missing}')
                next_log = time.monotonic() + 5.0
            time.sleep(1.0)
        return False

    def start_lifecycle_manager(self):
        client = self.create_client(
            ManageLifecycleNodes,
            f'/{self.lifecycle_manager}/manage_nodes')
        if not self.wait_for_client(
                client, self.lifecycle_manager, 'manage_nodes'):
            return False
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = client.call_async(request)
        if not self.wait_for_future(future):
            return False
        return bool(future.result().success)

    def bringup_node(self, node_name):
        get_state = self.create_client(GetState, f'/{node_name}/get_state')
        change_state = self.create_client(ChangeState, f'/{node_name}/change_state')

        if not self.wait_for_client(get_state, node_name, 'get_state'):
            return False
        if not self.wait_for_client(change_state, node_name, 'change_state'):
            return False

        for attempt in range(1, self.max_attempts + 1):
            state = self.get_state(get_state)
            if state == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(f'{node_name} is already active')
                return True

            if state == State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().info(f'Configuring {node_name} (attempt {attempt})')
                if not self.change_state(change_state, Transition.TRANSITION_CONFIGURE):
                    time.sleep(self.retry_delay)
                    continue
                state = self.wait_for_primary_state(get_state)

            if state == State.PRIMARY_STATE_INACTIVE:
                self.get_logger().info(f'Activating {node_name} (attempt {attempt})')
                if not self.change_state(change_state, Transition.TRANSITION_ACTIVATE):
                    time.sleep(self.retry_delay)
                    continue
                state = self.wait_for_primary_state(get_state)

            if state == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(f'{node_name} active')
                return True

            self.get_logger().warn(
                f'{node_name} state {state} is not ready; retrying in {self.retry_delay:.1f}s'
            )
            time.sleep(self.retry_delay)

        return False

    def wait_for_client(self, client, node_name, service_name):
        deadline = time.monotonic() + self.service_timeout
        while rclpy.ok() and not self.cancelled and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                return True
        self.get_logger().error(f'Timeout waiting for /{node_name}/{service_name}')
        return False

    def get_state(self, client):
        future = client.call_async(GetState.Request())
        if not self.wait_for_future(future):
            return State.PRIMARY_STATE_UNKNOWN
        return future.result().current_state.id

    def change_state(self, client, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        if not self.wait_for_future(future):
            return False
        return bool(future.result().success)

    def wait_for_primary_state(self, get_state_client):
        deadline = time.monotonic() + self.service_timeout
        state = State.PRIMARY_STATE_UNKNOWN
        while rclpy.ok() and not self.cancelled and time.monotonic() < deadline:
            state = self.get_state(get_state_client)
            if state in (
                State.PRIMARY_STATE_UNCONFIGURED,
                State.PRIMARY_STATE_INACTIVE,
                State.PRIMARY_STATE_ACTIVE,
                State.PRIMARY_STATE_FINALIZED,
            ):
                return state
            time.sleep(0.2)
        return state

    def wait_for_future(self, future):
        deadline = time.monotonic() + self.service_timeout
        while rclpy.ok() and not self.cancelled and time.monotonic() < deadline:
            if future.done():
                return future.result() is not None
            time.sleep(0.05)
        return False

    def destroy_node(self):
        self.cancelled = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleAutostarter()
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
