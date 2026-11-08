#!/usr/bin/env python3
import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class Nav2WaypointGoalSender(Node):
    def __init__(self):
        super().__init__('nav2_waypoint_goal_sender')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('initial_delay', 8.0)
        self.declare_parameter('waypoint_stride', 5)
        self.declare_parameter('min_spacing', 1.5)
        self.declare_parameter('max_waypoints', 200)
        self.declare_parameter(
            'required_active_nodes',
            ['controller_server', 'planner_server', 'bt_navigator'],
        )
        self.declare_parameter('wait_for_active_timeout', 180.0)

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.waypoint_stride = max(1, int(self.get_parameter('waypoint_stride').value))
        self.min_spacing = max(0.0, float(self.get_parameter('min_spacing').value))
        self.max_waypoints = max(1, int(self.get_parameter('max_waypoints').value))
        self.required_active_nodes = list(self.get_parameter('required_active_nodes').value)
        self.wait_for_active_timeout = float(self.get_parameter('wait_for_active_timeout').value)

        self.client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.sent = False
        delay = max(0.1, float(self.get_parameter('initial_delay').value))
        self.timer = self.create_timer(delay, self.send_once)

    def send_once(self):
        if self.sent:
            return
        self.sent = True
        self.timer.cancel()

        poses = self.load_poses()
        if not poses:
            self.get_logger().error('No waypoint poses loaded; navigation goal not sent')
            return

        if not self.wait_for_nav2_active():
            self.get_logger().error('Nav2 did not become active; navigation goal not sent')
            return

        self.get_logger().info('Waiting for Nav2 NavigateThroughPoses action server...')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 action server navigate_through_poses is unavailable')
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        self.get_logger().info(f'Sending {len(poses)} waypoint poses to Nav2')
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def wait_for_nav2_active(self):
        if not self.required_active_nodes:
            return True

        self.get_logger().info(f'Waiting for active Nav2 nodes: {self.required_active_nodes}')
        deadline = time.monotonic() + self.wait_for_active_timeout
        clients = {
            node_name: self.create_client(GetState, f'/{node_name}/get_state')
            for node_name in self.required_active_nodes
        }

        while rclpy.ok() and time.monotonic() < deadline:
            all_active = True
            for node_name, client in clients.items():
                if not client.wait_for_service(timeout_sec=1.0):
                    all_active = False
                    break
                future = client.call_async(GetState.Request())
                wait_deadline = time.monotonic() + 2.0
                while rclpy.ok() and not future.done():
                    if time.monotonic() > wait_deadline:
                        break
                    time.sleep(0.05)
                if not future.done() or future.result().current_state.id != State.PRIMARY_STATE_ACTIVE:
                    all_active = False
                    break
            if all_active:
                self.get_logger().info('Nav2 lifecycle nodes are active')
                return True
            time.sleep(1.0)

        return False

    def load_poses(self):
        if not self.waypoints_file or not os.path.exists(self.waypoints_file):
            self.get_logger().error(f'Waypoints file not found: {self.waypoints_file}')
            return []

        raw = []
        with open(self.waypoints_file, newline='') as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                try:
                    raw.append((
                        float(row.get('x', '0')),
                        float(row.get('y', '0')),
                        float(row.get('yaw', row.get('theta', '0'))),
                    ))
                except ValueError:
                    continue

        filtered = []
        last_xy = None
        for index, (x, y, yaw) in enumerate(raw):
            if index % self.waypoint_stride != 0 and index != len(raw) - 1:
                continue
            if last_xy is not None:
                distance = math.hypot(x - last_xy[0], y - last_xy[1])
                if distance < self.min_spacing and index != len(raw) - 1:
                    continue
            filtered.append((x, y, yaw))
            last_xy = (x, y)
            if len(filtered) >= self.max_waypoints:
                break

        return [self.make_pose(x, y, yaw) for x, y, yaw in filtered]

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the waypoint goal')
            return
        self.get_logger().info('Nav2 accepted the waypoint goal')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Nav2 feedback: '
            f'{feedback.number_of_poses_remaining} poses remaining, '
            f'{feedback.distance_remaining:.1f} m remaining',
            throttle_duration_sec=2.0,
        )

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Nav2 waypoint navigation finished with result: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2WaypointGoalSender()
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
