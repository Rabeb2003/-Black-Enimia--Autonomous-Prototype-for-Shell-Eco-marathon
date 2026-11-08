#!/usr/bin/env python3
"""
Steering Joint Synchronization Node
Synchronizes the right steering joint with the left steering joint
for tricycle drive configuration where only one steering joint is controlled.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SteeringSync(Node):
    def __init__(self):
        super().__init__('steering_sync')
        
        # Parameters
        self.declare_parameter('left_steer_joint', 'front_left_steer_joint')
        self.declare_parameter('right_steer_joint', 'front_right_steer_joint')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('publish_rate', 50.0)
        
        self.left_steer_joint = self.get_parameter('left_steer_joint').value
        self.right_steer_joint = self.get_parameter('right_steer_joint').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # State
        self.left_steer_position = 0.0
        
        # Subscriber to joint states
        self.joint_states_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_states_callback,
            10
        )
        
        # Publisher for synchronized joint states
        self.joint_states_pub = self.create_publisher(
            JointState,
            '/joint_states_sync',
            10
        )
        
        # Timer for publishing synchronized states
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_synced_states
        )
        
        self.get_logger().info(
            f'Steering Sync: {self.left_steer_joint} -> {self.right_steer_joint}'
        )
    
    def joint_states_callback(self, msg):
        """Read left steering joint position"""
        try:
            idx = msg.name.index(self.left_steer_joint)
            self.left_steer_position = msg.position[idx]
        except (ValueError, IndexError):
            pass
    
    def publish_synced_states(self):
        """Publish synchronized steering joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.right_steer_joint]
        msg.position = [self.left_steer_position]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        
        self.joint_states_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringSync()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
