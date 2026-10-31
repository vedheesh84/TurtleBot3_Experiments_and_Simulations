#!/usr/bin/env python3
"""
TF Breaker - Failure Injection for Learning

LEARNING OBJECTIVES:
- Understand TF dependency chain
- See how missing transforms break navigation
- Practice TF debugging with tf2_echo and view_frames

WHAT THIS DOES:
Publishes a broken TF transform that disrupts the TF tree.
Options:
1. Stop publishing odom->base_link (EKF failure)
2. Publish with wrong parent frame
3. Publish with large delay (stale transform)

SYMPTOMS YOU'LL SEE:
- "Could not transform" errors
- Navigation fails to start
- Robot position jumps in RViz

HOW TO FIX:
1. ros2 run tf2_tools view_frames
2. ros2 run tf2_ros tf2_echo map base_link
3. Find the broken link in the tree
"""
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import time


class TFBreaker(Node):
    def __init__(self, cli_mode: str = 'duplicate'):
        super().__init__('tf_breaker')
        self.declare_parameter('mode', cli_mode)
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.br = TransformBroadcaster(self)

        self.get_logger().warn('=' * 50)
        self.get_logger().warn('TF BREAKER ACTIVE - LEARNING MODE')
        self.get_logger().warn(f'Mode: {self.mode}')
        self.get_logger().warn('=' * 50)

        if self.mode == 'wrong_parent':
            self.timer = self.create_timer(0.02, self.publish_wrong_parent)
            self.get_logger().error(
                'Publishing odom->base_link with WRONG parent (base_link->odom)'
            )
            self.get_logger().info(
                'SYMPTOM: TF tree will have a loop or disconnection'
            )
        elif self.mode == 'stale':
            self.timer = self.create_timer(0.02, self.publish_stale)
            self.get_logger().error(
                'Publishing odom->base_link with 5 SECOND DELAY'
            )
            self.get_logger().info(
                'SYMPTOM: Extrapolation errors, robot appears in wrong place'
            )
        elif self.mode == 'duplicate':
            self.timer = self.create_timer(0.02, self.publish_duplicate)
            self.get_logger().error(
                'Publishing DUPLICATE odom->base_link (conflicts with EKF)'
            )
            self.get_logger().info(
                'SYMPTOM: Robot position flickers between two sources'
            )

    def publish_wrong_parent(self):
        """Publish with swapped parent/child (creates invalid tree)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # WRONG: should be 'odom'
        t.child_frame_id = 'odom'        # WRONG: should be 'base_link'
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

    def publish_stale(self):
        """Publish with old timestamp (causes extrapolation errors)."""
        t = TransformStamped()
        # 5 seconds in the past
        now = self.get_clock().now()
        stale_time = rclpy.time.Time(
            nanoseconds=now.nanoseconds - 5_000_000_000
        )
        t.header.stamp = stale_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link_stale'  # Different name to not conflict
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

    def publish_duplicate(self):
        """Publish competing odom->base_link (conflicts with EKF)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        # Offset position to make conflict visible
        t.transform.translation.x = 1.0  # 1 meter offset
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)


def main():
    parser = argparse.ArgumentParser(
        description='TF Breaker - Learn TF debugging by breaking things'
    )
    parser.add_argument(
        '--mode', '-m',
        choices=['wrong_parent', 'stale', 'duplicate'],
        default='duplicate',
        help='Type of TF breakage to inject'
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = TFBreaker(args.mode)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('TF Breaker stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
