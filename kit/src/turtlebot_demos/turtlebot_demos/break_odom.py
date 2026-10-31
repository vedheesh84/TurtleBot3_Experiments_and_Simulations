#!/usr/bin/env python3
"""
Odometry Breaker - Failure Injection for Learning

LEARNING OBJECTIVES:
- Understand odometry's role in localization
- See how bad odometry affects navigation
- Practice odometry debugging

WHAT THIS DOES:
Corrupts odometry data in various ways:
1. Add drift to position
2. Add noise to velocity
3. Stop publishing entirely
4. Publish with wrong frame

SYMPTOMS YOU'LL SEE:
- Robot drifts in RViz while stationary
- EKF produces erratic estimates
- Navigation fails or takes weird paths

HOW TO DEBUG:
1. ros2 topic echo /odometry/filtered
2. ros2 topic hz /diff_drive_controller/odom
3. Compare raw odom vs filtered odom
"""
import argparse
import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomBreaker(Node):
    def __init__(self, cli_mode: str = 'drift', cli_magnitude: float = 0.1):
        super().__init__('odom_breaker')
        self.declare_parameter('mode', cli_mode)
        self.declare_parameter('magnitude', cli_magnitude)
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.magnitude = self.get_parameter('magnitude').get_parameter_value().double_value
        self.drift_x = 0.0
        self.drift_y = 0.0

        self.get_logger().warn('=' * 50)
        self.get_logger().warn('ODOMETRY BREAKER ACTIVE - LEARNING MODE')
        self.get_logger().warn(f'Mode: {self.mode}, Magnitude: {self.magnitude}')
        self.get_logger().warn('=' * 50)

        # Subscribe to real odom, publish corrupted version
        self.sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )
        self.pub = self.create_publisher(
            Odometry,
            '/odom_corrupted',
            10
        )

        self.get_logger().info(
            'Subscribe to /diff_drive_controller/odom'
        )
        self.get_logger().info(
            'Publishing corrupted odom to /odom_corrupted'
        )
        self.get_logger().info('')
        self.get_logger().info(
            'To see the effect, remap EKF input:'
        )
        self.get_logger().info(
            '  odom0: /odom_corrupted'
        )

    def odom_callback(self, msg: Odometry):
        corrupted = Odometry()
        corrupted.header = msg.header
        corrupted.child_frame_id = msg.child_frame_id

        if self.mode == 'drift':
            # Accumulating drift (simulates encoder slip)
            self.drift_x += self.magnitude * 0.001
            self.drift_y += self.magnitude * 0.0005
            corrupted.pose.pose.position.x = msg.pose.pose.position.x + self.drift_x
            corrupted.pose.pose.position.y = msg.pose.pose.position.y + self.drift_y
            corrupted.pose.pose.position.z = msg.pose.pose.position.z
            corrupted.pose.pose.orientation = msg.pose.pose.orientation
            corrupted.twist = msg.twist

        elif self.mode == 'noise':
            # Random noise on velocity (simulates encoder noise)
            corrupted.pose = msg.pose
            corrupted.twist.twist.linear.x = msg.twist.twist.linear.x + \
                random.gauss(0, self.magnitude)
            corrupted.twist.twist.linear.y = msg.twist.twist.linear.y
            corrupted.twist.twist.linear.z = msg.twist.twist.linear.z
            corrupted.twist.twist.angular.x = msg.twist.twist.angular.x
            corrupted.twist.twist.angular.y = msg.twist.twist.angular.y
            corrupted.twist.twist.angular.z = msg.twist.twist.angular.z + \
                random.gauss(0, self.magnitude * 0.5)

        elif self.mode == 'wrong_frame':
            # Wrong frame_id (causes TF lookup failures)
            corrupted = msg
            corrupted.header.frame_id = 'wrong_odom_frame'
            corrupted.child_frame_id = 'wrong_base_frame'

        elif self.mode == 'stuck':
            # Position never changes (simulates encoder failure)
            corrupted.pose.pose.position.x = 0.0
            corrupted.pose.pose.position.y = 0.0
            corrupted.pose.pose.position.z = 0.0
            corrupted.pose.pose.orientation.w = 1.0
            corrupted.twist.twist.linear.x = 0.0
            corrupted.twist.twist.angular.z = 0.0

        self.pub.publish(corrupted)


def main():
    parser = argparse.ArgumentParser(
        description='Odometry Breaker - Learn odometry debugging'
    )
    parser.add_argument(
        '--mode', '-m',
        choices=['drift', 'noise', 'wrong_frame', 'stuck'],
        default='drift',
        help='Type of odometry corruption'
    )
    parser.add_argument(
        '--magnitude', '-g',
        type=float,
        default=0.1,
        help='Magnitude of corruption'
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = OdomBreaker(args.mode, args.magnitude)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Odom Breaker stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
