#!/usr/bin/env python3
"""
Odometry Drift Visualizer

LEARNING OBJECTIVES:
- Understand odometry drift over time
- See the difference between raw odom and EKF-filtered odom
- Visualize why sensor fusion matters

WHAT THIS DOES:
1. Subscribes to both raw odom and filtered odom
2. Tracks the cumulative difference
3. Prints drift statistics

USAGE:
    ros2 run turtlebot_demos odom_drift_visualizer

Drive the robot in a loop back to start.
Compare where raw odom thinks you are vs reality.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class DriftVisualizer(Node):
    def __init__(self):
        super().__init__('odom_drift_visualizer')

        # Store odometry data
        self.raw_odom = None
        self.filtered_odom = None
        self.start_raw = None
        self.start_filtered = None

        # Subscribers
        self.raw_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.raw_callback,
            10
        )
        self.filtered_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.filtered_callback,
            10
        )

        # Timer to print stats
        self.timer = self.create_timer(2.0, self.print_stats)

        self.get_logger().info('=' * 50)
        self.get_logger().info('ODOMETRY DRIFT VISUALIZER')
        self.get_logger().info('=' * 50)
        self.get_logger().info('')
        self.get_logger().info('Comparing:')
        self.get_logger().info('  Raw:      /diff_drive_controller/odom')
        self.get_logger().info('  Filtered: /odometry/filtered')
        self.get_logger().info('')
        self.get_logger().info('Drive the robot around and return to start.')
        self.get_logger().info('Watch how drift accumulates in raw odom.')
        self.get_logger().info('')

    def raw_callback(self, msg: Odometry):
        self.raw_odom = msg
        if self.start_raw is None:
            self.start_raw = msg
            self.get_logger().info('Raw odom start recorded')

    def filtered_callback(self, msg: Odometry):
        self.filtered_odom = msg
        if self.start_filtered is None:
            self.start_filtered = msg
            self.get_logger().info('Filtered odom start recorded')

    def print_stats(self):
        if self.raw_odom is None or self.filtered_odom is None:
            return

        # Current positions
        raw_x = self.raw_odom.pose.pose.position.x
        raw_y = self.raw_odom.pose.pose.position.y
        filt_x = self.filtered_odom.pose.pose.position.x
        filt_y = self.filtered_odom.pose.pose.position.y

        # Distance from start (raw)
        raw_dist = math.sqrt(
            (raw_x - self.start_raw.pose.pose.position.x) ** 2 +
            (raw_y - self.start_raw.pose.pose.position.y) ** 2
        )

        # Distance from start (filtered)
        filt_dist = math.sqrt(
            (filt_x - self.start_filtered.pose.pose.position.x) ** 2 +
            (filt_y - self.start_filtered.pose.pose.position.y) ** 2
        )

        # Difference between raw and filtered
        diff = math.sqrt((raw_x - filt_x) ** 2 + (raw_y - filt_y) ** 2)

        self.get_logger().info('-' * 40)
        self.get_logger().info(f'Raw position:      ({raw_x:+.3f}, {raw_y:+.3f})')
        self.get_logger().info(f'Filtered position: ({filt_x:+.3f}, {filt_y:+.3f})')
        self.get_logger().info(f'Distance from start (raw):      {raw_dist:.3f} m')
        self.get_logger().info(f'Distance from start (filtered): {filt_dist:.3f} m')
        self.get_logger().info(f'Raw vs Filtered difference:     {diff:.3f} m')

        if diff > 0.1:
            self.get_logger().warn(
                f'DRIFT DETECTED: {diff:.3f}m difference!'
            )


def main():
    rclpy.init()
    node = DriftVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Visualizer stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
