#!/usr/bin/env python3
"""
Costmap Breaker - Failure Injection for Learning

LEARNING OBJECTIVES:
- Understand costmap layers and their roles
- See how costmap issues affect planning
- Practice navigation debugging

WHAT THIS DOES:
Publishes fake obstacles or clears real ones:
1. Add phantom obstacles (robot thinks space is blocked)
2. Clear real obstacles (robot doesn't see walls)
3. Inflate everything (robot thinks it can't fit anywhere)

SYMPTOMS YOU'LL SEE:
- Robot refuses to plan through clear space
- Robot tries to drive through walls
- All paths seem blocked

HOW TO DEBUG:
1. Visualize costmaps in RViz
2. ros2 topic echo /local_costmap/costmap
3. Check obstacle layer sensor sources
"""
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class CostmapBreaker(Node):
    def __init__(self, cli_mode: str = 'phantom_obstacles'):
        super().__init__('costmap_breaker')
        self.declare_parameter('mode', cli_mode)
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.get_logger().warn('=' * 50)
        self.get_logger().warn('COSTMAP BREAKER ACTIVE - LEARNING MODE')
        self.get_logger().warn(f'Mode: {self.mode}')
        self.get_logger().warn('=' * 50)

        if self.mode == 'phantom_obstacles':
            # Publish fake laser scan with obstacles everywhere
            self.pub = self.create_publisher(LaserScan, '/scan_fake', 10)
            self.timer = self.create_timer(0.1, self.publish_phantom)
            self.get_logger().error(
                'Publishing fake scan with phantom obstacles to /scan_fake'
            )
            self.get_logger().info(
                'To see effect, add /scan_fake to costmap observation_sources'
            )

        elif self.mode == 'clear_obstacles':
            # Subscribe to real scan, publish cleared version
            self.sub = self.create_subscription(
                LaserScan, '/scan', self.clear_callback, 10
            )
            self.pub = self.create_publisher(LaserScan, '/scan_cleared', 10)
            self.get_logger().error(
                'Publishing scan with all obstacles REMOVED to /scan_cleared'
            )
            self.get_logger().info(
                'Robot will not see any obstacles!'
            )

        elif self.mode == 'close_obstacles':
            # Make all obstacles appear very close
            self.sub = self.create_subscription(
                LaserScan, '/scan', self.close_callback, 10
            )
            self.pub = self.create_publisher(LaserScan, '/scan_close', 10)
            self.get_logger().error(
                'Publishing scan with obstacles at 0.5m to /scan_close'
            )
            self.get_logger().info(
                'Robot will think it is surrounded!'
            )

    def publish_phantom(self):
        """Publish fake scan with obstacles at 1 meter all around."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 12.0

        # All ranges at 1.0 meter (phantom wall)
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [1.0] * num_readings
        scan.intensities = [100.0] * num_readings

        self.pub.publish(scan)

    def clear_callback(self, msg: LaserScan):
        """Remove all obstacles from scan."""
        cleared = LaserScan()
        cleared.header = msg.header
        cleared.angle_min = msg.angle_min
        cleared.angle_max = msg.angle_max
        cleared.angle_increment = msg.angle_increment
        cleared.time_increment = msg.time_increment
        cleared.scan_time = msg.scan_time
        cleared.range_min = msg.range_min
        cleared.range_max = msg.range_max

        # Set all ranges to max (no obstacles)
        cleared.ranges = [msg.range_max] * len(msg.ranges)
        cleared.intensities = [0.0] * len(msg.ranges)

        self.pub.publish(cleared)

    def close_callback(self, msg: LaserScan):
        """Make all obstacles appear at 0.5 meters."""
        close = LaserScan()
        close.header = msg.header
        close.angle_min = msg.angle_min
        close.angle_max = msg.angle_max
        close.angle_increment = msg.angle_increment
        close.time_increment = msg.time_increment
        close.scan_time = msg.scan_time
        close.range_min = msg.range_min
        close.range_max = msg.range_max

        # All valid ranges become 0.5m
        close.ranges = [
            0.5 if r < msg.range_max else msg.range_max
            for r in msg.ranges
        ]
        close.intensities = msg.intensities

        self.pub.publish(close)


def main():
    parser = argparse.ArgumentParser(
        description='Costmap Breaker - Learn costmap debugging'
    )
    parser.add_argument(
        '--mode', '-m',
        choices=['phantom_obstacles', 'clear_obstacles', 'close_obstacles'],
        default='phantom_obstacles',
        help='Type of costmap corruption'
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = CostmapBreaker(args.mode)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Costmap Breaker stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
