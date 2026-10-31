#!/usr/bin/env python3
"""
Map Saving Utility

LEARNING OBJECTIVES:
- Understand the map saving workflow
- See how to call SLAM Toolbox services
- Learn about occupancy grid maps vs pose graphs

USAGE:
    ros2 run turtlebot_slam save_map.py --map-name my_map

WHAT THIS SAVES:
1. Occupancy grid (.pgm + .yaml) - for Nav2
2. Pose graph (.posegraph) - for SLAM Toolbox localization
"""
import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph


class MapSaver(Node):
    def __init__(self, map_name: str, output_dir: str):
        super().__init__('map_saver')
        self.map_name = map_name
        self.output_dir = output_dir

        # Service clients
        self.save_map_client = self.create_client(
            SaveMap, '/slam_toolbox/save_map'
        )
        self.serialize_client = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map'
        )

        self.get_logger().info(f'Map Saver initialized')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(f'Map name: {self.map_name}')

    def save_occupancy_grid(self):
        """Save the occupancy grid map (for Nav2)."""
        if not self.save_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('save_map service not available')
            return False

        request = SaveMap.Request()
        request.name.data = os.path.join(self.output_dir, self.map_name)

        self.get_logger().info('Saving occupancy grid map...')
        future = self.save_map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(
                f'Occupancy grid saved: {self.map_name}.pgm, {self.map_name}.yaml'
            )
            return True
        else:
            self.get_logger().error('Failed to save occupancy grid')
            return False

    def save_pose_graph(self):
        """Save the pose graph (for SLAM Toolbox localization)."""
        if not self.serialize_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('serialize_map service not available')
            return False

        request = SerializePoseGraph.Request()
        request.filename = os.path.join(self.output_dir, self.map_name)

        self.get_logger().info('Saving pose graph...')
        future = self.serialize_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(
                f'Pose graph saved: {self.map_name}.posegraph, {self.map_name}.data'
            )
            return True
        else:
            self.get_logger().error('Failed to save pose graph')
            return False


def main():
    parser = argparse.ArgumentParser(description='Save SLAM Toolbox map')
    parser.add_argument(
        '--map-name', '-n',
        type=str,
        default='map',
        help='Name for the saved map files'
    )
    parser.add_argument(
        '--output-dir', '-o',
        type=str,
        default='.',
        help='Directory to save maps'
    )
    args = parser.parse_args()

    rclpy.init()
    node = MapSaver(args.map_name, args.output_dir)

    try:
        # Save both formats
        grid_ok = node.save_occupancy_grid()
        graph_ok = node.save_pose_graph()

        if grid_ok and graph_ok:
            node.get_logger().info('Map saving complete!')
            node.get_logger().info('')
            node.get_logger().info('Files saved:')
            node.get_logger().info(f'  - {args.map_name}.pgm      (occupancy grid image)')
            node.get_logger().info(f'  - {args.map_name}.yaml     (occupancy grid metadata)')
            node.get_logger().info(f'  - {args.map_name}.posegraph (SLAM pose graph)')
            node.get_logger().info(f'  - {args.map_name}.data     (SLAM scan data)')
        else:
            node.get_logger().error('Some maps failed to save')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
