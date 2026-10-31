#!/usr/bin/env python3
"""
SLAM Toolbox Mapping Launch

LEARNING OBJECTIVES:
- Understand how to launch SLAM in mapping mode
- See parameter file loading patterns
- Learn lifecycle node concepts

WHAT THIS DOES:
1. Launches SLAM Toolbox in online async mapping mode
2. Builds map as robot explores
3. Publishes map->odom transform
4. Saves pose graph for later use
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slam = get_package_share_directory('turtlebot_slam')
    mapping_config = os.path.join(pkg_slam, 'config', 'mapping_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        # SLAM Toolbox Node
        # -----------------
        # LEARNING: This is a lifecycle node. It goes through states:
        # unconfigured -> inactive -> active
        # The lifecycle manager (from Nav2) handles state transitions.
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                mapping_config,
                {'use_sim_time': use_sim_time}
            ],
            # LEARNING: Remappings connect SLAM to your robot's topics
            remappings=[
                ('/scan', '/scan'),
            ],
        ),
    ])
