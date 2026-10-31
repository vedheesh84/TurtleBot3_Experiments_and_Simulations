#!/usr/bin/env python3
"""
SLAM Toolbox Localization Launch

LEARNING OBJECTIVES:
- Understand localization vs mapping mode
- See how to load a saved map
- Learn pose tracking with existing map

WHAT THIS DOES:
1. Loads a previously saved map (pose graph)
2. Localizes robot within the map
3. Publishes map->odom transform
4. Does NOT modify the map
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slam = get_package_share_directory('turtlebot_slam')
    localization_config = os.path.join(pkg_slam, 'config', 'localization_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Path to the saved map file (.posegraph)'
        ),

        # SLAM Toolbox in Localization Mode
        # ----------------------------------
        # LEARNING: Same node, different config file
        # The 'mode: localization' parameter changes behavior
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                localization_config,
                {
                    'use_sim_time': use_sim_time,
                    'map_file_name': map_file,
                }
            ],
            remappings=[
                ('/scan', '/scan'),
            ],
        ),
    ])
