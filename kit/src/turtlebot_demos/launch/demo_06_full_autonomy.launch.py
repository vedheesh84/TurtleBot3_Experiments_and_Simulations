#!/usr/bin/env python3
"""
Demo 06: Full Autonomous System

BUILDS ON: ROS2_kits_ws/learning_integration

LEARNING OBJECTIVES:
- See all components working together
- Understand system-level debugging
- Practice failure recovery

PREREQUISITES:
- A saved map (from Demo 04)
- Understanding of Demos 01-05

FULL SYSTEM ARCHITECTURE:
    ┌─────────────────────────────────────────┐
    │              Behavior Tree               │
    │         (bt_navigator)                   │
    └──────────────┬──────────────────────────┘
                   │
    ┌──────────────┼──────────────────────────┐
    │              │                           │
    ▼              ▼                           ▼
┌─────────┐  ┌───────────┐  ┌─────────────────┐
│ Planner │  │Controller │  │ Behavior Server │
│ Server  │  │  Server   │  │  (recoveries)   │
└────┬────┘  └─────┬─────┘  └────────────────┘
     │             │
     │        ┌────┴────┐
     │        │         │
     ▼        ▼         ▼
┌─────────────────────────────────────────────┐
│               Costmaps                       │
│    (local + global, multiple layers)         │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│               SLAM / AMCL                    │
│         (map -> odom transform)              │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│                   EKF                        │
│         (odom -> base_link transform)        │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│            ros2_control                      │
│    (diff_drive_controller -> hardware)       │
└─────────────────────────────────────────────┘

FAILURE INJECTION:
    # Break TF
    ros2 run turtlebot_demos break_tf --mode duplicate

    # Break odometry
    ros2 run turtlebot_demos break_odom --mode drift

    # Break costmap
    ros2 run turtlebot_demos break_costmap --mode phantom_obstacles
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory('turtlebot_bringup')

    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Path to map yaml file'
        ),

        # Full simulation launch (includes everything)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'map': map_file,
            }.items(),
        ),
    ])
