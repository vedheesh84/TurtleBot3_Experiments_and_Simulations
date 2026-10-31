#!/usr/bin/env python3
"""
Demo 05: Nav2 Navigation Basics

BUILDS ON: ROS2_kits_ws/learning_execution

LEARNING OBJECTIVES:
- Understand Nav2 architecture
- See costmaps in action
- Learn behavior tree navigation

PREREQUISITES:
- A saved map (from Demo 04)

WHAT YOU'LL DO:
1. Launch Nav2 with saved map
2. Set initial pose in RViz
3. Send navigation goals
4. Watch planner and controller work
5. Trigger recovery behaviors

COMMANDS TO TRY:
    # Send a goal programmatically
    ros2 run turtlebot_demos goal_sender --x 2.0 --y 1.0

    # Check Nav2 lifecycle states
    ros2 lifecycle list /planner_server

    # Clear costmaps
    ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty

    # Check path
    ros2 topic echo /plan

NAV2 ARCHITECTURE:
    bt_navigator
        ├─ planner_server (global path)
        ├─ controller_server (local execution)
        └─ behavior_server (recoveries)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('turtlebot_description')
    pkg_localization = get_package_share_directory('turtlebot_localization')
    pkg_slam = get_package_share_directory('turtlebot_slam')
    pkg_nav = get_package_share_directory('turtlebot_navigation')

    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Path to map yaml file'
        ),

        # Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'sim_mode': 'true'}.items(),
        ),

        # EKF
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_localization, 'launch', 'ekf.launch.py')
                    ),
                    launch_arguments={'use_sim_time': 'true'}.items(),
                ),
            ],
        ),

        # SLAM in localization mode (or use AMCL)
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_slam, 'launch', 'localization.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'map_file': map_file,
                    }.items(),
                ),
            ],
        ),

        # Nav2 Stack
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'map': map_file,
                    }.items(),
                ),
            ],
        ),

        # RViz with navigation config
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(pkg_nav, 'rviz', 'navigation.rviz')],
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        ),
    ])
