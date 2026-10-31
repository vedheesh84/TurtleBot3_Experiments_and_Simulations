#!/usr/bin/env python3
"""
Demo 01: Teleop and TF Exploration

BUILDS ON: ROS2_kits_ws/learning_tf

LEARNING OBJECTIVES:
- Understand the robot's TF tree
- See odometry drift in action
- Learn frame relationships

WHAT YOU'LL DO:
1. Launch robot in Gazebo
2. Use teleop to drive
3. Visualize TF in RViz
4. Run view_frames to see TF tree
5. Watch odom frame drift from map

COMMANDS TO TRY:
    # See TF tree structure
    ros2 run tf2_tools view_frames

    # Check specific transform
    ros2 run tf2_ros tf2_echo map base_link

    # Monitor TF timing
    ros2 run tf2_ros tf2_monitor

EXPECTED TF TREE:
    map
     └─ odom (drifts over time!)
         └─ base_link
             ├─ laser_frame
             ├─ imu_link
             ├─ left_wheel_link
             └─ right_wheel_link
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('turtlebot_description')
    pkg_bringup = get_package_share_directory('turtlebot_bringup')

    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'use_teleop',
            default_value='true',
            description='Launch teleop keyboard terminal'
        ),

        # Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'sim_mode': 'true'}.items(),
        ),

        # RViz for visualization
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(pkg_description, 'rviz', 'display.rviz')],
                    parameters=[{'use_sim_time': True}],
                    condition=IfCondition(use_rviz),
                ),
            ],
        ),

        # Teleop keyboard
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='teleop_twist_keyboard',
                    executable='teleop_twist_keyboard',
                    name='teleop',
                    output='screen',
                    prefix='xterm -e',
                    remappings=[('/cmd_vel', '/cmd_vel')],
                    condition=IfCondition(use_teleop),
                ),
            ],
        ),
    ])
