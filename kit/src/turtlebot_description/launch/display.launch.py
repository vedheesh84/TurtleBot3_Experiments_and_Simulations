#!/usr/bin/env python3
"""
TurtleBot Description - Display Launch File
============================================

LEARNING OBJECTIVES (builds on learning_execution):
- Understand basic launch file structure
- See how robot_state_publisher works
- Learn joint_state_publisher_gui usage
- Practice RViz visualization

WHAT THIS LAUNCH DOES:
1. Loads URDF via xacro
2. Starts robot_state_publisher (publishes TF from URDF)
3. Starts joint_state_publisher_gui (allows manual joint control)
4. Launches RViz with preconfigured display

USAGE:
    ros2 launch turtlebot_description display.launch.py

TRY THIS:
- Move the sliders in joint_state_publisher_gui
- Watch the wheels rotate in RViz
- Run: ros2 run tf2_tools view_frames
- See the TF tree generated from URDF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directory
    pkg_description = get_package_share_directory('turtlebot_description')

    # Paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'turtlebot.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'display.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')

    # Process URDF with xacro (sim_mode=false for display)
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' sim_mode:=false']),
        value_type=str
    )

    return LaunchDescription([
        # ========================================
        # LAUNCH ARGUMENTS
        # ========================================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (for Gazebo)'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Launch joint_state_publisher_gui'
        ),

        # ========================================
        # ROBOT STATE PUBLISHER
        # ========================================
        # Publishes TF tree from URDF
        # Input: robot_description parameter
        # Output: /tf, /tf_static topics
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }]
        ),

        # ========================================
        # JOINT STATE PUBLISHER GUI
        # ========================================
        # Provides GUI to manually set joint positions
        # Input: robot_description parameter
        # Output: /joint_states topic
        #
        # In production, this is replaced by:
        # - joint_state_broadcaster (ros2_control)
        # - Hardware driver publishing joint states
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            condition=LaunchConfiguration('use_gui')
        ),

        # ========================================
        # RVIZ2
        # ========================================
        # Visualization tool
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),
    ])
