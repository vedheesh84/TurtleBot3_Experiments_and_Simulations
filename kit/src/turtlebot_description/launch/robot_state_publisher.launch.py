#!/usr/bin/env python3
"""
TurtleBot Description - Robot State Publisher Launch
====================================================

Minimal launch file for robot_state_publisher only.
Used by other packages to include robot description.

USAGE:
    ros2 launch turtlebot_description robot_state_publisher.launch.py
    ros2 launch turtlebot_description robot_state_publisher.launch.py sim_mode:=true
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

    # Launch arguments
    sim_mode = LaunchConfiguration('sim_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        # ========================================
        # LAUNCH ARGUMENTS
        # ========================================
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Enable simulation mode (uses Gazebo plugins)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for Arduino hardware interface'
        ),

        # ========================================
        # ROBOT STATE PUBLISHER
        # ========================================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file, ' sim_mode:=', sim_mode, ' serial_port:=', serial_port]),
                    value_type=str
                ),
                'use_sim_time': use_sim_time,
            }]
        ),
    ])
