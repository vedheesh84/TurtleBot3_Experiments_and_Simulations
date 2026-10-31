#!/usr/bin/env python3
"""
TurtleBot AMR - Hardware Bringup Launch
=======================================

Launches the physical (or pseudo-hardware emulated) robot system:
1. Robot State Publisher with URDF (sim_mode=false, serial_port configured)
2. Controller Manager (ros2_control_node)
3. Spawns joint_state_broadcaster and diff_drive_controller
4. Optional RViz visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('turtlebot_description')
    pkg_hardware = get_package_share_directory('turtlebot_hardware')
    pkg_bringup = get_package_share_directory('turtlebot_bringup')

    urdf_file = os.path.join(pkg_description, 'urdf', 'turtlebot.urdf.xacro')
    controllers_file = os.path.join(pkg_hardware, 'config', 'ros2_controllers.yaml')
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'simulation.rviz')

    serial_port = LaunchConfiguration('serial_port')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' sim_mode:=false', ' serial_port:=', serial_port]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for Arduino / microcontroller hardware interface'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz for visualization'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }]
        ),

        # ros2_control controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_file,
            ],
            output='screen',
        ),

        # Joint State Broadcaster Spawner
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ],
        ),

        # Diff Drive Controller Spawner
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ],
        ),

        # RViz (Conditional)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    condition=IfCondition(use_rviz),
                ),
            ],
        ),
    ])
