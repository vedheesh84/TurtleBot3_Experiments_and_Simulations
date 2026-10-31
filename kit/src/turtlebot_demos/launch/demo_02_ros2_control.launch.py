#!/usr/bin/env python3
"""
Demo 02: ros2_control Deep Dive
================================

BUILDS ON: ROS2_kits_ws/learning_lifecycle

LEARNING OBJECTIVES:
- Understand controller_manager architecture
- See hardware interface lifecycle
- Learn controller configuration

WHAT YOU'LL DO:
1. Launch robot with ros2_control
2. Inspect controller states
3. Load/unload controllers manually
4. Understand resource management

COMMANDS TO TRY:
    # List all controllers
    ros2 control list_controllers

    # List hardware interfaces
    ros2 control list_hardware_interfaces

    # Check controller manager state
    ros2 service list | grep controller_manager

    # Manually switch controllers
    ros2 control switch_controllers --stop diff_drive_controller
    ros2 control switch_controllers --start diff_drive_controller

    # Load a new controller
    ros2 control load_controller joint_state_broadcaster

CONTROLLER LIFECYCLE:
    unconfigured -> inactive -> active
    (loaded)       (configured) (running)
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

    controller_config = os.path.join(pkg_hardware, 'config', 'ros2_controllers.yaml')
    urdf_file = os.path.join(pkg_description, 'urdf', 'turtlebot.urdf.xacro')

    serial_port = LaunchConfiguration('serial_port')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' sim_mode:=false', ' serial_port:=', serial_port]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/tmp/tty_turtlebot_fake',
            description='Serial port for Arduino hardware interface'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz'
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

        # Controller Manager (ros2_control_node)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controller_config,
            ],
            output='screen',
        ),

        # Joint State Broadcaster (publishes /joint_states)
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

        # Diff Drive Controller
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

        # RViz
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(pkg_description, 'rviz', 'display.rviz')],
                    parameters=[{'use_sim_time': False}],
                    condition=IfCondition(use_rviz),
                ),
            ],
        ),
    ])
