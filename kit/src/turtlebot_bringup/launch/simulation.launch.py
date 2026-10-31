#!/usr/bin/env python3
"""
TurtleBot AMR - Full Simulation Launch
======================================

LEARNING OBJECTIVES (builds on learning_execution, learning_simulation):
- Understand complex launch file orchestration
- See TimerAction for startup sequencing (critical for Gazebo)
- Learn conditional launching with IfCondition
- Practice mode switching (SLAM vs localization)

WHAT THIS LAUNCH DOES:
1. Starts robot_state_publisher with URDF (sim_mode=true)
2. Launches Gazebo simulator
3. Spawns robot entity in Gazebo
4. Starts ros2_control controller manager
5. Activates diff_drive_controller and joint_state_broadcaster
6. Launches robot_localization EKF
7. Starts SLAM or localization (based on argument)
8. Launches Nav2 stack
9. Opens RViz

TIMING (TimerAction):
  0s:  robot_state_publisher
  2s:  Gazebo
  5s:  Spawn entity
  6s:  Controller manager
  8s:  robot_localization EKF
  10s: SLAM Toolbox
  12s: Nav2 servers
  14s: Lifecycle managers
  16s: RViz

USAGE:
  # SLAM mapping mode
  ros2 launch turtlebot_bringup simulation.launch.py

  # Localization mode (with existing map)
  ros2 launch turtlebot_bringup simulation.launch.py slam:=false map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========================================
    # PACKAGE DIRECTORIES
    # ========================================
    pkg_description = get_package_share_directory('turtlebot_description')
    pkg_hardware = get_package_share_directory('turtlebot_hardware')
    pkg_bringup = get_package_share_directory('turtlebot_bringup')
    pkg_localization = get_package_share_directory('turtlebot_localization')
    pkg_slam = get_package_share_directory('turtlebot_slam')
    pkg_navigation = get_package_share_directory('turtlebot_navigation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ========================================
    # FILE PATHS
    # ========================================
    urdf_file = os.path.join(pkg_description, 'urdf', 'turtlebot.urdf.xacro')
    controllers_file = os.path.join(pkg_hardware, 'config', 'ros2_controllers.yaml')
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'simulation.rviz')

    # ========================================
    # LAUNCH CONFIGURATIONS
    # ========================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')

    # Process URDF with xacro (simulation mode)
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' sim_mode:=true']),
        value_type=str
    )

    return LaunchDescription([
        # ========================================
        # LAUNCH ARGUMENTS
        # ========================================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Run SLAM (true) or localization with map (false)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Path to map YAML file (required if slam:=false)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Gazebo world file (empty for default world)'
        ),

        # ========================================
        # ROBOT STATE PUBLISHER (t=0s)
        # ========================================
        # Must start first - provides robot_description to Gazebo
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
        # GAZEBO SIMULATOR (t=2s)
        # ========================================
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
                    ]),
                    launch_arguments={
                        'world': world,
                        'verbose': 'false',
                    }.items(),
                ),
            ]
        ),

        # ========================================
        # SPAWN ROBOT (t=5s)
        # ========================================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'turtlebot',
                        '-topic', 'robot_description',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.1',
                    ],
                    output='screen',
                ),
            ]
        ),

        # ========================================
        # CONTROLLER SPAWNERS (t=7s)
        # ========================================
        # Spawn controllers after robot is in Gazebo
        TimerAction(
            period=7.0,
            actions=[
                # Joint state broadcaster
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
                # Diff drive controller
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ]
        ),

        # ========================================
        # ROBOT LOCALIZATION EKF (t=8s)
        # ========================================
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(pkg_localization, 'launch', 'ekf.launch.py')
                    ]),
                    launch_arguments={
                        'use_sim_time': 'true',
                    }.items(),
                ),
            ]
        ),

        # ========================================
        # SLAM TOOLBOX (t=10s, if slam=true)
        # ========================================
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(pkg_slam, 'launch', 'mapping.launch.py')
                    ]),
                    launch_arguments={
                        'use_sim_time': 'true',
                    }.items(),
                    condition=IfCondition(slam),
                ),
            ]
        ),

        # ========================================
        # MAP-BASED LOCALIZATION (t=10s, if slam=false)
        # ========================================
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(pkg_slam, 'launch', 'localization.launch.py')
                    ]),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'map': map_yaml,
                    }.items(),
                    condition=UnlessCondition(slam),
                ),
            ]
        ),

        # ========================================
        # NAV2 STACK (t=12s)
        # ========================================
        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')
                    ]),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'slam': slam,
                    }.items(),
                ),
            ]
        ),

        # ========================================
        # TELEOP KEYBOARD (t=14s)
        # ========================================
        TimerAction(
            period=14.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                         '--ros-args', '-r', '/cmd_vel:=/cmd_vel'],
                    output='screen',
                    prefix='xterm -e',
                ),
            ]
        ),

        # ========================================
        # RVIZ (t=16s)
        # ========================================
        TimerAction(
            period=16.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                    condition=IfCondition(use_rviz),
                ),
            ]
        ),
    ])
