#!/usr/bin/env python3
"""
Demo 03: Sensor Fusion with EKF

BUILDS ON: ROS2_kits_ws/learning_comms

LEARNING OBJECTIVES:
- Understand Extended Kalman Filter basics
- See sensor fusion in action (odom + IMU)
- Learn covariance tuning

WHAT YOU'LL DO:
1. Launch robot with EKF
2. Compare raw odom vs filtered odom
3. Watch drift correction
4. Tune covariances

COMMANDS TO TRY:
    # Compare raw vs filtered odometry
    ros2 topic echo /diff_drive_controller/odom
    ros2 topic echo /odometry/filtered

    # Visualize drift
    ros2 run turtlebot_demos odom_drift_visualizer

    # Check EKF covariance
    ros2 topic echo /odometry/filtered --field pose.covariance

EXPERIMENT:
1. Drive robot in a square, return to start
2. Raw odom will show drift
3. Filtered odom (with IMU) will be more accurate
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('turtlebot_description')
    pkg_localization = get_package_share_directory('turtlebot_localization')

    return LaunchDescription([
        # Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'sim_mode': 'true'}.items(),
        ),

        # EKF Node
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

        # Drift Visualizer
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='turtlebot_demos',
                    executable='odom_drift_visualizer',
                    name='drift_viz',
                    output='screen',
                ),
            ],
        ),

        # RViz
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(pkg_description, 'rviz', 'display.rviz')],
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        ),

        # Teleop
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='teleop_twist_keyboard',
                    executable='teleop_twist_keyboard',
                    name='teleop',
                    output='screen',
                    prefix='xterm -e',
                ),
            ],
        ),
    ])
