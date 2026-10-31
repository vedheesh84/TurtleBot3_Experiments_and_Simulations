#!/usr/bin/env python3
"""
Demo 04: SLAM Mapping

BUILDS ON: ROS2_kits_ws/learning_lifecycle

LEARNING OBJECTIVES:
- Understand SLAM pose graph construction
- See loop closure in action
- Learn map saving workflow

WHAT YOU'LL DO:
1. Launch robot with SLAM Toolbox
2. Drive around to build map
3. Close a loop (return to start)
4. Watch pose graph optimization
5. Save the map

COMMANDS TO TRY:
    # Watch SLAM internals
    ros2 topic echo /slam_toolbox/graph_visualization

    # Save map when done
    ros2 run turtlebot_slam save_map.py --map-name my_first_map

    # Or use nav2_map_server
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

LOOP CLOSURE:
When you return to a previously visited area:
1. SLAM detects matching scans
2. Pose graph is optimized
3. Map "snaps" into consistency
4. Watch for the map update!
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
    pkg_slam = get_package_share_directory('turtlebot_slam')
    pkg_bringup = get_package_share_directory('turtlebot_bringup')

    return LaunchDescription([
        # Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'sim_mode': 'true'}.items(),
        ),

        # EKF for odometry
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

        # SLAM Toolbox (mapping mode)
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_slam, 'launch', 'mapping.launch.py')
                    ),
                    launch_arguments={'use_sim_time': 'true'}.items(),
                ),
            ],
        ),

        # RViz with SLAM config
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(pkg_bringup, 'rviz', 'slam.rviz')],
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        ),

        # Teleop
        TimerAction(
            period=6.0,
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
