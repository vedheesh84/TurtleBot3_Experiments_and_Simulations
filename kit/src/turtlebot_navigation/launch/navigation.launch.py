#!/usr/bin/env python3
"""
Nav2 Navigation Launch

LEARNING OBJECTIVES:
- Understand Nav2 node composition
- See lifecycle management in action
- Learn parameter loading patterns

WHAT THIS LAUNCHES:
1. map_server        - Serves the static map
2. planner_server    - Global path planning
3. controller_server - Local trajectory following
4. behavior_server   - Recovery behaviors
5. bt_navigator      - Behavior tree orchestration
6. lifecycle_manager - Coordinates all nodes
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_nav = get_package_share_directory('turtlebot_navigation')
    nav2_config = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    default_bt = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml = LaunchConfiguration('default_bt_xml_filename')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Path to map yaml file'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_config,
            description='Path to Nav2 params file'
        ),
        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=default_bt,
            description='Behavior tree XML file'
        ),

        # Set use_sim_time for all nodes
        SetParameter(name='use_sim_time', value=use_sim_time),

        # Map Server
        # ----------
        # LEARNING: Serves the occupancy grid map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {'yaml_filename': map_file}
            ],
        ),

        # Planner Server (Global Planner)
        # --------------------------------
        # LEARNING: Computes paths from start to goal
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),

        # Controller Server (Local Planner)
        # ----------------------------------
        # LEARNING: Follows the path, avoids obstacles
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),

        # Behavior Server (Recoveries)
        # ----------------------------
        # LEARNING: spin, backup, wait behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),

        # BT Navigator
        # ------------
        # LEARNING: Orchestrates navigation using behavior trees
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file,
                {'default_bt_xml_filename': default_bt_xml}
            ],
        ),

        # Lifecycle Manager
        # -----------------
        # LEARNING: Coordinates node lifecycle transitions
        # unconfigured -> inactive -> active
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'autostart': autostart},
                {'node_names': [
                    'map_server',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ]}
            ],
        ),
    ])
