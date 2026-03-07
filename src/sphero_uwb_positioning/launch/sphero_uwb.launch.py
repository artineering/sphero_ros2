#!/usr/bin/env python3
"""
sphero_uwb.launch.py
====================
Launch file for Sphero UWB Positioning system.

Usage:
    ros2 launch sphero_uwb_positioning sphero_uwb.launch.py
    ros2 launch sphero_uwb_positioning sphero_uwb.launch.py config:=/path/to/custom_config.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Default config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('sphero_uwb_positioning'),
        'config',
        'uwb_config.yaml'
    ])

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'config',
            default_value=config_file,
            description='Path to UWB configuration file'
        ),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error)'
        ),

        # Sphero UWB Positioning Node
        Node(
            package='sphero_uwb_positioning',
            executable='sphero_uwb_positioning_node',
            name='sphero_uwb_positioning_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
