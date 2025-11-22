#!/usr/bin/env python3
"""
Launch file for Soccer Game Controller.

This launch file starts:
1. Multi-robot webserver for Sphero management
2. Soccer game controller node (which will start ArUco SLAM itself)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():
    """Generate launch description for soccer game."""

    # Multi-robot webserver (using ExecuteProcess as it's not a ROS node)
    multirobot_webserver = ExecuteProcess(
        cmd=['ros2', 'run', 'multirobot_webserver', 'multirobot_webapp'],
        output='log'  # Send to log instead of screen to avoid cluttering terminal
    )

    # Soccer game controller node (delayed to allow webserver to start)
    soccer_game_controller = Node(
        package='soccer_game_controller',
        executable='soccer_game_controller',
        name='soccer_game_controller',
        output='screen'
    )

    # Delay the game controller to allow webserver to start
    delayed_game_controller = TimerAction(
        period=2.0,
        actions=[soccer_game_controller]
    )

    return LaunchDescription([
        multirobot_webserver,
        delayed_game_controller
    ])
