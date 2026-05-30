#!/usr/bin/env python3
"""Dev/test launch for the matrix-marker positioning source.

Production is webserver-managed; this brings up matrix_slam_node standalone
with the config yaml (dev robot_table fallback).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_slam')
    config = os.path.join(pkg_share, 'config', 'matrix_positioning.yaml')

    return LaunchDescription([
        Node(
            package='aruco_slam',
            executable='matrix_slam_node',
            name='matrix_slam_node',
            output='screen',
            parameters=[config],
        ),
    ])
