#!/usr/bin/env python3
"""Launch the overhead global-shutter Sphero tracker.

    ros2 launch overhead_tracking overhead_tracking.launch.py
    ros2 launch overhead_tracking overhead_tracking.launch.py source:=sim

Operator workflow once it is up:
    ros2 service call /overhead_tracker_node/detect_arena   std_srvs/srv/Trigger
    ros2 service call /overhead_tracker_node/detect_spheros std_srvs/srv/Trigger
    ros2 service call /overhead_tracker_node/link_spheros \\
        multirobot_msgs/srv/Register "{callsigns: [], skip_compass: true}"

Watch it: /overhead_tracker_node/annotated/compressed (image),
          /overhead_tracker_node/state (JSON status).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('overhead_tracking')
    config = os.path.join(share, 'config', 'overhead_tracking.yaml')

    # Calibration lives OUTSIDE the workspace: `colcon build` wipes install/, so
    # an arena stored in the share directory is silently lost on the next build.
    default_arena = os.path.join(os.path.expanduser('~'), 'overhead_field',
                                 'overhead_arena.yaml')

    args = [
        DeclareLaunchArgument('source', default_value='v4l2',
                              description="'v4l2' (real camera) or 'sim'"),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('arena_yaml_path', default_value=default_arena,
                              description='arena calibration, kept outside install/'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    node = Node(
        package='overhead_tracking',
        executable='overhead_tracker_node',
        name='overhead_tracker_node',
        output='screen',
        parameters=[config, {
            'source': LaunchConfiguration('source'),
            'camera_device': LaunchConfiguration('camera_device'),
            'arena_yaml_path': LaunchConfiguration('arena_yaml_path'),
        }],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
    )
    return LaunchDescription(args + [node])
