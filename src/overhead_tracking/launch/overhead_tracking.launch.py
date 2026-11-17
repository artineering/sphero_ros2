#!/usr/bin/env python3
"""Launch the overhead global-shutter Sphero tracker.

    ros2 launch overhead_tracking overhead_tracking.launch.py
    ros2 launch overhead_tracking overhead_tracking.launch.py source:=sim
    ros2 launch overhead_tracking overhead_tracking.launch.py source:=replay \\
        replay_path:=recordings/kf_20261005_171405/raw.mjpeg [replay_lockstep:=true] \\
        link_method:=nudge
    # a recording without a solo-nudge phase: seed the links from a track.csv
    ros2 launch overhead_tracking overhead_tracking.launch.py source:=replay \\
        replay_path:=recordings/patrol_20261104_163022/raw.mjpeg replay_lockstep:=true \\
        link_seed_csv:=recordings/patrol_20261104_163022/track.csv

Operator workflow once it is up:
    ros2 service call /overhead_tracker_node/detect_arena   std_srvs/srv/Trigger
    ros2 service call /overhead_tracker_node/detect_spheros std_srvs/srv/Trigger
    ros2 service call /overhead_tracker_node/link_spheros \\
        multirobot_msgs/srv/Register "{callsigns: [], skip_compass: true}"
    # after a lighting, gain or exposure change:
    ros2 service call /overhead_tracker_node/rebuild_floor  std_srvs/srv/Trigger

Watch it: /overhead_tracker_node/annotated/compressed (image),
          /overhead_tracker_node/state (JSON status).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('overhead_tracking')
    config = os.path.join(share, 'config', 'overhead_tracking.yaml')

    # Calibration lives OUTSIDE the workspace: `colcon build` wipes install/, so
    # an arena stored in the share directory is silently lost on the next build.
    default_arena = os.path.join(os.path.expanduser('~'), 'overhead_field',
                                 'overhead_arena.yaml')

    args = [
        DeclareLaunchArgument('source', default_value='v4l2',
                              description="'v4l2' (real camera), 'sim' or 'replay'"),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('replay_path', default_value='',
                              description='raw MJPEG file for source:=replay'),
        DeclareLaunchArgument('replay_rate', default_value='1.0'),
        DeclareLaunchArgument('replay_loop', default_value='false'),
        DeclareLaunchArgument('replay_lockstep', default_value='false',
                              description='never drop frames; advance as pulled'),
        DeclareLaunchArgument('link_method', default_value='probe',
                              description="'probe' (LED lit-diff) or 'nudge' "
                                          '(identity by motion; replays need it)'),
        DeclareLaunchArgument('link_seed_csv', default_value='',
                              description='REPLAY ONLY: track.csv whose first rows '
                                          'seed the links (no nudge phase)'),
        DeclareLaunchArgument('track_rate_hz', default_value='60.0',
                              description='processing rate: 60 (every frame) or 30'),
        DeclareLaunchArgument('led_color_space', default_value='bgr',
                              description="LED image colour space: 'hsv' or 'bgr'"),
        DeclareLaunchArgument('publish_raw_jpeg', default_value='false',
                              description='raw camera bytes on ~/camera/compressed'),
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
            'replay_path': LaunchConfiguration('replay_path'),
            'replay_rate': ParameterValue(LaunchConfiguration('replay_rate'),
                                          value_type=float),
            'replay_loop': ParameterValue(LaunchConfiguration('replay_loop'),
                                          value_type=bool),
            'replay_lockstep': ParameterValue(
                LaunchConfiguration('replay_lockstep'), value_type=bool),
            'arena_yaml_path': LaunchConfiguration('arena_yaml_path'),
            'link_method': LaunchConfiguration('link_method'),
            'link_seed_csv': LaunchConfiguration('link_seed_csv'),
            'track_rate_hz': ParameterValue(LaunchConfiguration('track_rate_hz'),
                                            value_type=float),
            'led_color_space': LaunchConfiguration('led_color_space'),
            'publish_raw_jpeg': ParameterValue(
                LaunchConfiguration('publish_raw_jpeg'), value_type=bool),
        }],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
    )
    return LaunchDescription(args + [node])
