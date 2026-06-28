#!/usr/bin/env python3
"""Launch the Kinect v1 field tracker (sim by default).

  ros2 launch kinect_field_tracking kinect_field_tracking.launch.py
  # real camera:
  ros2 launch kinect_field_tracking kinect_field_tracking.launch.py source:=kinect

Services once up:
  ros2 service call /field_tracker_node/calibrate std_srvs/srv/Trigger
  ros2 service call /field_tracker_node/capture_baseline std_srvs/srv/Trigger
  ros2 service call /field_tracker_node/register multirobot_msgs/srv/Register \
      "{callsigns: [], skip_compass: false}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('kinect_field_tracking')
    config = os.path.join(pkg_share, 'config', 'kinect_field_tracking.yaml')

    # Persist calibration OUTSIDE the workspace: install/ is wiped by colcon build,
    # so a heightmap/field YAML written under the package share would be lost on the
    # next rebuild. Default to a stable per-user dir; override with the launch args.
    default_dir = os.path.join(os.path.expanduser('~'), 'kinect_field')

    source = LaunchConfiguration('source')
    heightmap_path = LaunchConfiguration('heightmap_path')
    field_yaml_path = LaunchConfiguration('field_yaml_path')

    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='sim',
                              description="depth source: 'kinect' or 'sim'"),
        DeclareLaunchArgument(
            'heightmap_path',
            default_value=os.path.join(default_dir, 'kinect_heightmap.npy'),
            description='empty-arena depth heightmap (.npy); persists across rebuilds'),
        DeclareLaunchArgument(
            'field_yaml_path',
            default_value=os.path.join(default_dir, 'kinect_field.yaml'),
            description='field calibration YAML; persists across rebuilds'),
        Node(
            package='kinect_field_tracking',
            executable='field_tracker_node',
            name='field_tracker_node',
            output='screen',
            parameters=[config, {
                'source': source,
                'heightmap_path': heightmap_path,
                'field_yaml_path': field_yaml_path,
            }],
        ),
    ])
