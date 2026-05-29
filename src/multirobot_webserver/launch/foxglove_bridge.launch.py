from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('address', default_value='0.0.0.0'),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
                'tls': False,
                'send_buffer_limit': 10000000,
                'capabilities': [
                    'clientPublish',
                    'parameters',
                    'parametersSubscribe',
                    'services',
                    'connectionGraph',
                    'assets',
                ],
                'topic_whitelist': [
                    '/sphero/.*',
                    '/sphero_fleet/.*',
                    '/aruco_slam/.*',
                    '/uwb/.*',
                    '/rosout',
                    '/parameter_events',
                    '/tf',
                    '/tf_static',
                ],
            }],
        ),
    ])
