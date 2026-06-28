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
                # NOTE: 'parameters'/'parametersSubscribe' are intentionally
                # omitted. With them on, the bridge probes every node's parameter
                # service on discovery; a freshly-deployed sphero controller's
                # param service isn't ready in time, so the bridge errors and
                # ignore-lists the node. We use Foxglove for visualization, not
                # param editing, so we drop param support to kill that noise.
                'capabilities': [
                    'clientPublish',
                    'services',
                    'connectionGraph',
                    'assets',
                ],
                'topic_whitelist': [
                    '/sphero/.*',
                    '/sphero_fleet/.*',
                    '/field_tracker_node/.*',
                    '/localization/.*',
                    '/rosout',
                    '/parameter_events',
                    '/tf',
                    '/tf_static',
                ],
            }],
        ),
    ])
