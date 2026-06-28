from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Bring up the multi-robot web server.

    The foxglove_bridge is launched separately via foxglove_bridge.launch.py,
    so it is not launched here. Localization is provided externally by the
    overhead Kinect (kinect_field_tracking), also launched separately. The
    webapp uses a blocking input() prompt at startup, so it runs with
    output='screen' on the attached TTY.
    """
    return LaunchDescription([
        Node(
            package='multirobot_webserver',
            executable='multirobot_webapp',
            name='multirobot_webapp',
            output='screen',
            emulate_tty=True,
        ),
    ])
