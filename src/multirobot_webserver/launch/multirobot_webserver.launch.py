from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Bring up the multi-robot web server.

    The web server self-starts the foxglove_bridge in its main() and starts
    the UWB positioning (BLE) node on demand via its REST API, so neither is
    launched here. The webapp uses a blocking input() prompt at startup, so it
    runs with output='screen' on the attached TTY.
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
