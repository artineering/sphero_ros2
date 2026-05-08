from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('sphero_uwb_positioning'),
        'config', 'ble_uwb_config.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config_file),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='sphero_uwb_positioning',
            executable='ble_position_node',
            name='uwb_ble_position_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
