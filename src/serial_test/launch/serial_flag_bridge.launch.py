from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('serial_test')
    default_params = os.path.join(pkg_share, 'params', 'serial_flags.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to serial bridge parameter YAML',
        ),
        Node(
            package='serial_test',
            executable='serial_flag_bridge',
            name='serial_flag_bridge',
            output='screen',
            parameters=[params_file],
        ),
    ])
