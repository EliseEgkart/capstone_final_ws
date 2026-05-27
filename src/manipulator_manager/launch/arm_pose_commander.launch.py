import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('manipulator_manager')

    config_path = os.path.join(
        pkg_share,
        'config',
        'arm_pose_commander.yaml'
    )

    arm_pose_commander_node = Node(
        package='manipulator_manager',
        executable='arm_pose_commander',
        name='arm_pose_commander',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([
        arm_pose_commander_node
    ])