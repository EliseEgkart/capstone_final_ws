import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # MoveIt launch directory
    launch_dir = os.path.join(
        get_package_share_directory('manipulator_moveit'),
        'launch'
    )

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/moveit_rviz.launch.py'])
    )
    ld.add_action(rviz_launch)

    # move_group launch include
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'move_group.launch.py')
        )
    )

    ld.add_action(move_group_launch)

    return ld