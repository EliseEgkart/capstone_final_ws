import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # =========================================================
    # Package paths
    # =========================================================
    manipulator_moveit_share = get_package_share_directory('manipulator_moveit')
    camera_perception_share = get_package_share_directory('camera_perception_pkg')
    manipulator_manager_share = get_package_share_directory('manipulator_manager')

    # =========================================================
    # Launch paths
    # =========================================================
    moveit_core_launch_path = os.path.join(
        manipulator_moveit_share,
        'launch',
        'moveit_core.launch.py'
    )

    manipulator_perception_launch_path = os.path.join(
        camera_perception_share,
        'launch',
        'manipulator_perception.launch.py'
    )

    # =========================================================
    # Config paths
    # =========================================================
    marker_commander_config = os.path.join(
        manipulator_manager_share,
        'config',
        'marker_moveit_commander.yaml'
    )

    # =========================================================
    # 1. MoveIt core launch
    # =========================================================
    moveit_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_core_launch_path)
    )

    # =========================================================
    # 2. Manipulator perception launch
    # =========================================================
    manipulator_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(manipulator_perception_launch_path)
    )

    # =========================================================
    # 3. Marker MoveIt commander
    # =========================================================
    marker_moveit_commander_node = Node(
        package='manipulator_manager',
        executable='marker_moveit_commander',
        name='marker_moveit_commander',
        output='screen',
        parameters=[
            marker_commander_config
        ]
    )

    # =========================================================
    # Launch order
    # =========================================================
    return LaunchDescription([
        # MoveIt must be started first because marker_moveit_commander
        # sends goals to the MoveGroup action server.
        moveit_core_launch,

        # Perception stack starts after MoveIt initialization begins.
        TimerAction(
            period=3.0,
            actions=[
                manipulator_perception_launch
            ]
        ),

        # Commander starts after MoveIt and perception topics begin to appear.
        TimerAction(
            period=10.0,
            actions=[
                marker_moveit_commander_node
            ]
        ),
    ])