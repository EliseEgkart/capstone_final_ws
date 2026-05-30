#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def get_launch_file(package_name: str, launch_file_name: str) -> str:
    """Return an installed launch file path from a ROS 2 package share directory."""
    return os.path.join(
        get_package_share_directory(package_name),
        'launch',
        launch_file_name,
    )


def get_first_existing_config(package_share: str, candidates: list[str]) -> str:
    """Return the first existing config path from package_share/config."""
    for filename in candidates:
        path = os.path.join(package_share, 'config', filename)
        if os.path.exists(path):
            return path

    return os.path.join(package_share, 'config', candidates[0])


def generate_launch_description():
    # =========================================================
    # Package share paths
    # =========================================================
    manipulator_manager_share = get_package_share_directory('manipulator_manager')

    # =========================================================
    # Default config paths for manipulator task system
    # =========================================================
    default_arm_config = get_first_existing_config(
        manipulator_manager_share,
        [
            'arm_pose_commander_v2.yaml',
        ]
    )

    default_prepress_config = get_first_existing_config(
        manipulator_manager_share,
        [
            'marker_prepress_commander_v2.yaml',
        ]
    )

    default_task_config = get_first_existing_config(
        manipulator_manager_share,
        [
            'manipulator_task_manager_v3_student.yaml',
        ]
    )

    # =========================================================
    # Launch arguments
    # =========================================================
    prepress_plan_only = LaunchConfiguration('prepress_plan_only')
    unload_wait_for_result = LaunchConfiguration('unload_wait_for_result')
    arm_config = LaunchConfiguration('arm_config')
    prepress_config = LaunchConfiguration('prepress_config')
    task_config = LaunchConfiguration('task_config')

    declare_prepress_plan_only = DeclareLaunchArgument(
        'prepress_plan_only',
        default_value='false',
        description='Forwarded only to manipulator_task_system_v3_student.launch.py'
    )

    declare_unload_wait_for_result = DeclareLaunchArgument(
        'unload_wait_for_result',
        default_value='true',
        description='Accepted for v2 launch compatibility; v3_student unload is fire-and-forget'
    )

    declare_arm_config = DeclareLaunchArgument(
        'arm_config',
        default_value=default_arm_config,
        description='Forwarded only to manipulator_task_system_v3_student.launch.py'
    )

    declare_prepress_config = DeclareLaunchArgument(
        'prepress_config',
        default_value=default_prepress_config,
        description='Forwarded only to manipulator_task_system_v3_student.launch.py'
    )

    declare_task_config = DeclareLaunchArgument(
        'task_config',
        default_value=default_task_config,
        description='Forwarded only to manipulator_task_system_v3_student.launch.py'
    )

    # =========================================================
    # Launch file paths
    # =========================================================
    moveit_core_launch_path = get_launch_file(
        'manipulator_moveit',
        'moveit_core.launch.py'
    )

    task_system_launch_path = get_launch_file(
        'manipulator_manager',
        'manipulator_task_system_v3_student.launch.py'
    )

    # =========================================================
    # Include launch files
    # =========================================================
    moveit_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_core_launch_path)
    )

    task_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(task_system_launch_path),
        launch_arguments={
            'prepress_plan_only': prepress_plan_only,
            'unload_wait_for_result': unload_wait_for_result,
            'arm_config': arm_config,
            'prepress_config': prepress_config,
            'task_config': task_config,
        }.items()
    )

    # =========================================================
    # Camera perception launch as an isolated process
    # =========================================================
    # NOTE:
    # - This intentionally uses ExecuteProcess instead of IncludeLaunchDescription.
    # - It prevents all parent launch arguments such as prepress_plan_only,
    #   unload_wait_for_result, arm_config, prepress_config, and task_config
    #   from leaking into camera_perception_pkg/manipulator_perception.launch.py
    #   and then into realsense2_camera/rs_launch.py.
    camera_perception_process = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'camera_perception_pkg',
            'manipulator_perception.launch.py',
        ],
        output='screen',
    )

    # =========================================================
    # AMR navigator node
    # =========================================================
    amr_navigator_node = Node(
        package='amr_navigator',
        executable='indoor_students_manager',
        name='indoor_students_manager',
        output='screen',
        parameters=[{
            'manipulator_task_cmd_topic': '/manipulator_task_cmd',
            'manipulator_task_result_topic': '/manipulator_task_cmd',
            'manipulator_task_state_topic': '/manipulator_task_state_v3_student',
        }]
    )

    # =========================================================
    # Stable startup order
    # =========================================================
    # Same all-in-one structure as manipulator_all_in_one_v2.launch.py:
    #   0 sec  : manipulator_moveit / moveit_core.launch.py
    #   3 sec  : manipulator_manager / manipulator_task_system_v3_student.launch.py
    #   5 sec  : camera_perception_pkg / manipulator_perception.launch.py
    #   55 sec : amr_navigator / indoor_students_manager
    return LaunchDescription([
        declare_prepress_plan_only,
        declare_unload_wait_for_result,
        declare_arm_config,
        declare_prepress_config,
        declare_task_config,

        LogInfo(msg='[all_in_one] Launching manipulator all-in-one system'),

        TimerAction(
            period=0.0,
            actions=[
                LogInfo(msg='[all_in_one] Starting MoveIt core...'),
                moveit_core_launch,
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='[all_in_one] Starting manipulator task system...'),
                task_system_launch,
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='[all_in_one] Starting camera perception as isolated process...'),
                camera_perception_process,
            ]
        ),

        TimerAction(
            period=55.0,
            actions=[
                LogInfo(msg='[all_in_one] Starting AMR navigator last...'),
                amr_navigator_node,
            ]
        ),
    ])
