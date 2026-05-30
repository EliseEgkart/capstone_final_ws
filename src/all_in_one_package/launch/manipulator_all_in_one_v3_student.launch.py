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


def get_launch_file(package_name: str, launch_file_name: str) -> str:
    return os.path.join(
        get_package_share_directory(package_name),
        'launch',
        launch_file_name,
    )


def get_config(package_name: str, filename: str) -> str:
    return os.path.join(
        get_package_share_directory(package_name),
        'config',
        filename,
    )


def generate_launch_description():
    prepress_plan_only = LaunchConfiguration('prepress_plan_only')
    arm_config = LaunchConfiguration('arm_config')
    prepress_config = LaunchConfiguration('prepress_config')
    task_config = LaunchConfiguration('task_config')

    declare_prepress_plan_only = DeclareLaunchArgument(
        'prepress_plan_only',
        default_value='false',
        description='If true, prepress commander plans only and does not execute',
    )

    declare_arm_config = DeclareLaunchArgument(
        'arm_config',
        default_value=get_config('manipulator_manager', 'arm_pose_commander_v2.yaml'),
        description='Forwarded to manipulator_task_system_v3_student.launch.py',
    )

    declare_prepress_config = DeclareLaunchArgument(
        'prepress_config',
        default_value=get_config(
            'manipulator_manager',
            'marker_prepress_commander_v2.yaml',
        ),
        description='Forwarded to manipulator_task_system_v3_student.launch.py',
    )

    declare_task_config = DeclareLaunchArgument(
        'task_config',
        default_value=get_config(
            'manipulator_manager',
            'manipulator_task_manager_v3_student.yaml',
        ),
        description='Forwarded to manipulator_task_system_v3_student.launch.py',
    )

    moveit_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_launch_file('manipulator_moveit', 'moveit_core.launch.py')
        )
    )

    task_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_launch_file(
                'manipulator_manager',
                'manipulator_task_system_v3_student.launch.py',
            )
        ),
        launch_arguments={
            'prepress_plan_only': prepress_plan_only,
            'arm_config': arm_config,
            'prepress_config': prepress_config,
            'task_config': task_config,
        }.items(),
    )

    camera_perception_process = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'camera_perception_pkg',
            'manipulator_perception.launch.py',
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_prepress_plan_only,
        declare_arm_config,
        declare_prepress_config,
        declare_task_config,

        LogInfo(msg='[all_in_one_v3_student] Launching student demo system'),

        TimerAction(
            period=0.0,
            actions=[
                LogInfo(msg='[all_in_one_v3_student] Starting MoveIt core...'),
                moveit_core_launch,
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(
                    msg='[all_in_one_v3_student] Starting manipulator task system...'
                ),
                task_system_launch,
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(
                    msg='[all_in_one_v3_student] Starting camera perception...'
                ),
                camera_perception_process,
            ],
        ),
    ])
