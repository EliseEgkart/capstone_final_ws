#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('manipulator_manager')
    config_dir = os.path.join(pkg_share, 'config')

    arm_config = LaunchConfiguration('arm_config')
    prepress_config = LaunchConfiguration('prepress_config')
    task_config = LaunchConfiguration('task_config')
    prepress_plan_only = LaunchConfiguration('prepress_plan_only')
    unload_wait_for_result = LaunchConfiguration('unload_wait_for_result')

    declare_arm_config = DeclareLaunchArgument(
        'arm_config',
        default_value=os.path.join(config_dir, 'arm_pose_commander_v2.yaml'),
        description='YAML config file for arm_pose_commander_v2',
    )

    declare_prepress_config = DeclareLaunchArgument(
        'prepress_config',
        default_value=os.path.join(config_dir, 'marker_prepress_commander_v2.yaml'),
        description='YAML config file for marker_prepress_commander_v2',
    )

    declare_task_config = DeclareLaunchArgument(
        'task_config',
        default_value=os.path.join(config_dir, 'manipulator_task_manager_v3_student.yaml'),
        description='YAML config file for manipulator_task_manager_v3_student',
    )

    declare_prepress_plan_only = DeclareLaunchArgument(
        'prepress_plan_only',
        default_value='false',
        description='If true, prepress commander plans only and does not execute',
    )

    declare_unload_wait_for_result = DeclareLaunchArgument(
        'unload_wait_for_result',
        default_value='true',
        description='Accepted for v2 launch compatibility; v3_student unload is fire-and-forget',
    )

    arm_pose_commander_v2 = Node(
        package='manipulator_manager',
        executable='arm_pose_commander_v2',
        name='arm_pose_commander_v2',
        output='screen',
        emulate_tty=True,
        parameters=[arm_config],
    )

    marker_prepress_commander_v2 = Node(
        package='manipulator_manager',
        executable='marker_prepress_commander_v2',
        name='marker_prepress_commander_v2',
        output='screen',
        emulate_tty=True,
        parameters=[
            prepress_config,
            {
                'plan_only': ParameterValue(prepress_plan_only, value_type=bool),
            },
        ],
    )

    manipulator_task_manager_v3_student = Node(
        package='manipulator_manager',
        executable='manipulator_task_manager_v3_student',
        name='manipulator_task_manager_v3_student',
        output='screen',
        emulate_tty=True,
        parameters=[
            task_config,
            {
                'unload_wait_for_result': ParameterValue(unload_wait_for_result, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        declare_arm_config,
        declare_prepress_config,
        declare_task_config,
        declare_prepress_plan_only,
        declare_unload_wait_for_result,
        arm_pose_commander_v2,
        marker_prepress_commander_v2,
        TimerAction(
            period=0.5,
            actions=[manipulator_task_manager_v3_student],
        ),
    ])
