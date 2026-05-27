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
    button_config = LaunchConfiguration('button_config')
    task_config = LaunchConfiguration('task_config')

    button_plan_only = LaunchConfiguration('button_plan_only')
    unload_wait_for_result = LaunchConfiguration('unload_wait_for_result')

    declare_arm_config = DeclareLaunchArgument(
        'arm_config',
        default_value=os.path.join(config_dir, 'arm_pose_commander.yaml'),
        description='YAML config file for arm_pose_commander',
    )

    declare_button_config = DeclareLaunchArgument(
        'button_config',
        default_value=os.path.join(config_dir, 'marker_button_press_commander.yaml'),
        description='YAML config file for marker_button_press_commander',
    )

    declare_task_config = DeclareLaunchArgument(
        'task_config',
        default_value=os.path.join(config_dir, 'manipulator_task_manager.yaml'),
        description='YAML config file for manipulator_task_manager',
    )

    declare_button_plan_only = DeclareLaunchArgument(
        'button_plan_only',
        default_value='false',
        description='If true, marker_button_press_commander plans only and does not execute',
    )

    declare_unload_wait_for_result = DeclareLaunchArgument(
        'unload_wait_for_result',
        default_value='true',
        description='If false, task manager assumes unload done after configured delay',
    )

    arm_pose_commander = Node(
        package='manipulator_manager',
        executable='arm_pose_commander',
        name='arm_pose_commander',
        output='screen',
        emulate_tty=True,
        parameters=[arm_config],
    )

    marker_button_press_commander = Node(
        package='manipulator_manager',
        executable='marker_button_press_commander',
        name='marker_button_press_commander',
        output='screen',
        emulate_tty=True,
        parameters=[
            button_config,
            {
                'plan_only': ParameterValue(button_plan_only, value_type=bool),
            },
        ],
    )

    manipulator_task_manager = Node(
        package='manipulator_manager',
        executable='manipulator_task_manager',
        name='manipulator_task_manager',
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
        declare_button_config,
        declare_task_config,
        declare_button_plan_only,
        declare_unload_wait_for_result,

        arm_pose_commander,
        marker_button_press_commander,

        # Start the top-level manager slightly later so lower-level command topics exist first.
        TimerAction(
            period=0.5,
            actions=[manipulator_task_manager],
        ),
    ])