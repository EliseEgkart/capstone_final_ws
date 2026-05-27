import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
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


def generate_launch_description():
    # =========================================================
    # Launch arguments
    # =========================================================
    button_plan_only = LaunchConfiguration('button_plan_only')
    unload_wait_for_result = LaunchConfiguration('unload_wait_for_result')

    declare_button_plan_only = DeclareLaunchArgument(
        'button_plan_only',
        default_value='false',
        description='Forwarded only to manipulator_task_system.launch.py'
    )

    declare_unload_wait_for_result = DeclareLaunchArgument(
        'unload_wait_for_result',
        default_value='true',
        description='Forwarded only to manipulator_task_system.launch.py'
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
        'manipulator_task_system.launch.py'
    )

    perception_launch_path = get_launch_file(
        'camera_perception_pkg',
        'manipulator_perception.launch.py'
    )

    # =========================================================
    # AMR navigator node
    # =========================================================
    amr_navigator_node = Node(
        package='amr_navigator',
        executable='elevator_delivery_final_with_manipulator',
        name='elevator_delivery_final_with_manipulator',
        output='screen'
    )

    # =========================================================
    # Include launch files with scoped contexts
    # =========================================================
    # NOTE:
    # - Each included launch is wrapped in GroupAction(scoped=True).
    # - The camera perception launch is additionally wrapped with forwarding=False
    #   so unrelated launch arguments from MoveIt/task system do not leak into
    #   realsense2_camera/rs_launch.py.
    moveit_core_group = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_core_launch_path)
            )
        ]
    )

    task_system_group = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(task_system_launch_path),
                launch_arguments={
                    'button_plan_only': button_plan_only,
                    'unload_wait_for_result': unload_wait_for_result,
                }.items()
            )
        ]
    )

    perception_group = GroupAction(
        scoped=True,
        forwarding=False,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch_path)
            )
        ]
    )

    # =========================================================
    # Stable startup order
    # =========================================================
    # Requested timing:
    #   0 sec  : manipulator_moveit / moveit_core.launch.py
    #   3 sec  : manipulator_manager / manipulator_task_system.launch.py
    #   5 sec  : camera_perception_pkg / manipulator_perception.launch.py
    #   18 sec : amr_navigator / elevator_delivery_final_with_manipulator
    return LaunchDescription([
        declare_button_plan_only,
        declare_unload_wait_for_result,

        LogInfo(msg='🚀 Launching manipulator all-in-one system'),

        TimerAction(
            period=0.0,
            actions=[
                LogInfo(msg='[all_in_one] Starting MoveIt core...'),
                moveit_core_group,
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='[all_in_one] Starting manipulator task system...'),
                task_system_group,
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='[all_in_one] Starting camera perception...'),
                perception_group,
            ]
        ),

        TimerAction(
            period=18.0,
            actions=[
                LogInfo(msg='[all_in_one] Starting AMR navigator last...'),
                amr_navigator_node,
            ]
        ),
    ])