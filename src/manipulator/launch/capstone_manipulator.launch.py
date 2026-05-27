# =========================================================
# Purpose
# =========================================================
# Load the manipulator URDF/Xacro model, publish robot states,
# and visualize the robot in RViz with optional joint GUI control.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # =========================================================
    # Launch arguments
    # =========================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    gui = LaunchConfiguration('gui')

    # =========================================================
    # Default paths
    # =========================================================
    pkg_share = get_package_share_directory('manipulator')

    default_model = os.path.join(
        pkg_share,
        'description',
        'capstone_manipulator.urdf.xacro'
    )

    default_rviz = os.path.join(
        pkg_share,
        'rviz',
        'manipulator.rviz'
    )

    # =========================================================
    # Declare launch arguments
    # =========================================================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='Absolute path to robot URDF/Xacro file'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Absolute path to RViz config file'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Use joint_state_publisher_gui if true'
    )

    # =========================================================
    # Robot description
    # =========================================================
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        model
    ])

    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    # =========================================================
    # Robot State Publisher
    # =========================================================
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {
                'use_sim_time': use_sim_time
            }
        ]
    )

    # =========================================================
    # Joint State Publisher GUI
    # =========================================================
    jsp_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # =========================================================
    # Joint State Publisher without GUI
    # =========================================================
    jsp_node = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # =========================================================
    # RViz2
    # =========================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model,
        declare_rviz,
        declare_gui,

        rsp_node,
        jsp_node,
        jsp_gui_node,
        rviz_node,
    ])