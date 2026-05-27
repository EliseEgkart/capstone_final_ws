from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('amr_navigator'),
            'params',
            'points7.yaml'
        ]),
        description='Path to navigator parameter file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    navigator_node = Node(
        package='amr_navigator',
        executable='navigator_node',
        name='navigator',                # 여기!
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        navigator_node
    ])
