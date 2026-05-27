import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    serial_launch = os.path.join(
        get_package_share_directory('serial_test'),
        'serial_test.launch.py'
    )

    imu_ekf_launch = os.path.join(
        get_package_share_directory('ebimu_pkg'),
        'launch',
        'ebimu_ekf.launch.py'
    )

    lidar_launch = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_c1_2_launch.py'
    )

    localization_launch = os.path.join(
        get_package_share_directory('amr_navigator'),
        'launch',
        'nav2_bringup',
        'localization_launch.py'
    )

    navigation_launch = os.path.join(
        get_package_share_directory('amr_navigator'),
        'launch',
        'nav2_bringup',
        'navigation_launch.py'
    )

    default_map_yaml = os.path.join(
        get_package_share_directory('amr_navigator'),
        'map',
        'ff_ekf_3f.yaml'
    )

    default_params_file = os.path.join(
        get_package_share_directory('amr_navigator'),
        'params',
        'nav2_params.yaml'
    )

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml,
        description='Full path to the map yaml file. Default is ff_ekf_3f.yaml'
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to nav2_params.yaml'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup Nav2 lifecycle nodes'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS log level'
    )

    elevator_floor_node = Node(
        package='ebimu_pkg',
        executable='elevator_floor_node',
        name='elevator_floor_node',
        output='screen',
        parameters=[{
            'imu_topic': '/imu/data',
            'start_floor': 3,
            'acc_z_threshold': 0.02,
            'thresh_count': 30,
            'window': 0.5,
            'use_baseline_compensation': True,
            'baseline_duration': 0.8,
        }]
    )

    return LaunchDescription([
        declare_map_arg,
        declare_params_file_arg,
        declare_use_sim_time_arg,
        declare_autostart_arg,
        declare_log_level_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(serial_launch)
        ),

        TimerAction(
            period=1.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(imu_ekf_launch)
                )
            ],
        ),

        TimerAction(
            period=3.0,
            actions=[
                elevator_floor_node
            ],
        ),

        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_launch)
                )
            ],
        ),

        TimerAction(
            period=7.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(localization_launch),
                    launch_arguments={
                        'map': map_yaml,
                        'params_file': params_file,
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'log_level': log_level,
                    }.items(),
                )
            ],
        ),

        TimerAction(
            period=9.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_launch),
                    launch_arguments={
                        'params_file': params_file,
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'log_level': log_level,
                    }.items(),
                )
            ],
        ),
    ])