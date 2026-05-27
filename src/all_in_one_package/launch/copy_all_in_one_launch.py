import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    serial_launch = os.path.join(get_package_share_directory('serial_test'), 'serial_test.launch.py')
    imu_ekf_launch = os.path.join(get_package_share_directory('ebimu_pkg'), 'launch', 'ebimu_ekf.launch.py')
    lidar_launch = os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_2_launch.py')
    # merger_launch = os.path.join(get_package_share_directory('ros2_laser_scan_merger'), 'launch', 'merge_2_scan.launch.py')
    localization_launch = os.path.join(
        get_package_share_directory('amr_navigator'),
        'launch',
        'nav2_bringup',
        'localization_launch.py',
    )
    navigation_launch = os.path.join(
        get_package_share_directory('amr_navigator'),
        'launch',
        'nav2_bringup',
        'navigation_launch.py',
    )
    map_yaml = os.path.join(get_package_share_directory('amr_navigator'), 'map', 'ff_ekf_2f_map.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(serial_launch)),

        TimerAction(
            period=1.0,
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(imu_ekf_launch))],
        ),

        TimerAction(
            period=2.0,
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch))],
        ),

        # TimerAction(
        #     period=4.0,
        #     actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(merger_launch))],
        # ),

        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(localization_launch),
                    launch_arguments={'map': map_yaml}.items(),
                )
            ],
        ),

        TimerAction(
            period=8.0,
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(navigation_launch))],
        ),
    ])