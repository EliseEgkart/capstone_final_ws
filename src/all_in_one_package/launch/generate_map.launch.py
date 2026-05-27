import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    cartographer_launch = os.path.join(
        get_package_share_directory('amr_cartographer'),
        'launch',
        'amr_cartographer.launch.py'
    )

    return LaunchDescription([
        # 1. serial_test 내부에서 조이스틱 + 모터/휠 odom 관련 노드 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(serial_launch)
        ),

        # 2. IMU publisher + robot_localization EKF 실행
        #    EKF가 /wheel/odometry + /imu/data 를 받아 /odom 생성
        TimerAction(
            period=1.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(imu_ekf_launch)
                )
            ],
        ),

        # 3. LiDAR 실행
        TimerAction(
            period=2.5,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_launch)
                )
            ],
        ),

        # 4. Cartographer는 EKF /odom 이 뜬 뒤 실행되도록 약간 늦게 시작
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(cartographer_launch)
                )
            ],
        ),
    ])