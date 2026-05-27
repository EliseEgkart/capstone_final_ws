import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # =========================================================
    # Package paths
    # =========================================================
    realsense_share = get_package_share_directory('realsense2_camera')
    camera_perception_share = get_package_share_directory('camera_perception_pkg')

    # =========================================================
    # Config paths
    # =========================================================
    perception_config = os.path.join(
        camera_perception_share,
        'config',
        'manipulator_perception.yaml'
    )

    # =========================================================
    # RealSense launch path
    # =========================================================
    realsense_launch_path = os.path.join(
        realsense_share,
        'launch',
        'rs_launch.py'
    )

    # =========================================================
    # RealSense D435 launch
    # =========================================================
    # NOTE:
    # - Launch argument에서는 pointcloud.enable 사용
    # - Jetson 런타임에서는 pointcloud__neon_ 파라미터가 실제로 생성될 수 있음
    # - 따라서 아래 TimerAction에서 ros2 param set으로 neon 파라미터를 직접 설정
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',

            'enable_depth': 'true',
            'enable_color': 'true',

            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
        }.items()
    )

    # =========================================================
    # Jetson NEON pointcloud runtime parameter setting
    # =========================================================
    set_neon_pointcloud_params = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'param', 'set',
                    '/camera/camera',
                    'pointcloud__neon_.enable',
                    'true'
                ],
                output='screen'
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'param', 'set',
                    '/camera/camera',
                    'pointcloud__neon_.stream_filter',
                    '2'
                ],
                output='screen'
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'param', 'set',
                    '/camera/camera',
                    'pointcloud__neon_.stream_index_filter',
                    '0'
                ],
                output='screen'
            ),
        ]
    )

    # =========================================================
    # YOLOv8 detection node
    # =========================================================
    yolov8_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_node',
        name='yolov8_node',
        output='screen'
    )

    # =========================================================
    # Object distance node
    # =========================================================
    # Initial target button is loaded from:
    # config/manipulator_perception.yaml
    #
    # Runtime target change is handled by:
    # /manipulator_perception/target_button
    object_distance_node = Node(
        package='camera_perception_pkg',
        executable='object_distance_node',
        name='object_distance_node',
        output='screen',
        parameters=[
            perception_config
        ]
    )

    # =========================================================
    # YOLOv8 debug node
    # =========================================================
    yolov8_debug_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_debug_node',
        name='yolov8_debug_node',
        output='screen'
    )

    # =========================================================
    # Launch order
    # =========================================================
    return LaunchDescription([
        # 1. RealSense 먼저 실행
        TimerAction(
            period=1.0,
            actions=[
                realsense_launch
            ]
        ),

        # 2. RealSense 노드가 뜬 뒤 Jetson NEON pointcloud 파라미터 직접 설정
        set_neon_pointcloud_params,

        # 3. 인식 노드들은 RealSense + pointcloud 설정 이후 실행
        TimerAction(
            period=6.0,
            actions=[
                yolov8_node
            ]
        ),

        TimerAction(
            period=8.0,
            actions=[
                object_distance_node
            ]
        ),

        TimerAction(
            period=9.0,
            actions=[
                yolov8_debug_node
            ]
        ),
    ])
