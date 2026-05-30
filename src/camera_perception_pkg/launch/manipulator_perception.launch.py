import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # =========================================================
    # Launch arguments
    # =========================================================
    use_yolo_debug = LaunchConfiguration('use_yolo_debug')
    initial_reset = LaunchConfiguration('initial_reset')
    enable_realsense_sync = LaunchConfiguration('enable_realsense_sync')

    declare_use_yolo_debug = DeclareLaunchArgument(
        'use_yolo_debug',
        default_value='true',
        description='If true, start yolov8_debug_node. Set false to reduce Jetson GPU/CPU load.'
    )

    declare_initial_reset = DeclareLaunchArgument(
        'initial_reset',
        default_value='true',
        description='If true, reset RealSense once during startup for USB recovery.'
    )

    declare_enable_realsense_sync = DeclareLaunchArgument(
        'enable_realsense_sync',
        default_value='false',
        description='If true, enable RealSense frame sync. False is usually more stable on Jetson.'
    )

    # =========================================================
    # Package paths
    # =========================================================
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
    # RealSense D435 node - Jetson NEON direct mbest_finalode
    # =========================================================
    # IMPORTANT:
    # - This launch does not use rs_launch.py.
    # - This launch does not use runtime "ros2 param list/set".
    # - Jetson/ARM NEON pointcloud parameters are injected directly at node startup.
    #
    # Required core settings:
    #   depth_module.depth_profile = 640x480x15
    #   align_depth.enable = True
    #   pointcloud__neon_.enable = True
    #
    # Note:
    # - If rs_launch.py is used, the argument name is pointcloud.enable.
    # - In this direct-node Jetson mode, use pointcloud__neon_.enable.
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                # ---------------------------------------------------------
                # Preserve existing topic layout:
                #   /camera/camera/color/image_raw
                #   /camera/camera/aligned_depth_to_color/image_raw
                # ---------------------------------------------------------
                'camera_name': 'camera',
                'camera_namespace': 'camera',

                # ---------------------------------------------------------
                # Stream profiles
                # ---------------------------------------------------------
                'depth_module.depth_profile': '640x480x15',
                'rgb_camera.color_profile': '640x480x15',
                'rgb_camera.power_line_frequency': 2,

                # ---------------------------------------------------------
                # RGB manual exposure / white balance
                # Same values previously used through rs_launch.py
                # ---------------------------------------------------------
                'rgb_camera.enable_auto_exposure': False,
                'rgb_camera.exposure': 380,
                'rgb_camera.gain': 40,
                'rgb_camera.enable_auto_white_balance': True,
                #'rgb_camera.white_balance': 5680,
                

                # D435 may still open infra streams internally through the
                # depth module. Keep them lighter than the default 848x480x30.
                'depth_module.infra_profile': '640x480x15',

                # ---------------------------------------------------------
                # Required streams
                # ---------------------------------------------------------
                'enable_depth': True,
                'enable_color': True,
                'align_depth.enable': True,

                # Explicitly request no published infra image topics.
                'enable_infra': False,
                'enable_infra1': False,
                'enable_infra2': False,

                # On Jetson, sync can increase frame timeout pressure.
                'enable_sync': ParameterValue(enable_realsense_sync, value_type=bool),

                # ---------------------------------------------------------
                # Jetson / ARM NEON pointcloud parameters
                # ---------------------------------------------------------
                'pointcloud__neon_.enable': True,

                # 1 = depth stream as pointcloud source.
                # 2 = color texture, unstable on current Jetson setup.
                # 0 = any, caused "Process - Any" warning.
                'pointcloud__neon_.stream_filter': 2,
                'pointcloud__neon_.stream_index_filter': 0,

                # Keep pointcloud generation robust even without color texture.
                'pointcloud__neon_.allow_no_texture_points': True,
                'pointcloud__neon_.ordered_pc': False,

                # ---------------------------------------------------------
                # TF
                # ---------------------------------------------------------
                # URDF owns mounting TF up to camera_link.
                # RealSense owns optical frames such as camera_depth_optical_frame.
                'publish_tf': True,
                'tf_publish_rate': 0.0,

                # ---------------------------------------------------------
                # USB/device recovery
                # ---------------------------------------------------------
                'initial_reset': ParameterValue(initial_reset, value_type=bool),
                'wait_for_device_timeout': -1.0,
                'reconnect_timeout': 6.0,
            }
        ]
    )

    # =========================================================
    # YOLO/PyTorch environment for Jetson stability
    # =========================================================
    yolo_env = {
        'CUDA_MODULE_LOADING': 'LAZY',
        'PYTORCH_CUDA_ALLOC_CONF': 'max_split_size_mb:64,garbage_collection_threshold:0.8',
    }

    # =========================================================
    # Object distance node
    # =========================================================
    object_distance_node = Node(
        package='camera_perception_pkg',
        executable='object_distance_node',
        name='object_distance_node',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            perception_config
        ]
    )

    # =========================================================
    # YOLOv8 detection node
    # =========================================================
    yolov8_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_node',
        name='yolov8_node',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=5.0,
        additional_env=yolo_env,
    )

    # =========================================================
    # YOLOv8 debug node
    # =========================================================
    yolov8_debug_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_debug_node',
        name='yolov8_debug_node',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=5.0,
        additional_env=yolo_env,
        condition=IfCondition(use_yolo_debug),
    )

    # =========================================================
    # Launch order
    # =========================================================
    # No rs_setup.
    # No topic_gate.
    # No runtime parameter service dependency.
    #
    # ROS subscriptions connect automatically when camera topics appear.
    return LaunchDescription([
        declare_use_yolo_debug,
        declare_initial_reset,
        declare_enable_realsense_sync,

        TimerAction(
            period=1.0,
            actions=[
                realsense_node
            ]
        ),

        TimerAction(
            period=12.0,
            actions=[
                object_distance_node
            ]
        ),

        TimerAction(
            period=18.0,
            actions=[
                yolov8_node
            ]
        ),

        TimerAction(
            period=23.0,
            actions=[
                yolov8_debug_node
            ]
        ),
    ])