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
    # RealSense D435 node - Jetson NEON direct mode
    # =========================================================
    # IMPORTANT:
    # 1. Do NOT use rs_setup or ros2 param list/set.
    #    Parameter-service waiting caused a severe startup bottleneck.
    #
    # 2. Do NOT use rs_launch.py for Jetson NEON pointcloud configuration.
    #    Some Jetson/ARM builds expose pointcloud parameters as:
    #      pointcloud__neon_.enable
    #      pointcloud__neon_.stream_filter
    #      pointcloud__neon_.stream_index_filter
    #
    # 3. Use realsense2_camera_node directly and inject pointcloud__neon_
    #    parameters at node startup.
    #
    # 4. publish_tf must be true here.
    #    The URDF provides TF up to camera_link, while RealSense provides
    #    camera optical frames such as camera_depth_optical_frame and
    #    camera_color_optical_frame. If publish_tf is false, RViz can drop
    #    depth/pointcloud messages because optical frame TF is missing.
    #
    # 5. stream_filter=0 creates untextured/depth-only pointcloud.
    #    This avoids:
    #      "No stream match for pointcloud chosen texture Process - Color"
    #    while keeping color stream alive for YOLO.
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
                # Keep the existing topic layout:
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

                # ---------------------------------------------------------
                # Required streams
                # ---------------------------------------------------------
                'enable_depth': True,
                'enable_color': True,
                'align_depth.enable': True,
                'enable_sync': True,

                # ---------------------------------------------------------
                # Jetson / ARM NEON pointcloud parameters
                # ---------------------------------------------------------
                'pointcloud__neon_.enable': True,

                # 0 = untextured/depth-only pointcloud.
                # 2 = color texture, but this caused "No stream match..."
                #     on the current Jetson setup.
                'pointcloud__neon_.stream_filter': 0,
                'pointcloud__neon_.stream_index_filter': 0,

                # Allow pointcloud generation even without texture.
                'pointcloud__neon_.allow_no_texture_points': True,

                # False is lighter and usually enough for visualization.
                'pointcloud__neon_.ordered_pc': False,

                # ---------------------------------------------------------
                # TF
                # ---------------------------------------------------------
                # Keep this true so RealSense publishes optical frames.
                # URDF/robot_state_publisher owns the manipulator mounting TF
                # up to camera_link; RealSense owns camera optical frames.
                'publish_tf': True,

                # 0.0 means static TF behavior for camera frames.
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
    # Uses:
    #   /detections
    #   /camera/camera/aligned_depth_to_color/image_raw
    #   /camera/camera/color/camera_info
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
    # Can be turned on/off:
    #   use_yolo_debug:=true
    #   use_yolo_debug:=false
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
    # No ros2 param list/set.
    #
    # ROS subscriptions will connect automatically when the camera topics appear.
    # This avoids deadlocks from slow parameter service or CLI discovery.
    return LaunchDescription([
        declare_use_yolo_debug,
        declare_initial_reset,

        # 1. RealSense first.
        TimerAction(
            period=1.0,
            actions=[
                realsense_node
            ]
        ),

        # 2. Depth/object-distance node.
        TimerAction(
            period=18.0,
            actions=[
                object_distance_node
            ]
        ),

        # 3. YOLO after camera has had time to stabilize.
        TimerAction(
            period=22.0,
            actions=[
                yolov8_node
            ]
        ),

        # 4. Debug node last, optional.
        TimerAction(
            period=30.0,
            actions=[
                yolov8_debug_node
            ]
        ),
    ])