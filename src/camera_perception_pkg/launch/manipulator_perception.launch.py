import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


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
    # Package paths / configs
    # =========================================================
    camera_perception_share = get_package_share_directory('camera_perception_pkg')

    perception_config = os.path.join(
        camera_perception_share,
        'config',
        'manipulator_perception.yaml'
    )

    # =========================================================
    # RealSense D435 node - Jetson NEON direct mode
    # =========================================================
    # IMPORTANT:
    # - Use direct realsense2_camera_node instead of rs_launch.py so Jetson
    #   pointcloud__neon_ parameters are injected before the node starts.
    # - Keep color stream enabled for YOLO.
    # - Use untextured/depth-only pointcloud by stream_filter=0.
    #   This avoids "No stream match for pointcloud chosen texture Process - Color"
    #   when color frames are dropped or not synchronized with pointcloud.
    # - Explicitly set camera_name/camera_namespace to preserve /camera/camera/*
    #   topic layout used by the existing perception YAML.
    # - publish_tf=false protects the manipulator TF tree because camera frames
    #   are already provided by URDF / robot_state_publisher.
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                # Preserve expected topic layout:
                # /camera/camera/color/image_raw
                # /camera/camera/aligned_depth_to_color/image_raw
                # /camera/camera/depth/points or /camera/camera/depth/color/points
                'camera_name': 'camera',
                'camera_namespace': 'camera',

                # Stream profiles
                'depth_module.depth_profile': '640x480x15',
                'rgb_camera.color_profile': '640x480x15',

                # Required streams
                'enable_depth': True,
                'enable_color': True,
                'align_depth.enable': True,
                'enable_sync': True,

                # Jetson / ARM NEON pointcloud parameters.
                # stream_filter=0: untextured/depth-only pointcloud.
                # Keep color stream separately for YOLO.
                'pointcloud__neon_.enable': True,
                'pointcloud__neon_.stream_filter': 0,
                'pointcloud__neon_.stream_index_filter': 0,
                'pointcloud__neon_.allow_no_texture_points': True,
                'pointcloud__neon_.ordered_pc': False,

                # TF safety: robot/manipulator URDF owns camera_link transform.
                'publish_tf': False,
                'tf_publish_rate': 0.0,

                # USB/device recovery
                'initial_reset': initial_reset,
                'wait_for_device_timeout': -1.0,
                'reconnect_timeout': 6.0,
            }
        ]
    )

    # =========================================================
    # Topic gate
    # =========================================================
    # Do not query /camera/camera parameter services.
    # Wait only for actual ROS topics.
    wait_camera_topics_process = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            """
set -u

# Expected topic layout
COLOR_TOPIC_1="/camera/camera/color/image_raw"
DEPTH_TOPIC_1="/camera/camera/aligned_depth_to_color/image_raw"
POINT_TOPIC_1="/camera/camera/depth/color/points"
POINT_TOPIC_2="/camera/camera/depth/points"
POINT_TOPIC_3="/camera/camera/points"

# Fallback topic layout, in case direct node publishes without duplicated camera name.
COLOR_TOPIC_2="/camera/color/image_raw"
DEPTH_TOPIC_2="/camera/aligned_depth_to_color/image_raw"
POINT_TOPIC_4="/camera/depth/color/points"
POINT_TOPIC_5="/camera/depth/points"
POINT_TOPIC_6="/camera/points"

echo "[topic_gate] waiting for color/depth/pointcloud topics..."

COLOR_READY="false"
DEPTH_READY="false"
POINT_READY="false"

COLOR_FOUND=""
DEPTH_FOUND=""
POINT_FOUND=""

for i in $(seq 1 180); do
    TOPICS="$(ros2 topic list 2>/dev/null || true)"

    if echo "$TOPICS" | grep -Fxq "$COLOR_TOPIC_1"; then
        COLOR_READY="true"
        COLOR_FOUND="$COLOR_TOPIC_1"
    elif echo "$TOPICS" | grep -Fxq "$COLOR_TOPIC_2"; then
        COLOR_READY="true"
        COLOR_FOUND="$COLOR_TOPIC_2"
    fi

    if echo "$TOPICS" | grep -Fxq "$DEPTH_TOPIC_1"; then
        DEPTH_READY="true"
        DEPTH_FOUND="$DEPTH_TOPIC_1"
    elif echo "$TOPICS" | grep -Fxq "$DEPTH_TOPIC_2"; then
        DEPTH_READY="true"
        DEPTH_FOUND="$DEPTH_TOPIC_2"
    fi

    for PT in "$POINT_TOPIC_1" "$POINT_TOPIC_2" "$POINT_TOPIC_3" "$POINT_TOPIC_4" "$POINT_TOPIC_5" "$POINT_TOPIC_6"; do
        if echo "$TOPICS" | grep -Fxq "$PT"; then
            POINT_READY="true"
            POINT_FOUND="$PT"
            break
        fi
    done

    if [ "$COLOR_READY" = "true" ] && [ "$DEPTH_READY" = "true" ] && [ "$POINT_READY" = "true" ]; then
        echo "[topic_gate] color/depth/pointcloud topics are ready"
        echo "[topic_gate] color topic     : ${COLOR_FOUND}"
        echo "[topic_gate] aligned depth   : ${DEPTH_FOUND}"
        echo "[topic_gate] pointcloud topic: ${POINT_FOUND}"

        if [ "$COLOR_FOUND" != "$COLOR_TOPIC_1" ]; then
            echo "[topic_gate] WARN: color topic is not in expected /camera/camera layout"
        fi

        if [ "$DEPTH_FOUND" != "$DEPTH_TOPIC_1" ]; then
            echo "[topic_gate] WARN: depth topic is not in expected /camera/camera layout"
        fi

        exit 0
    fi

    echo "[topic_gate] waiting... ${i}/180 color=${COLOR_READY} depth=${DEPTH_READY} pointcloud=${POINT_READY}"
    sleep 0.5
done

echo "[topic_gate] ERROR: required camera topics were not ready"
echo "[topic_gate] current camera topics:"
ros2 topic list 2>/dev/null | grep -E "camera|point|points|cloud|depth|color" || true
exit 1
"""
        ],
        output='screen'
    )

    wait_camera_topics = TimerAction(
        period=8.0,
        actions=[wait_camera_topics_process]
    )

    # =========================================================
    # YOLO/PyTorch environment for Jetson stability
    # =========================================================
    yolo_env = {
        'CUDA_MODULE_LOADING': 'LAZY',
        'PYTORCH_CUDA_ALLOC_CONF': 'max_split_size_mb:64,garbage_collection_threshold:0.8',
    }

    # =========================================================
    # Perception nodes
    # =========================================================
    object_distance_node = Node(
        package='camera_perception_pkg',
        executable='object_distance_node',
        name='object_distance_node',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
        parameters=[perception_config],
    )

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
    # Start perception nodes after camera topics are ready
    # =========================================================
    start_perception_after_topic_gate = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_camera_topics_process,
            on_exit=[
                TimerAction(period=1.0, actions=[object_distance_node]),
                TimerAction(period=4.0, actions=[yolov8_node]),
                TimerAction(period=9.0, actions=[yolov8_debug_node]),
            ]
        )
    )

    return LaunchDescription([
        declare_use_yolo_debug,
        declare_initial_reset,

        TimerAction(period=1.0, actions=[realsense_node]),

        wait_camera_topics,

        start_perception_after_topic_gate,
    ])