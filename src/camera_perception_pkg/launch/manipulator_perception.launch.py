import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # =========================================================
    # Launch arguments
    # =========================================================
    use_yolo_debug = LaunchConfiguration('use_yolo_debug')

    declare_use_yolo_debug = DeclareLaunchArgument(
        'use_yolo_debug',
        default_value='true',
        description='If true, start yolov8_debug_node. Set false to reduce Jetson GPU/CPU load.'
    )

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
    # IMPORTANT:
    # - Do not pass use_yolo_debug to RealSense.
    # - Do not handle rgb_camera.power_line_frequency here.
    # - Do not block perception startup on RealSense parameter service.
    # - publish_tf=false is critical when the camera frames are already provided
    #   by the robot URDF / robot_state_publisher. Otherwise RealSense can publish
    #   duplicate camera_link/camera_* frames and break the manipulator TF tree.
    # - GroupAction(scoped=True, forwarding=False) prevents parent launch args
    #   from leaking into rs_launch.py.
    realsense_launch_arguments = {
        'depth_module.depth_profile': '640x480x15',
        'rgb_camera.color_profile': '640x480x15',
        'enable_depth': 'true',
        'enable_color': 'true',
        'align_depth.enable': 'true',
        'enable_sync': 'true',
        'pointcloud.enable': 'false',
        'publish_tf': 'false',
        'tf_publish_rate': '0.0',
        'initial_reset': 'true',
        'wait_for_device_timeout': '-1.0',
        'reconnect_timeout': '6.0',
    }

    realsense_launch = GroupAction(
        scoped=True,
        forwarding=False,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch_path),
                launch_arguments=realsense_launch_arguments.items()
            )
        ]
    )

    # =========================================================
    # Wait only for image/depth topics
    # =========================================================
    wait_image_depth_topics_process = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            """
set -u

COLOR_TOPIC="/camera/camera/color/image_raw"
DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"

echo "[topic_gate] waiting for color/aligned-depth topics..."

COLOR_READY="false"
DEPTH_READY="false"

for i in $(seq 1 120); do
    TOPICS="$(ros2 topic list 2>/dev/null || true)"

    if echo "$TOPICS" | grep -Fxq "$COLOR_TOPIC"; then
        COLOR_READY="true"
    fi

    if echo "$TOPICS" | grep -Fxq "$DEPTH_TOPIC"; then
        DEPTH_READY="true"
    fi

    if [ "$COLOR_READY" = "true" ] && [ "$DEPTH_READY" = "true" ]; then
        echo "[topic_gate] color/depth topics are ready"
        exit 0
    fi

    echo "[topic_gate] waiting... ${i}/120 color=${COLOR_READY} depth=${DEPTH_READY}"
    sleep 0.5
done

echo "[topic_gate] ERROR: required camera topics were not ready"
echo "[topic_gate] expected color: ${COLOR_TOPIC}"
echo "[topic_gate] expected depth : ${DEPTH_TOPIC}"
exit 1
"""
        ],
        output='screen'
    )

    wait_image_depth_topics = TimerAction(
        period=6.0,
        actions=[
            wait_image_depth_topics_process
        ]
    )

    # =========================================================
    # Optional non-blocking pointcloud setup
    # =========================================================
    pointcloud_optional_setup_process = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            """
set -u

NODE="/camera/camera"
LIST_TIMEOUT="1s"
SET_TIMEOUT="2s"

echo "[pc_optional] trying optional pointcloud setup without blocking perception..."

if ! timeout "${LIST_TIMEOUT}" ros2 param list "$NODE" >/tmp/realsense_params_optional.txt 2>/dev/null; then
    echo "[pc_optional] parameter service not ready; skip pointcloud setup"
    exit 0
fi

PC_PREFIX=""

if grep -q "pointcloud__neon_.enable" /tmp/realsense_params_optional.txt; then
    PC_PREFIX="pointcloud__neon_"
    echo "[pc_optional] using Jetson NEON prefix: ${PC_PREFIX}"
elif grep -q "pointcloud.enable" /tmp/realsense_params_optional.txt; then
    PC_PREFIX="pointcloud"
    echo "[pc_optional] using standard prefix: ${PC_PREFIX}"
else
    echo "[pc_optional] pointcloud params not found; skip"
    exit 0
fi

timeout "${SET_TIMEOUT}" ros2 param set "$NODE" "${PC_PREFIX}.stream_filter" 2 || true
timeout "${SET_TIMEOUT}" ros2 param set "$NODE" "${PC_PREFIX}.stream_index_filter" 0 || true
timeout "${SET_TIMEOUT}" ros2 param set "$NODE" "${PC_PREFIX}.enable" true || true

echo "[pc_optional] optional pointcloud setup done"
exit 0
"""
        ],
        output='screen'
    )

    pointcloud_optional_setup = TimerAction(
        period=12.0,
        actions=[
            pointcloud_optional_setup_process
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
        parameters=[perception_config]
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
    # Start perception nodes only after image/depth topics are ready
    # =========================================================
    start_perception_after_topic_gate = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_image_depth_topics_process,
            on_exit=[
                TimerAction(period=1.0, actions=[object_distance_node]),
                TimerAction(period=4.0, actions=[yolov8_node]),
                TimerAction(period=9.0, actions=[yolov8_debug_node]),
            ]
        )
    )

    return LaunchDescription([
        declare_use_yolo_debug,

        TimerAction(period=1.0, actions=[realsense_launch]),

        wait_image_depth_topics,

        pointcloud_optional_setup,

        start_perception_after_topic_gate,
    ])