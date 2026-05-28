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
    #   That warning is not critical for this task and is intentionally ignored.
    # - GroupAction(scoped=True, forwarding=False) prevents parent launch args
    #   from leaking into rs_launch.py.
    realsense_launch_arguments = {
        'depth_module.depth_profile': '640x480x15',
        'rgb_camera.color_profile': '640x480x15',

        'enable_depth': 'true',
        'enable_color': 'true',
        'align_depth.enable': 'true',
        'enable_sync': 'true',

        # Jetson pointcloud is enabled later after stream_filter/index setup.
        'pointcloud.enable': 'false',

        # USB/device recovery options.
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
    # RealSense runtime setup
    # =========================================================
    # This process waits until /camera/camera is alive, then:
    #   1. enables Jetson NEON pointcloud safely
    #   2. waits until color and aligned depth topics exist
    #
    # NOTE:
    # - rgb_camera.power_line_frequency is intentionally not handled here.
    realsense_runtime_setup_process = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            """
set -u

NODE="/camera/camera"
LIST_TIMEOUT="2s"
SET_TIMEOUT="3s"
GET_TIMEOUT="2s"

COLOR_TOPIC="/camera/camera/color/image_raw"
DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"

echo "[rs_setup] waiting for RealSense parameter service..."

NODE_READY="false"

for i in $(seq 1 120); do
    if timeout "${LIST_TIMEOUT}" ros2 param list "$NODE" >/tmp/realsense_params.txt 2>/dev/null; then
        NODE_READY="true"
        echo "[rs_setup] RealSense parameter service is ready"
        break
    fi

    echo "[rs_setup] waiting node... ${i}/120"
    sleep 0.5
done

if [ "$NODE_READY" != "true" ]; then
    echo "[rs_setup] ERROR: RealSense parameter service not ready"
    exit 1
fi

set_param_retry() {
    PARAM_NAME="$1"
    PARAM_VALUE="$2"
    REQUIRED="$3"

    for j in $(seq 1 6); do
        echo "[rs_setup] set ${PARAM_NAME}=${PARAM_VALUE} try ${j}/6"
        if timeout "${SET_TIMEOUT}" ros2 param set "$NODE" "${PARAM_NAME}" "${PARAM_VALUE}"; then
            return 0
        fi

        sleep 0.5
    done

    if [ "$REQUIRED" = "required" ]; then
        echo "[rs_setup] ERROR: failed to set required parameter ${PARAM_NAME}"
        return 1
    fi

    echo "[rs_setup] WARN: failed to set optional parameter ${PARAM_NAME}"
    return 0
}

echo "[rs_setup] checking pointcloud parameter prefix..."

PC_PREFIX=""

for i in $(seq 1 60); do
    if timeout "${LIST_TIMEOUT}" ros2 param list "$NODE" >/tmp/realsense_params.txt 2>/dev/null; then
        if grep -q "pointcloud__neon_.enable" /tmp/realsense_params.txt; then
            PC_PREFIX="pointcloud__neon_"
            echo "[rs_setup] using Jetson NEON pointcloud prefix: ${PC_PREFIX}"
            break
        fi

        if grep -q "pointcloud.enable" /tmp/realsense_params.txt; then
            PC_PREFIX="pointcloud"
            echo "[rs_setup] using standard pointcloud prefix: ${PC_PREFIX}"
            break
        fi
    fi

    echo "[rs_setup] waiting pointcloud params... ${i}/60"
    sleep 0.5
done

if [ -n "$PC_PREFIX" ]; then
    set_param_retry "${PC_PREFIX}.stream_filter" 2 "required" || true
    set_param_retry "${PC_PREFIX}.stream_index_filter" 0 "required" || true
    set_param_retry "${PC_PREFIX}.enable" true "required" || true

    echo "[rs_setup] pointcloud verification with timeout"
    timeout "${GET_TIMEOUT}" ros2 param get "$NODE" "${PC_PREFIX}.stream_filter" || true
    timeout "${GET_TIMEOUT}" ros2 param get "$NODE" "${PC_PREFIX}.stream_index_filter" || true
    timeout "${GET_TIMEOUT}" ros2 param get "$NODE" "${PC_PREFIX}.enable" || true
else
    echo "[rs_setup] WARN: pointcloud parameters were not found; continuing with image/depth pipeline"
fi

echo "[rs_setup] waiting for color/depth topics..."

COLOR_READY="false"
DEPTH_READY="false"

for i in $(seq 1 90); do
    TOPICS="$(ros2 topic list 2>/dev/null || true)"

    if echo "$TOPICS" | grep -Fxq "$COLOR_TOPIC"; then
        COLOR_READY="true"
    fi

    if echo "$TOPICS" | grep -Fxq "$DEPTH_TOPIC"; then
        DEPTH_READY="true"
    fi

    if [ "$COLOR_READY" = "true" ] && [ "$DEPTH_READY" = "true" ]; then
        echo "[rs_setup] color/depth topics are ready"
        break
    fi

    echo "[rs_setup] waiting topics... ${i}/90 color=${COLOR_READY} depth=${DEPTH_READY}"
    sleep 0.5
done

if [ "$COLOR_READY" != "true" ]; then
    echo "[rs_setup] ERROR: color topic was not observed: ${COLOR_TOPIC}"
    exit 1
fi

if [ "$DEPTH_READY" != "true" ]; then
    echo "[rs_setup] ERROR: aligned depth topic was not observed: ${DEPTH_TOPIC}"
    exit 1
fi

echo "[rs_setup] RealSense runtime setup done"
exit 0
"""
        ],
        output='screen'
    )

    realsense_runtime_setup = TimerAction(
        period=8.0,
        actions=[
            realsense_runtime_setup_process
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
    # NOTE:
    # - Can be turned on/off by use_yolo_debug.
    # - Default is true because this system currently needs debug visibility.
    # - If Jetson CUDA memory becomes unstable, run with use_yolo_debug:=false.
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
    # Start perception nodes only after RealSense runtime setup
    # =========================================================
    start_perception_after_realsense_setup = RegisterEventHandler(
        OnProcessExit(
            target_action=realsense_runtime_setup_process,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[
                        object_distance_node
                    ]
                ),
                TimerAction(
                    period=4.0,
                    actions=[
                        yolov8_node
                    ]
                ),
                TimerAction(
                    period=9.0,
                    actions=[
                        yolov8_debug_node
                    ]
                ),
            ]
        )
    )

    # =========================================================
    # Launch order
    # =========================================================
    return LaunchDescription([
        declare_use_yolo_debug,

        TimerAction(
            period=1.0,
            actions=[
                realsense_launch
            ]
        ),

        realsense_runtime_setup,

        start_perception_after_realsense_setup,
    ])