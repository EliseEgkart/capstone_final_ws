import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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
    # 나중에 False 해야함
    declare_use_yolo_debug = DeclareLaunchArgument(
        'use_yolo_debug',
        default_value='true',
        description='Start yolov8_debug_node. Keep false on Jetson for stable YOLO/CUDA memory.'
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
    # NOTE:
    # - Jetson/ARM 환경에서는 pointcloud 파라미터가
    #   pointcloud__neon_.enable 형태로 생성될 수 있음.
    # - pointcloud를 launch argument에서 바로 켜면 stream filter 설정 전에
    #   먼저 활성화될 수 있으므로, 여기서는 끄고 아래 설정 블록에서
    #   stream_filter -> stream_index_filter -> enable 순서로 켠다.
    # - rgb_camera.power_line_frequency는 허용 범위가 0~2이므로 2로 고정한다.
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',

            'enable_depth': 'true',
            'enable_color': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true',

            # Jetson pointcloud는 아래 bash 블록에서 안정적으로 켠다.
            'pointcloud.enable': 'false',

            # Korean 60 Hz environment. Do not use 3; RealSense allows only 0~2.
            'rgb_camera.power_line_frequency': '2',

            # Stability options for intermittent USB/device initialization.
            'initial_reset': 'true',
            'wait_for_device_timeout': '-1.0',
            'reconnect_timeout': '6.0',
        }.items()
    )

    # =========================================================
    # Jetson NEON pointcloud runtime parameter setting
    # =========================================================
    # NOTE:
    # - 기존처럼 ros2 param set 3개를 동시에 따로 실행하지 않는다.
    # - /camera/camera 노드와 pointcloud 파라미터가 실제로 생길 때까지 기다린다.
    # - Jetson NEON(pointcloud__neon_)을 우선 사용하고, 없으면 일반 pointcloud로 fallback한다.
    # - stream_filter, stream_index_filter를 먼저 설정한 뒤 enable을 마지막에 true로 바꾼다.
    # - 검증 단계에서 ros2 param get이 멈추지 않도록 timeout을 건다.
    set_neon_pointcloud_process = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            """
set -u

NODE="/camera/camera"
PC_PREFIX=""
LIST_TIMEOUT="2s"
SET_TIMEOUT="3s"
GET_TIMEOUT="2s"

echo "[pcfg] waiting for RealSense node and pointcloud parameters..."

FOUND="false"

for i in $(seq 1 90); do
    if timeout "${LIST_TIMEOUT}" ros2 param list "$NODE" >/tmp/realsense_params.txt 2>/dev/null; then
        if grep -q "pointcloud__neon_.enable" /tmp/realsense_params.txt; then
            PC_PREFIX="pointcloud__neon_"
            FOUND="true"
            echo "[pcfg] found Jetson NEON pointcloud parameters: ${PC_PREFIX}"
            break
        fi

        if grep -q "pointcloud.enable" /tmp/realsense_params.txt; then
            PC_PREFIX="pointcloud"
            FOUND="true"
            echo "[pcfg] found standard pointcloud parameters: ${PC_PREFIX}"
            break
        fi
    fi

    echo "[pcfg] waiting... ${i}/90"
    sleep 0.5
done

if [ "$FOUND" != "true" ]; then
    echo "[pcfg] ERROR: pointcloud parameter was not found"
    echo "[pcfg] available pointcloud parameters:"
    timeout "${LIST_TIMEOUT}" ros2 param list "$NODE" 2>/dev/null | grep pointcloud || true
    exit 1
fi

set_param_retry() {
    PARAM_NAME="$1"
    PARAM_VALUE="$2"

    for j in $(seq 1 5); do
        echo "[pcfg] set ${PARAM_NAME}=${PARAM_VALUE} try ${j}/5"
        if timeout "${SET_TIMEOUT}" ros2 param set "$NODE" "${PARAM_NAME}" "${PARAM_VALUE}"; then
            return 0
        fi

        sleep 0.5
    done

    echo "[pcfg] ERROR: failed to set ${PARAM_NAME}"
    return 1
}

echo "[pcfg] setting pointcloud stream parameters first"
set_param_retry "${PC_PREFIX}.stream_filter" 2 || exit 1
set_param_retry "${PC_PREFIX}.stream_index_filter" 0 || exit 1

echo "[pcfg] enabling pointcloud"
set_param_retry "${PC_PREFIX}.enable" true || exit 1

echo "[pcfg] verification with timeout"
timeout "${GET_TIMEOUT}" ros2 param get "$NODE" "${PC_PREFIX}.stream_filter" || true
timeout "${GET_TIMEOUT}" ros2 param get "$NODE" "${PC_PREFIX}.stream_index_filter" || true
timeout "${GET_TIMEOUT}" ros2 param get "$NODE" "${PC_PREFIX}.enable" || true

echo "[pcfg] pointcloud configuration done"
exit 0
"""
        ],
        output='screen'
    )

    set_neon_pointcloud_params = TimerAction(
        period=6.0,
        actions=[
            set_neon_pointcloud_process
        ]
    )

    # =========================================================
    # Common YOLO/PyTorch environment for Jetson stability
    # =========================================================
    # - CUDA_MODULE_LOADING=LAZY reduces first-load memory pressure.
    # - PYTORCH_CUDA_ALLOC_CONF helps reduce CUDA memory fragmentation.
    yolo_env = {
        'CUDA_MODULE_LOADING': 'LAZY',
        'PYTORCH_CUDA_ALLOC_CONF': 'max_split_size_mb:64,garbage_collection_threshold:0.8',
    }

    # =========================================================
    # YOLOv8 detection node
    # =========================================================
    # NOTE:
    # - respawn=True makes the node come back if CUDA/cuDNN allocation fails once.
    # - Keep yolov8_debug_node disabled by default to reduce Jetson GPU memory pressure.
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
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            perception_config
        ]
    )

    # =========================================================
    # YOLOv8 debug node
    # =========================================================
    # NOTE:
    # - Disabled by default because Jetson can fail YOLO warmup with CUDA/cuDNN
    #   allocation errors when debug visualization and RViz are also running.
    # - Enable only when needed:
    #   ros2 launch camera_perception_pkg manipulator_perception.launch.py use_yolo_debug:=true
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
    # Start perception nodes after pointcloud parameter setup
    # =========================================================
    # NOTE:
    # - TimerAction만으로는 순서를 완전히 보장하기 어렵다.
    # - 따라서 pointcloud 설정 프로세스가 끝난 뒤 인식 노드를 실행한다.
    # - object_distance_node를 먼저 띄워 depth subscription을 준비하고,
    #   YOLO는 2초 뒤 시작해 Jetson 초기 부하를 줄인다.
    start_perception_after_pointcloud_setup = RegisterEventHandler(
        OnProcessExit(
            target_action=set_neon_pointcloud_process,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[
                        object_distance_node
                    ]
                ),
                TimerAction(
                    period=3.0,
                    actions=[
                        yolov8_node
                    ]
                ),
                TimerAction(
                    period=8.0,
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

        # 1. RealSense 먼저 실행
        TimerAction(
            period=1.0,
            actions=[
                realsense_launch
            ]
        ),

        # 2. RealSense 노드와 pointcloud 파라미터가 준비될 때까지 대기 후 설정
        set_neon_pointcloud_params,

        # 3. pointcloud 설정 프로세스가 종료된 뒤 인식 노드 실행
        start_perception_after_pointcloud_setup,
    ])