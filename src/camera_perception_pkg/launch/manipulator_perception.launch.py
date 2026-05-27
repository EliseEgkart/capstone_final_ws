import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
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
    # - Jetson/ARM 환경에서는   ointcloud 파라미터가
    #   pointcloud__neon_.enable 형태로 생성될 수 있음.
    # - pointcloud를 launch argument에서 바로 켜면 stream filter 설정 전에
    #   먼저 활성화될 수 있으므로, 여기서는 끄고 아래 설정 블록에서
    #   stream_filter -> stream_index_filter -> enable 순서로 켠다.
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
            # 'rgb_camera.color_profile': '1280x720x15',
            'enable_depth': 'true',
            'enable_color': 'true',

            'pointcloud.enable': 'false',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
        }.items()
    )

    # =========================================================
    # Jetson NEON pointcloud runtime parameter setting
    # =========================================================
    # NOTE:
    # - 기존처럼 ros2 param set 3개를 동시에 따로 실행하지 않는다.
    # - /camera/camera 노드와 pointcloud__neon_ 파라미터가 실제로 생길 때까지 기다린다.
    # - stream_filter, stream_index_filter를 먼저 설정한 뒤 enable을 마지막에 true로 바꾼다.
    # - 설정 후 ros2 param get으로 적용 여부를 출력한다.
    set_neon_pointcloud_process = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            r'''
set -u

NODE="/camera/camera"
PC_PREFIX="pointcloud__neon_"

echo "[pcfg] waiting for RealSense node and Jetson NEON pointcloud parameters..."

FOUND="false"

for i in $(seq 1 60); do
    if ros2 param list "$NODE" >/tmp/realsense_params.txt 2>/dev/null; then
        if grep -q "${PC_PREFIX}.enable" /tmp/realsense_params.txt; then
            FOUND="true"
            echo "[pcfg] found ${PC_PREFIX} parameters"
            break
        fi
    fi

    echo "[pcfg] waiting... ${i}/60"
    sleep 0.5
done

if [ "$FOUND" != "true" ]; then
    echo "[pcfg] ERROR: ${PC_PREFIX}.enable was not found"
    echo "[pcfg] available pointcloud parameters:"
    ros2 param list "$NODE" 2>/dev/null | grep pointcloud || true
    exit 1
fi

echo "[pcfg] setting pointcloud stream parameters first"
ros2 param set "$NODE" "${PC_PREFIX}.stream_filter" 2
ros2 param set "$NODE" "${PC_PREFIX}.stream_index_filter" 0

echo "[pcfg] enabling Jetson NEON pointcloud"
ros2 param set "$NODE" "${PC_PREFIX}.enable" true

echo "[pcfg] verification"
ros2 param get "$NODE" "${PC_PREFIX}.stream_filter"
ros2 param get "$NODE" "${PC_PREFIX}.stream_index_filter"
ros2 param get "$NODE" "${PC_PREFIX}.enable"

echo "[pcfg] Jetson NEON pointcloud configuration done"
'''
        ],
        output='screen'
    )

    set_neon_pointcloud_params = TimerAction(
        period=4.0,
        actions=[
            set_neon_pointcloud_process
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
    # Start perception nodes after pointcloud parameter setup
    # =========================================================
    # NOTE:
    # - TimerAction만으로는 순서를 완전히 보장하기 어렵다.
    # - 따라서 pointcloud 설정 프로세스가 끝난 뒤 인식 노드를 실행한다.
    start_perception_after_pointcloud_setup = RegisterEventHandler(
        OnProcessExit(
            target_action=set_neon_pointcloud_process,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[
                        yolov8_node
                    ]
                ),
                TimerAction(
                    period=2.0,
                    actions=[
                        object_distance_node
                    ]
                ),
                TimerAction(
                    period=3.0,
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
        # 1. RealSense 먼저 실행
        TimerAction(
            period=1.0,
            actions=[
                realsense_launch
            ]
        ),

        # 2. RealSense 노드와 pointcloud__neon_ 파라미터가 준비될 때까지 대기 후 설정
        set_neon_pointcloud_params,

        # 3. pointcloud 설정 프로세스가 종료된 뒤 인식 노드 실행
        start_perception_after_pointcloud_setup,
    ])