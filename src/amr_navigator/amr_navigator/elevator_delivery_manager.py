import math
import os
import time
from typing import Dict, Any, Tuple, Optional, List

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap, ClearEntireCostmap


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_distance(a: Dict[str, Any], b: Dict[str, Any]) -> float:
    dx = float(b['x']) - float(a['x'])
    dy = float(b['y']) - float(a['y'])
    return math.hypot(dx, dy)


class ElevatorDeliveryManager(Node):
    def __init__(self):
        super().__init__('elevator_delivery_manager')

        # ------------------------------------------------------------
        # Basic parameters
        # ------------------------------------------------------------
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('start_floor', 1)

        self.declare_parameter('scan_topic', '/rplidar1/scan_filtered')
        self.declare_parameter('current_floor_topic', '/current_floor')
        self.declare_parameter('elevator_start_topic', '/elevator/start')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')

        # ------------------------------------------------------------
        # Door detection parameters
        # ------------------------------------------------------------
        
        self.declare_parameter('approach_door_after_rotate', True)
        self.declare_parameter('door_wait_distance', 0.60)
        self.declare_parameter('door_approach_max_distance', 0.60)
        self.declare_parameter('door_approach_speed', 0.04)
        self.declare_parameter('door_approach_timeout', 20.0)


        self.declare_parameter('door_max_valid_range', 5.0)
        self.declare_parameter('door_min_valid_count', 8)
        self.declare_parameter('treat_max_range_as_open', False)
        # LiDAR scan 기준 문이 정면이면 0.0.
        # 만약 LiDAR frame에서 정면이 180도라면 실행 시 door_center_deg:=180.0 으로 바꿔야 함.
        self.declare_parameter('door_center_deg', 180.0)

        # 기존 12도는 너무 좁을 수 있어 기본값을 25도로 넓힘.
        self.declare_parameter('door_half_width_deg', 8.0)

        # 기존 1.20m는 현장에 따라 너무 클 수 있어 기본값을 0.90m로 낮춤.
        #self.declare_parameter('door_open_distance', 0.90)
        self.declare_parameter('door_open_distance', 1.30)


        # 기존 0.60은 너무 엄격할 수 있어 기본값을 0.35로 낮춤.
        self.declare_parameter('door_min_open_ratio', 0.70)

        # 0.1초 루프 기준 8번 연속이면 약 0.8초 동안 open.
        self.declare_parameter('door_stable_count_required', 20)

        # 수동으로 elevator_inside 근처에 옮겼을 때 door wait를 빠져나오기 위한 반경.
        self.declare_parameter('already_inside_radius', 0.60)

        # ------------------------------------------------------------
        # Direct drive parameters for elevator zone
        # ------------------------------------------------------------
        self.declare_parameter('boarding_speed', 0.12)
        self.declare_parameter('exit_speed', 0.12)

        # 0.0이면 waypoints 거리로 자동 계산.
        # 현장에서 실제 이동거리를 강제로 지정하고 싶으면 예: 1.80
        self.declare_parameter('boarding_distance_override', 0.0)
        self.declare_parameter('exit_distance_override', 0.0)

        # 직진 중 정면 장애물이 이 거리보다 가까우면 정지.
        self.declare_parameter('direct_drive_stop_distance', 0.40)
        self.declare_parameter('direct_drive_check_obstacle', True)

        # ------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------
        waypoint_file = str(self.get_parameter('waypoint_file').value)
        self.start_floor = int(self.get_parameter('start_floor').value)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.current_floor_topic = str(self.get_parameter('current_floor_topic').value)
        self.elevator_start_topic = str(self.get_parameter('elevator_start_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.door_max_valid_range = float(
            self.get_parameter('door_max_valid_range').value
        )
        self.door_min_valid_count = int(
            self.get_parameter('door_min_valid_count').value
        )
        self.treat_max_range_as_open = bool(
            self.get_parameter('treat_max_range_as_open').value
        )

        self.approach_door_after_rotate = bool(
            self.get_parameter('approach_door_after_rotate').value
        )
        self.door_wait_distance = float(
            self.get_parameter('door_wait_distance').value
        )
        self.door_approach_max_distance = float(
            self.get_parameter('door_approach_max_distance').value
        )
        self.door_approach_speed = float(
            self.get_parameter('door_approach_speed').value
        )
        self.door_approach_timeout = float(
            self.get_parameter('door_approach_timeout').value
        )

        self.door_center_deg = float(self.get_parameter('door_center_deg').value)
        self.door_half_width_deg = float(self.get_parameter('door_half_width_deg').value)
        self.door_open_distance = float(self.get_parameter('door_open_distance').value)
        self.door_min_open_ratio = float(self.get_parameter('door_min_open_ratio').value)
        self.door_stable_count_required = int(
            self.get_parameter('door_stable_count_required').value
        )
        self.already_inside_radius = float(
            self.get_parameter('already_inside_radius').value
        )

        self.boarding_speed = float(self.get_parameter('boarding_speed').value)
        self.exit_speed = float(self.get_parameter('exit_speed').value)
        self.boarding_distance_override = float(
            self.get_parameter('boarding_distance_override').value
        )
        self.exit_distance_override = float(
            self.get_parameter('exit_distance_override').value
        )
        self.direct_drive_stop_distance = float(
            self.get_parameter('direct_drive_stop_distance').value
        )
        self.direct_drive_check_obstacle = bool(
            self.get_parameter('direct_drive_check_obstacle').value
        )

        if not waypoint_file:
            waypoint_file = os.path.join(
                get_package_share_directory('amr_navigator'),
                'config',
                'waypoints.yaml'
            )

        with open(waypoint_file, 'r') as f:
            self.wp = yaml.safe_load(f)

        # ------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------
        self.scan: Optional[LaserScan] = None
        self.current_floor: Optional[int] = None
        self.current_pose = None

        # ------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.create_subscription(
            Int32,
            self.current_floor_topic,
            self.floor_callback,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        # ------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------
        self.elevator_start_pub = self.create_publisher(
            Int32,
            self.elevator_start_topic,
            10
        )

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # ------------------------------------------------------------
        # Clients
        # ------------------------------------------------------------
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.load_map_client = self.create_client(
            LoadMap,
            '/map_server/load_map'
        )

        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )

        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )

        self.get_logger().info(
            "ElevatorDeliveryManager ready. "
            f"waypoint_file={waypoint_file}, "
            f"start_floor={self.start_floor}, "
            f"scan_topic={self.scan_topic}, "
            f"cmd_vel_topic={self.cmd_vel_topic}, "
            f"door_center_deg={self.door_center_deg}, "
            f"door_half_width_deg={self.door_half_width_deg}, "
            f"door_open_distance={self.door_open_distance}, "
            f"door_min_open_ratio={self.door_min_open_ratio}, "
            f"boarding_speed={self.boarding_speed}, "
            f"exit_speed={self.exit_speed}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def floor_callback(self, msg: Int32):
        self.current_floor = int(msg.data)

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    # ------------------------------------------------------------------
    # YAML helpers
    # ------------------------------------------------------------------
    def get_floor_info(self, floor: int) -> Dict[str, Any]:
        maps = self.wp.get('maps', {})

        if floor in maps:
            return maps[floor]

        if str(floor) in maps:
            return maps[str(floor)]

        raise KeyError(f"waypoints.yaml에 maps.{floor} 정보가 없습니다.")

    def get_room_and_target_floor(self, room_number: str) -> Tuple[Dict[str, Any], int]:
        room_number = room_number.strip()

        if not room_number:
            raise ValueError("목적 호수가 비어 있습니다.")

        if not room_number[0].isdigit():
            raise ValueError(
                f"목적 호수 '{room_number}'의 첫 글자가 숫자가 아닙니다. "
                "예: 302, 304, 204"
            )

        target_floor = int(room_number[0])

        if target_floor <= 0:
            raise ValueError(
                f"목적 호수 '{room_number}'에서 추출한 목적층이 {target_floor}입니다. "
                "현재 코드는 1층 이상 목적지만 지원합니다."
            )

        rooms = self.wp.get('rooms', {})

        if room_number not in rooms:
            raise KeyError(
                f"waypoints.yaml의 rooms에 '{room_number}'가 없습니다. "
                f'rooms: "{room_number}": 항목을 추가해야 합니다.'
            )

        room = rooms[room_number]

        yaml_floor = room.get('floor', None)
        if yaml_floor is not None:
            try:
                yaml_floor_int = int(yaml_floor)
                if yaml_floor_int != target_floor:
                    self.get_logger().warning(
                        f"방 번호로 계산한 층={target_floor}, "
                        f"waypoints.yaml에 적힌 floor={yaml_floor_int}. "
                        f"이번 미션에서는 방 번호 앞자리 {target_floor}층을 사용합니다."
                    )
            except Exception:
                self.get_logger().warning(
                    f"waypoints.yaml의 rooms['{room_number}'].floor 값을 정수로 해석할 수 없습니다."
                )

        return room, target_floor

    # ------------------------------------------------------------------
    # Pose / map
    # ------------------------------------------------------------------
    def make_pose(self, pose_dict: Dict[str, Any], frame_id: str = 'map') -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pose_dict['x'])
        msg.pose.position.y = float(pose_dict['y'])
        msg.pose.orientation = yaw_to_quaternion(float(pose_dict['yaw']))
        return msg

    def publish_initial_pose(self, pose_dict: Dict[str, Any]):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(pose_dict['x'])
        msg.pose.pose.position.y = float(pose_dict['y'])
        msg.pose.pose.orientation = yaw_to_quaternion(float(pose_dict['yaw']))

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        self.get_logger().info(
            f"Publishing initial pose: "
            f"x={pose_dict['x']}, y={pose_dict['y']}, yaw={pose_dict['yaw']}"
        )

        for _ in range(20):
            self.initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def is_close_to_pose(self, pose_dict: Dict[str, Any], radius: float) -> bool:
        if self.current_pose is None:
            return False

        dx = self.current_pose.position.x - float(pose_dict['x'])
        dy = self.current_pose.position.y - float(pose_dict['y'])
        dist = math.hypot(dx, dy)

        return dist <= radius

    def load_map(self, map_yaml_name: str) -> bool:
        map_path = os.path.join(
            get_package_share_directory('amr_navigator'),
            'map',
            map_yaml_name
        )

        self.get_logger().info(f"Loading map: {map_path}")

        while rclpy.ok() and not self.load_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /map_server/load_map...")

        req = LoadMap.Request()
        req.map_url = map_path

        future = self.load_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()

        if result is None:
            self.get_logger().error("LoadMap service returned None.")
            return False

        self.get_logger().info(f"LoadMap result: {result.result}")

        self.clear_costmaps()
        self.spin_sleep(1.0)
        return True

    def clear_costmaps(self):
        req = ClearEntireCostmap.Request()

        if self.clear_global_costmap_client.wait_for_service(timeout_sec=0.5):
            future = self.clear_global_costmap_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().info("Requested global costmap clear.")

        if self.clear_local_costmap_client.wait_for_service(timeout_sec=0.5):
            future = self.clear_local_costmap_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().info("Requested local costmap clear.")

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------
    def go_to_pose(self, pose_dict: Dict[str, Any], name: str = 'goal') -> bool:
        self.get_logger().info(f"Go to {name}: {pose_dict}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(pose_dict)

        self.nav_client.wait_for_server()

        send_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(f"Goal handle is None: {name}")
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected: {name}")
            return False

        self.get_logger().info(f"Goal accepted: {name}")

        result_future = goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(f"Goal result is None: {name}")
            return False

        status = wrapped_result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal succeeded: {name}")
            return True

        self.get_logger().error(f"Goal failed: {name}, status={status}")
        return False

    # ------------------------------------------------------------------
    # Door detection by LiDAR
    # ------------------------------------------------------------------
    def get_sector_ranges(
        self,
        center_deg: float,
        half_width_deg: float,
        count_inf_as_max: bool = False
    ) -> List[float]:
        if self.scan is None:
            return []

        scan = self.scan
        center = math.radians(center_deg)
        half_width = math.radians(half_width_deg)

        selected = []

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            diff = normalize_angle(angle - center)

            if abs(diff) > half_width:
                continue

            if math.isnan(r):
                continue

            # 핵심 수정:
            # inf 또는 max range는 "문 열림" 증거로 쓰지 않음.
            if math.isinf(r):
                if count_inf_as_max and self.treat_max_range_as_open:
                    selected.append(float(scan.range_max))
                continue

            if r < scan.range_min:
                continue

            # 40.0 같은 max range 값은 문 열림 증거가 아니라
            # 유효하지 않은 값으로 처리.
            if r >= self.door_max_valid_range:
                continue

            if r > scan.range_max:
                continue

            selected.append(float(r))

        return selected

    def get_door_stats(self) -> Dict[str, Any]:
        values = self.get_sector_ranges(
            center_deg=self.door_center_deg,
            half_width_deg=self.door_half_width_deg,
            count_inf_as_max=False
        )

        if len(values) < self.door_min_valid_count:
            return {
                'valid_count': len(values),
                'min': None,
                'median': None,
                'max': None,
                'open_count': 0,
                'open_ratio': 0.0,
                'is_open': False,
            }

        sorted_values = sorted(values)
        n = len(sorted_values)

        median = sorted_values[n // 2]
        min_v = sorted_values[0]
        max_v = sorted_values[-1]

        open_count = sum(
            1 for v in sorted_values
            if v >= self.door_open_distance
        )
        open_ratio = open_count / float(n)

        # 문 열림 조건을 엄격하게 변경:
        # 유효한 finite range가 충분히 있어야 하고,
        # 그중 상당수가 open_distance 이상이어야 함.
        is_open = (
            n >= self.door_min_valid_count
            and open_ratio >= self.door_min_open_ratio
            and median >= self.door_open_distance
        )

        return {
            'valid_count': n,
            'min': min_v,
            'median': median,
            'max': max_v,
            'open_count': open_count,
            'open_ratio': open_ratio,
            'is_open': is_open,
        }

    def is_door_open(self) -> bool:
        stats = self.get_door_stats()
        return bool(stats['is_open'])

    def wait_until_door_open(
        self,
        label: str = 'door',
        already_inside_pose: Optional[Dict[str, Any]] = None
    ) -> bool:
        self.get_logger().info(
            f"Waiting until {label} opens... "
            f"door_center_deg={self.door_center_deg}, "
            f"door_half_width_deg={self.door_half_width_deg}, "
            f"door_open_distance={self.door_open_distance}, "
            f"door_min_open_ratio={self.door_min_open_ratio}"
        )

        stable_count = 0
        last_log_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            # 디버깅 중 수동으로 elevator_inside 근처로 옮긴 경우,
            # door wait에 계속 갇히지 않도록 탈출.
            if already_inside_pose is not None:
                if self.is_close_to_pose(already_inside_pose, self.already_inside_radius):
                    self.get_logger().info(
                        f"Robot is already near elevator_inside. "
                        f"Skip waiting for {label}."
                    )
                    return True

            stats = self.get_door_stats()

            if stats['is_open']:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= self.door_stable_count_required:
                self.get_logger().info(
                    f"{label} opened. "
                    f"valid={stats['valid_count']}, "
                    f"min={stats['min']}, "
                    f"median={stats['median']}, "
                    f"max={stats['max']}, "
                    f"open_ratio={stats['open_ratio']:.2f}"
                )
                return True

            now = time.time()
            if now - last_log_time > 1.0:
                self.get_logger().info(
                    f"Still waiting for {label}... "
                    f"valid={stats['valid_count']}, "
                    f"min={stats['min']}, "
                    f"median={stats['median']}, "
                    f"max={stats['max']}, "
                    f"open_ratio={stats['open_ratio']:.2f}, "
                    f"stable_count={stable_count}/{self.door_stable_count_required}"
                )
                last_log_time = now

        return False

    # ------------------------------------------------------------------
    # Direct drive inside elevator zone
    # ------------------------------------------------------------------
    def get_front_min_distance(self) -> Optional[float]:
        values = self.get_sector_ranges(
            center_deg=self.door_center_deg,
            half_width_deg=10.0,
            count_inf_as_max=False
        )

        if not values:
            return None

        return min(values)

    def stop_robot(self, repeat: int = 20):
        stop = Twist()
        for _ in range(repeat):
            self.cmd_vel_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive_straight_simple(
        self,
        distance_m: float,
        speed_mps: float,
        label: str,
        check_obstacle: Optional[bool] = None
    ) -> bool:
        if check_obstacle is None:
            check_obstacle = self.direct_drive_check_obstacle

        if abs(speed_mps) < 1.0e-6:
            self.get_logger().error("speed_mps is zero. Cannot drive.")
            return False

        direction = 1.0 if distance_m >= 0.0 else -1.0
        speed = abs(speed_mps) * direction
        duration = abs(distance_m) / abs(speed_mps)

        self.get_logger().info(
            f"{label}: direct drive start. "
            f"distance={distance_m:.3f} m, speed={speed:.3f} m/s, duration={duration:.2f} s"
        )

        twist = Twist()
        twist.linear.x = speed

        start = time.time()

        while rclpy.ok() and (time.time() - start) < duration:
            if check_obstacle:
                front_min = self.get_front_min_distance()

                if front_min is None:
                    self.stop_robot()
                    self.get_logger().error(
                        f"{label}: no valid front scan during direct drive. Stop for safety."
                    )
                    return False

                if front_min < self.direct_drive_stop_distance:
                    self.stop_robot()
                    self.get_logger().error(
                        f"{label}: obstacle too close during direct drive. "
                        f"front_min={front_min:.3f}, "
                        f"stop_distance={self.direct_drive_stop_distance:.3f}"
                    )
                    return False

            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        self.get_logger().info(f"{label}: direct drive done.")
        return True
    
    def approach_door_until_wait_distance(self, label: str = 'Approach elevator door') -> bool:
            """
            180도 회전 후 엘리베이터 문을 잘 인식할 수 있도록
            문 쪽으로 천천히 접근한다.

            정지 조건:
            1. door sector median이 door_wait_distance 이하가 됨
            2. direct_drive_stop_distance 이하로 너무 가까워짐
            3. door_approach_max_distance만큼 이동함
            4. timeout
            """
            self.get_logger().info(
                f"{label}: start. "
                f"target door_wait_distance={self.door_wait_distance:.2f} m, "
                f"max_distance={self.door_approach_max_distance:.2f} m, "
                f"speed={self.door_approach_speed:.2f} m/s"
            )

            if self.door_approach_speed <= 0.0:
                self.get_logger().error("door_approach_speed must be positive.")
                return False

            moved_distance = 0.0
            start_time = time.time()
            last_log_time = time.time()

            twist = Twist()
            twist.linear.x = self.door_approach_speed

            while rclpy.ok():
                now = time.time()
                elapsed = now - start_time

                if elapsed > self.door_approach_timeout:
                    self.stop_robot()
                    self.get_logger().warning(
                        f"{label}: timeout. moved_distance={moved_distance:.2f} m"
                    )
                    return True

                moved_distance = elapsed * self.door_approach_speed

                if moved_distance >= self.door_approach_max_distance:
                    self.stop_robot()
                    self.get_logger().info(
                        f"{label}: reached max approach distance. "
                        f"moved_distance={moved_distance:.2f} m"
                    )
                    return True

                stats = self.get_door_stats()
                median = stats.get('median', None)
                min_v = stats.get('min', None)

                # 유효한 문 방향 scan이 있는 경우
                if median is not None:
                    # 너무 가까우면 안전 정지
                    if min_v is not None and min_v <= self.direct_drive_stop_distance:
                        self.stop_robot()
                        self.get_logger().info(
                            f"{label}: stop because min distance is close. "
                            f"min={min_v:.3f}, stop_distance={self.direct_drive_stop_distance:.3f}"
                        )
                        return True

                    # 문 인식 대기 위치에 도달
                    if median <= self.door_wait_distance:
                        self.stop_robot()
                        self.get_logger().info(
                            f"{label}: reached door wait distance. "
                            f"median={median:.3f}, target={self.door_wait_distance:.3f}"
                        )
                        return True

                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.05)

                if now - last_log_time > 1.0:
                    self.get_logger().info(
                        f"{label}: approaching... "
                        f"moved={moved_distance:.2f} m, "
                        f"median={median}, min={min_v}"
                    )
                    last_log_time = now

            self.stop_robot()
            return False

    def rotate_180_simple(self, angular_z: float = 0.35, duration_sec: float = 9.0):
        self.get_logger().info("Rotate 180 deg")

        twist = Twist()
        twist.angular.z = float(angular_z)

        start = time.time()
        while rclpy.ok() and time.time() - start < duration_sec:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        self.get_logger().info("Rotate 180 deg done.")

    # ------------------------------------------------------------------
    # Elevator floor estimator control
    # ------------------------------------------------------------------
    def start_elevator_floor_estimation(self, target_floor: int):
        self.get_logger().info(
            f"Notify elevator_floor_node: start_floor={self.start_floor}, "
            f"target_floor={target_floor}"
        )

        msg = Int32()
        msg.data = int(target_floor)

        for _ in range(15):
            self.elevator_start_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_until_target_floor(self, target_floor: int) -> bool:
        self.get_logger().info(f"Waiting target floor: {target_floor}")

        last_log_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.current_floor == target_floor:
                self.get_logger().info(f"Arrived at target floor: {target_floor}")
                return True

            now = time.time()
            if now - last_log_time > 3.0:
                self.get_logger().info(
                    f"Current estimated floor={self.current_floor}, target={target_floor}"
                )
                last_log_time = now

        return False

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def spin_sleep(self, duration_sec: float):
        end_time = time.time() + duration_sec

        while rclpy.ok() and time.time() < end_time:
            remain = max(0.0, end_time - time.time())
            rclpy.spin_once(self, timeout_sec=min(0.1, remain))

    # ------------------------------------------------------------------
    # Main mission
    # ------------------------------------------------------------------
    def run(self):
        room_number = input("목적 호수를 입력하세요. 예: 302, 304 > ").strip()

        try:
            room, target_floor = self.get_room_and_target_floor(room_number)
            floor1 = self.get_floor_info(self.start_floor)
            target_floor_info = self.get_floor_info(target_floor)
        except Exception as e:
            self.get_logger().error(str(e))
            return

        self.get_logger().info(
            f"Mission requested. room={room_number}, target_floor={target_floor}"
        )

        # 1층 map 로드 및 초기 pose 세팅
        if not self.load_map(floor1['map_yaml']):
            return

        self.publish_initial_pose(floor1['start'])
        self.spin_sleep(2.0)

        # 목적지가 1층인 경우 엘리베이터 없이 바로 이동
        if target_floor == self.start_floor:
            self.get_logger().info(
                f"Target floor is same as start floor: {self.start_floor}. "
                "Skip elevator mission."
            )
            self.go_to_pose(room['pose'], f"room {room_number}")
            return

        # 1층 엘리베이터 앞까지는 Nav2로 이동
        if not self.go_to_pose(floor1['elevator_front'], '1F elevator_front'):
            return

        # 문 열림 대기
        if not self.wait_until_door_open(
            '1F elevator door',
            already_inside_pose=floor1['elevator_inside']
        ):
            return

        # elevator_front -> elevator_inside는 Nav2가 아니라 저속 직진으로 처리
        if self.is_close_to_pose(floor1['elevator_inside'], self.already_inside_radius):
            self.get_logger().info("Already inside elevator. Skip boarding direct drive.")
        else:
            if self.boarding_distance_override > 0.0:
                boarding_distance = self.boarding_distance_override
            else:
                boarding_distance = pose_distance(
                    floor1['elevator_front'],
                    floor1['elevator_inside']
                )

            if not self.drive_straight_simple(
                boarding_distance,
                self.boarding_speed,
                'Boarding elevator'
            ):
                return

        # 실제로 엘리베이터 안에 들어간 뒤 현재 pose를 1층 elevator_inside로 보정
        self.publish_initial_pose(floor1['elevator_inside'])
        self.spin_sleep(1.0)

        # 층수 추정 시작
        self.start_elevator_floor_estimation(target_floor)

        # 문을 바라보도록 180도 회전
        self.rotate_180_simple()

        # 180도 회전 후 문 인식이 잘 되는 위치까지 조금 전진
        if self.approach_door_after_rotate:
            if not self.approach_door_until_wait_distance('Approach door after 180 rotation'):
                return

        # 목적 층 도착 대기
        if not self.wait_until_target_floor(target_floor):
            return

        # 목적 층에서 문 열림 대기
        if not self.wait_until_door_open(f'{target_floor}F elevator door'):
            return

        # 목적 층 map으로 전환
        if not self.load_map(target_floor_info['map_yaml']):
            return

        # 목적 층에서 엘리베이터 내부 pose로 AMCL 초기화
        self.publish_initial_pose(target_floor_info['elevator_inside'])
        self.spin_sleep(2.0)

        # 목적 층 elevator_inside -> elevator_exit도 Nav2 대신 저속 직진
        if self.exit_distance_override > 0.0:
            exit_distance = self.exit_distance_override
        else:
            exit_distance = pose_distance(
                target_floor_info['elevator_inside'],
                target_floor_info['elevator_exit']
            )

        if not self.drive_straight_simple(
            exit_distance,
            self.exit_speed,
            f'Exit elevator at {target_floor}F'
        ):
            return

        # 엘리베이터 밖으로 나온 뒤 pose를 elevator_exit로 보정하고 costmap clear
        self.publish_initial_pose(target_floor_info['elevator_exit'])
        self.clear_costmaps()
        self.spin_sleep(2.0)

        # 목적 호수로 이동
        if not self.go_to_pose(room['pose'], f'room {room_number}'):
            return

        self.get_logger().info("Mission complete.")


def main():
    rclpy.init()
    node = ElevatorDeliveryManager()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()