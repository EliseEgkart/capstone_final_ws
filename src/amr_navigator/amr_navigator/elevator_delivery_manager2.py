#!/usr/bin/env python3
import math
import os
import time
from typing import Dict, Any, Optional, List, Union

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


FloorKey = Union[int, str]
PoseDict = Dict[str, Any]


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


def pose_distance(a: PoseDict, b: PoseDict) -> float:
    dx = float(b['x']) - float(a['x'])
    dy = float(b['y']) - float(a['y'])
    return math.hypot(dx, dy)


def pose_bearing(a: PoseDict, b: PoseDict) -> float:
    dx = float(b['x']) - float(a['x'])
    dy = float(b['y']) - float(a['y'])
    return math.atan2(dy, dx)


class ElevatorDeliveryManager2(Node):
    """
    3F room_front -> elevator_btn_front -> elevator_front -> elevator_inside
    -> elevator_btn_inside -> elevator_inside_for_exit -> B1 elevator_exit
    -> B1 destination 연속 동작 시나리오.

    waypoint2/waypoints2/wayypoints2 YAML의 maps 구조를 사용한다.
    현재 저장소에는 config/wayypoints2.yaml 이름으로 올라와 있으므로,
    waypoint_file 파라미터가 비어 있으면 세 이름을 순서대로 탐색한다.
    """

    def __init__(self):
        super().__init__('elevator_delivery_manager2')

        # ------------------------------------------------------------
        # Basic parameters
        # ------------------------------------------------------------
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('start_floor', 3)
        self.declare_parameter('target_floor_key', 'B1')
        self.declare_parameter('target_floor_signal', 0)

        self.declare_parameter('scan_topic', '/rplidar1/scan_filtered')
        self.declare_parameter('current_floor_topic', '/current_floor')
        self.declare_parameter('elevator_start_topic', '/elevator/start')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')

        # ------------------------------------------------------------
        # Scenario timing parameters
        # ------------------------------------------------------------
        self.declare_parameter('front_button_wait_sec', 10.0)
        self.declare_parameter('inside_button_wait_sec', 5.0)

        # ------------------------------------------------------------
        # Door detection parameters
        # ------------------------------------------------------------
        self.declare_parameter('door_max_valid_range', 10.0)
        self.declare_parameter('door_min_valid_count', 8)
        self.declare_parameter('treat_max_range_as_open', False)

        # LiDAR scan 기준 문이 정면이면 0.0.
        # 기존 코드와 같은 기본값을 유지한다.
        # 만약 LiDAR frame에서 정면이 0도라면 실행 시 door_center_deg:=0.0 으로 바꾼다.
        self.declare_parameter('door_center_deg', 180.0)
        self.declare_parameter('door_half_width_deg', 8.0)
        self.declare_parameter('door_open_distance', 1.30)
        self.declare_parameter('door_min_open_ratio', 0.70)
        self.declare_parameter('door_stable_count_required', 20)
        self.declare_parameter('already_inside_radius', 0.60)

        # ------------------------------------------------------------
        # Direct / forced drive parameters
        # ------------------------------------------------------------
        self.declare_parameter('boarding_speed', 0.18)
        self.declare_parameter('forced_move_speed', 0.16)
        self.declare_parameter('exit_speed', 0.18)
        self.declare_parameter('rotate_speed', 0.35)

        # elevator_btn_front -> elevator_front 강제 이동 파라미터
        self.declare_parameter('front_approach_speed', 0.18)
        self.declare_parameter('front_approach_distance_override', 0.0)

        # 0.0이면 waypoint 거리로 자동 계산.
        # 현장에서 실제 이동거리를 강제로 지정하고 싶으면 예: 1.80
        self.declare_parameter('boarding_distance_override', 0.0)
        self.declare_parameter('exit_distance_override', 0.0)


        # 강제 이동은 Nav2가 아니라 cmd_vel open-loop 제어이다.
        # forced_drive_check_obstacle=False이면 장애물 검사 없이 지정 거리만큼 이동한다.
        # 사람이 오가는 환경에서는 True로 바꾸는 것을 권장한다.
        self.declare_parameter('forced_drive_check_obstacle', False)
        self.declare_parameter('direct_drive_stop_distance', 0.40)

        # AMCL 보정 관련
        self.declare_parameter('publish_initial_pose_before_forced_move', True)
        self.declare_parameter('publish_initial_pose_after_forced_move', True)
        self.declare_parameter('initial_pose_sleep_sec', 0.5)

        # ------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------
        waypoint_file_param = str(self.get_parameter('waypoint_file').value)
        self.start_floor = int(self.get_parameter('start_floor').value)
        self.target_floor_key = str(self.get_parameter('target_floor_key').value)
        self.target_floor_signal = int(self.get_parameter('target_floor_signal').value)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.current_floor_topic = str(self.get_parameter('current_floor_topic').value)
        self.elevator_start_topic = str(self.get_parameter('elevator_start_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.front_button_wait_sec = float(self.get_parameter('front_button_wait_sec').value)
        self.inside_button_wait_sec = float(self.get_parameter('inside_button_wait_sec').value)

        self.door_max_valid_range = float(self.get_parameter('door_max_valid_range').value)
        self.door_min_valid_count = int(self.get_parameter('door_min_valid_count').value)
        self.treat_max_range_as_open = bool(self.get_parameter('treat_max_range_as_open').value)
        self.door_center_deg = float(self.get_parameter('door_center_deg').value)
        self.door_half_width_deg = float(self.get_parameter('door_half_width_deg').value)
        self.door_open_distance = float(self.get_parameter('door_open_distance').value)
        self.door_min_open_ratio = float(self.get_parameter('door_min_open_ratio').value)
        self.door_stable_count_required = int(self.get_parameter('door_stable_count_required').value)
        self.already_inside_radius = float(self.get_parameter('already_inside_radius').value)

        self.boarding_speed = float(self.get_parameter('boarding_speed').value)
        self.forced_move_speed = float(self.get_parameter('forced_move_speed').value)
        self.exit_speed = float(self.get_parameter('exit_speed').value)
        self.rotate_speed = float(self.get_parameter('rotate_speed').value)

        self.front_approach_speed = float(
            self.get_parameter('front_approach_speed').value
        )
        
        self.front_approach_distance_override = float(
            self.get_parameter('front_approach_distance_override').value
        )


        self.boarding_distance_override = float(self.get_parameter('boarding_distance_override').value)
        self.exit_distance_override = float(self.get_parameter('exit_distance_override').value)
        self.forced_drive_check_obstacle = bool(self.get_parameter('forced_drive_check_obstacle').value)
        self.direct_drive_stop_distance = float(self.get_parameter('direct_drive_stop_distance').value)

        self.publish_initial_pose_before_forced_move = bool(
            self.get_parameter('publish_initial_pose_before_forced_move').value
        )
        self.publish_initial_pose_after_forced_move = bool(
            self.get_parameter('publish_initial_pose_after_forced_move').value
        )
        self.initial_pose_sleep_sec = float(self.get_parameter('initial_pose_sleep_sec').value)

        self.waypoint_file = self.resolve_waypoint_file(waypoint_file_param)

        with open(self.waypoint_file, 'r') as f:
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
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Int32, self.current_floor_topic, self.floor_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        # ------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------
        self.elevator_start_pub = self.create_publisher(Int32, self.elevator_start_topic, 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ------------------------------------------------------------
        # Clients
        # ------------------------------------------------------------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.load_map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )

        self.get_logger().info(
            'ElevatorDeliveryManager2 ready. '
            f'waypoint_file={self.waypoint_file}, '
            f'start_floor={self.start_floor}, '
            f'target_floor_key={self.target_floor_key}, '
            f'target_floor_signal={self.target_floor_signal}, '
            f'scan_topic={self.scan_topic}, '
            f'cmd_vel_topic={self.cmd_vel_topic}, '
            f'door_center_deg={self.door_center_deg}, '
            f'door_open_distance={self.door_open_distance}, '
            f'forced_drive_check_obstacle={self.forced_drive_check_obstacle}'
        )

    # ------------------------------------------------------------------
    # Parameter / YAML helpers
    # ------------------------------------------------------------------
    def resolve_waypoint_file(self, waypoint_file_param: str) -> str:
        if waypoint_file_param:
            return waypoint_file_param

        config_dir = os.path.join(
            get_package_share_directory('amr_navigator'),
            'config'
        )

        # 사용자가 말한 이름과 현재 GitHub에 올라온 오타 이름을 모두 허용한다.
        candidates = [
            'waypoint2.yaml',
            'waypoints2.yaml',
            'wayypoints2.yaml',
        ]

        for filename in candidates:
            path = os.path.join(config_dir, filename)
            if os.path.exists(path):
                return path

        # 여기까지 오면 파일이 없다는 것을 명확히 로그로 남기기 위해 마지막 후보를 반환한다.
        return os.path.join(config_dir, 'waypoint2.yaml')

    def _floor_key_candidates(self, floor: FloorKey) -> List[FloorKey]:
        candidates: List[FloorKey] = [floor]

        if isinstance(floor, int):
            candidates.append(str(floor))
            if floor == 0:
                candidates.extend(['B1', 'b1', 'Basement1', 'basement1'])
        else:
            floor_str = str(floor)
            candidates.append(floor_str)
            if floor_str.isdigit():
                candidates.append(int(floor_str))
            if floor_str.upper() == 'B1':
                candidates.extend([0, '0'])

        deduped: List[FloorKey] = []
        for item in candidates:
            if item not in deduped:
                deduped.append(item)
        return deduped

    def get_floor_info(self, floor: FloorKey) -> Dict[str, Any]:
        maps = self.wp.get('maps', {})

        for key in self._floor_key_candidates(floor):
            if key in maps:
                return maps[key]

        raise KeyError(
            f"waypoint2 YAML에 maps.{floor} 정보가 없습니다. "
            f"현재 maps 키={list(maps.keys())}"
        )

    def require_keys(self, data: Dict[str, Any], keys: List[str], label: str):
        missing = [key for key in keys if key not in data]
        if missing:
            raise KeyError(f'{label}에 필요한 키가 없습니다: {missing}')

    def require_pose(self, floor_info: Dict[str, Any], pose_key: str, label: str) -> PoseDict:
        if pose_key not in floor_info:
            raise KeyError(f'{label}에 {pose_key} waypoint가 없습니다.')

        pose = floor_info[pose_key]
        for field in ('x', 'y', 'yaw'):
            if field not in pose:
                raise KeyError(f'{label}.{pose_key}.{field} 값이 없습니다.')

        return pose

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
    # Pose / map
    # ------------------------------------------------------------------
    def make_pose(self, pose_dict: PoseDict, frame_id: str = 'map') -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pose_dict['x'])
        msg.pose.position.y = float(pose_dict['y'])
        msg.pose.orientation = yaw_to_quaternion(float(pose_dict['yaw']))
        return msg

    def publish_initial_pose(self, pose_dict: PoseDict):
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
            'Publishing initial pose: '
            f"x={pose_dict['x']}, y={pose_dict['y']}, yaw={pose_dict['yaw']}"
        )

        for _ in range(20):
            self.initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def is_close_to_pose(self, pose_dict: PoseDict, radius: float) -> bool:
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

        self.get_logger().info(f'Loading map: {map_path}')

        while rclpy.ok() and not self.load_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /map_server/load_map...')

        req = LoadMap.Request()
        req.map_url = map_path

        future = self.load_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result is None:
            self.get_logger().error('LoadMap service returned None.')
            return False

        self.get_logger().info(f'LoadMap result: {result.result}')
        self.clear_costmaps()
        self.spin_sleep(1.0)
        return True

    def clear_costmaps(self):
        req = ClearEntireCostmap.Request()

        if self.clear_global_costmap_client.wait_for_service(timeout_sec=0.5):
            future = self.clear_global_costmap_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().info('Requested global costmap clear.')

        if self.clear_local_costmap_client.wait_for_service(timeout_sec=0.5):
            future = self.clear_local_costmap_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().info('Requested local costmap clear.')

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------
    def go_to_pose(self, pose_dict: PoseDict, name: str = 'goal') -> bool:
        self.get_logger().info(f'Go to {name}: {pose_dict}')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(pose_dict)

        self.nav_client.wait_for_server()

        send_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(f'Goal handle is None: {name}')
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected: {name}')
            return False

        self.get_logger().info(f'Goal accepted: {name}')
        result_future = goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(f'Goal result is None: {name}')
            return False

        status = wrapped_result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal succeeded: {name}')
            return True

        self.get_logger().error(f'Goal failed: {name}, status={status}')
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
        selected: List[float] = []

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            diff = normalize_angle(angle - center)

            if abs(diff) > half_width:
                continue

            if math.isnan(r):
                continue

            if math.isinf(r):
                if count_inf_as_max and self.treat_max_range_as_open:
                    selected.append(float(scan.range_max))
                continue

            if r < scan.range_min:
                continue

            # max range 값은 문 열림 증거가 아니라 유효하지 않은 값으로 처리한다.
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

        open_count = sum(1 for v in sorted_values if v >= self.door_open_distance)
        open_ratio = open_count / float(n)

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

    def wait_until_door_open(
        self,
        label: str = 'door',
        already_inside_pose: Optional[PoseDict] = None
    ) -> bool:
        self.get_logger().info(
            f'Waiting until {label} opens... '
            f'door_center_deg={self.door_center_deg}, '
            f'door_half_width_deg={self.door_half_width_deg}, '
            f'door_open_distance={self.door_open_distance}, '
            f'door_min_open_ratio={self.door_min_open_ratio}'
        )

        stable_count = 0
        last_log_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if already_inside_pose is not None:
                if self.is_close_to_pose(already_inside_pose, self.already_inside_radius):
                    self.get_logger().info(
                        f'Robot is already near target inside pose. Skip waiting for {label}.'
                    )
                    return True

            stats = self.get_door_stats()

            if stats['is_open']:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= self.door_stable_count_required:
                self.get_logger().info(
                    f'{label} opened. '
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
                    f'Still waiting for {label}... '
                    f"valid={stats['valid_count']}, "
                    f"min={stats['min']}, "
                    f"median={stats['median']}, "
                    f"max={stats['max']}, "
                    f"open_ratio={stats['open_ratio']:.2f}, "
                    f'stable_count={stable_count}/{self.door_stable_count_required}'
                )
                last_log_time = now

        return False

    def wait_until_target_floor_then_door_open(self, target_floor: int, label: str) -> bool:
        """
        지하 1층을 의미하는 current_floor=0을 받은 뒤에만 문 열림 stable count를 센다.
        그 전 층에서 문이 열리더라도 stable_count를 0으로 유지하므로 하차 동작으로 넘어가지 않는다.
        """
        self.get_logger().info(
            f'Waiting target floor={target_floor} and {label} open. '
            'Door-open events before target floor will be ignored.'
        )

        stable_count = 0
        last_log_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()

            if self.current_floor != target_floor:
                stable_count = 0
                if now - last_log_time > 3.0:
                    self.get_logger().info(
                        f'Not target floor yet. current_floor={self.current_floor}, '
                        f'target_floor={target_floor}. Ignore door state.'
                    )
                    last_log_time = now
                continue

            stats = self.get_door_stats()

            if stats['is_open']:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= self.door_stable_count_required:
                self.get_logger().info(
                    f'Target floor {target_floor} reached and {label} opened. '
                    f"valid={stats['valid_count']}, "
                    f"median={stats['median']}, "
                    f"open_ratio={stats['open_ratio']:.2f}"
                )
                return True

            if now - last_log_time > 1.0:
                self.get_logger().info(
                    f'At target floor={target_floor}; waiting for {label}... '
                    f"valid={stats['valid_count']}, "
                    f"median={stats['median']}, "
                    f"open_ratio={stats['open_ratio']:.2f}, "
                    f'stable_count={stable_count}/{self.door_stable_count_required}'
                )
                last_log_time = now

        return False

    # ------------------------------------------------------------------
    # Direct / forced drive
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

    def rotate_relative_simple(self, angle_rad: float, label: str = 'Rotate') -> bool:
        angle = normalize_angle(angle_rad)

        if abs(angle) < 1.0e-3:
            self.get_logger().info(f'{label}: angle is almost zero. Skip rotate.')
            return True

        if self.rotate_speed <= 0.0:
            self.get_logger().error('rotate_speed must be positive.')
            return False

        direction = 1.0 if angle >= 0.0 else -1.0
        angular_z = abs(self.rotate_speed) * direction
        duration = abs(angle) / abs(self.rotate_speed)

        self.get_logger().info(
            f'{label}: rotate start. '
            f'angle={angle:.3f} rad, angular_z={angular_z:.3f} rad/s, duration={duration:.2f} s'
        )

        twist = Twist()
        twist.angular.z = angular_z

        start = time.time()
        while rclpy.ok() and (time.time() - start) < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        self.get_logger().info(f'{label}: rotate done.')
        return True

    def drive_straight_simple(
        self,
        distance_m: float,
        speed_mps: float,
        label: str,
        check_obstacle: Optional[bool] = None
    ) -> bool:
        if check_obstacle is None:
            check_obstacle = self.forced_drive_check_obstacle

        if abs(speed_mps) < 1.0e-6:
            self.get_logger().error('speed_mps is zero. Cannot drive.')
            return False

        direction = 1.0 if distance_m >= 0.0 else -1.0
        speed = abs(speed_mps) * direction
        duration = abs(distance_m) / abs(speed_mps)

        self.get_logger().info(
            f'{label}: direct drive start. '
            f'distance={distance_m:.3f} m, speed={speed:.3f} m/s, '
            f'duration={duration:.2f} s, check_obstacle={check_obstacle}'
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
                        f'{label}: no valid front scan during direct drive. Stop for safety.'
                    )
                    return False

                if front_min < self.direct_drive_stop_distance:
                    self.stop_robot()
                    self.get_logger().error(
                        f'{label}: obstacle too close during direct drive. '
                        f'front_min={front_min:.3f}, '
                        f'stop_distance={self.direct_drive_stop_distance:.3f}'
                    )
                    return False

            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        self.get_logger().info(f'{label}: direct drive done.')
        return True

    def force_move_between_waypoints(
        self,
        from_pose: PoseDict,
        to_pose: PoseDict,
        label: str,
        speed_mps: float,
        distance_override: float = 0.0,
        check_obstacle: Optional[bool] = None
    ) -> bool:
        """
        Nav2를 쓰지 않고 cmd_vel로 다음 waypoint까지 강제 이동한다.
        1) from_pose yaw 기준으로 목표 좌표 방향으로 open-loop 회전
        2) 거리만큼 직진
        3) to_pose yaw로 open-loop 회전
        4) AMCL pose를 to_pose로 보정
        """
        if check_obstacle is None:
            check_obstacle = self.forced_drive_check_obstacle

        if self.publish_initial_pose_before_forced_move:
            self.publish_initial_pose(from_pose)
            self.spin_sleep(self.initial_pose_sleep_sec)

        from_yaw = float(from_pose['yaw'])
        to_yaw = float(to_pose['yaw'])
        bearing = pose_bearing(from_pose, to_pose)
        distance = float(distance_override) if distance_override > 0.0 else pose_distance(from_pose, to_pose)

        self.get_logger().info(
            f'{label}: forced move. '
            f'from_yaw={from_yaw:.3f}, bearing={bearing:.3f}, '
            f'to_yaw={to_yaw:.3f}, distance={distance:.3f}'
        )

        first_turn = normalize_angle(bearing - from_yaw)
        if not self.rotate_relative_simple(first_turn, f'{label} first turn'):
            return False

        if distance > 1.0e-3:
            if not self.drive_straight_simple(
                distance,
                speed_mps,
                label,
                check_obstacle=check_obstacle
            ):
                return False

        final_turn = normalize_angle(to_yaw - bearing)
        if not self.rotate_relative_simple(final_turn, f'{label} final turn'):
            return False

        if self.publish_initial_pose_after_forced_move:
            self.publish_initial_pose(to_pose)
            self.spin_sleep(self.initial_pose_sleep_sec)

        return True

    # ------------------------------------------------------------------
    # Elevator floor estimator control
    # ------------------------------------------------------------------
    def start_elevator_floor_estimation(self, target_floor: int):
        self.get_logger().info(
            f'Notify elevator_floor_node: start_floor={self.start_floor}, '
            f'target_floor_signal={target_floor}'
        )

        # 이전 미션 또는 테스트에서 남은 0층 값 때문에 바로 하차하지 않도록 초기화한다.
        self.current_floor = None

        msg = Int32()
        msg.data = int(target_floor)

        for _ in range(15):
            self.elevator_start_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def spin_sleep(self, duration_sec: float):
        end_time = time.time() + duration_sec

        while rclpy.ok() and time.time() < end_time:
            remain = max(0.0, end_time - time.time())
            rclpy.spin_once(self, timeout_sec=min(0.1, remain))

    def compute_exit_distance(self, start_floor_info: Dict[str, Any], target_floor_info: Dict[str, Any]) -> float:
        if self.exit_distance_override > 0.0:
            self.get_logger().info(
                f'Use exit_distance_override={self.exit_distance_override:.3f} m'
            )
            return self.exit_distance_override

        # B1 YAML에 elevator_inside 또는 elevator_inside_for_exit가 추가되면 그 값을 우선 사용한다.
        for inside_key in ('elevator_inside_for_exit', 'elevator_inside'):
            if inside_key in target_floor_info and 'elevator_exit' in target_floor_info:
                distance = pose_distance(target_floor_info[inside_key], target_floor_info['elevator_exit'])
                self.get_logger().info(
                    f'Use target floor {inside_key}->elevator_exit distance={distance:.3f} m'
                )
                return distance

        # 현재 wayypoints2.yaml에는 B1 elevator_inside가 없으므로,
        # 같은 엘리베이터 구조라고 가정하고 3F의 exit standby -> elevator_front 거리를 사용한다.
        if 'elevator_inside_for_exit' in start_floor_info and 'elevator_front' in start_floor_info:
            distance = pose_distance(start_floor_info['elevator_inside_for_exit'], start_floor_info['elevator_front'])
            self.get_logger().warning(
                'Target floor has no elevator_inside/elevator_inside_for_exit. '
                'Use 3F elevator_inside_for_exit -> elevator_front distance as exit distance. '
                f'distance={distance:.3f} m. Tune exit_distance_override if needed.'
            )
            return distance

        if 'elevator_inside' in start_floor_info and 'elevator_front' in start_floor_info:
            distance = pose_distance(start_floor_info['elevator_inside'], start_floor_info['elevator_front'])
            self.get_logger().warning(
                'Target floor has no elevator_inside/elevator_inside_for_exit. '
                'Use 3F elevator_inside -> elevator_front distance as exit distance. '
                f'distance={distance:.3f} m. Tune exit_distance_override if needed.'
            )
            return distance

        raise KeyError(
            'Cannot compute exit distance. Add B1 elevator_inside/elevator_inside_for_exit '
            'to waypoint2 YAML or pass exit_distance_override.'
        )

    # ------------------------------------------------------------------
    # Main mission
    # ------------------------------------------------------------------
    def run(self):
        try:
            start_info = self.get_floor_info(self.start_floor)
            target_info = self.get_floor_info(self.target_floor_key)

            self.require_keys(
                start_info,
                [
                    'map_yaml',
                    'room_front',
                    'elevator_btn_front',
                    'elevator_front',
                    'elevator_inside',
                    'elevator_btn_inside',
                    'elevator_inside_for_exit',
                ],
                f'{self.start_floor}F map info'
            )
            self.require_keys(
                target_info,
                ['map_yaml', 'elevator_exit', 'destination'],
                f'{self.target_floor_key} map info'
            )

            room_front = self.require_pose(start_info, 'room_front', f'{self.start_floor}F')
            elevator_btn_front = self.require_pose(start_info, 'elevator_btn_front', f'{self.start_floor}F')
            elevator_front = self.require_pose(start_info, 'elevator_front', f'{self.start_floor}F')
            elevator_inside = self.require_pose(start_info, 'elevator_inside', f'{self.start_floor}F')
            elevator_btn_inside = self.require_pose(start_info, 'elevator_btn_inside', f'{self.start_floor}F')
            elevator_inside_for_exit = self.require_pose(
                start_info,
                'elevator_inside_for_exit',
                f'{self.start_floor}F'
            )
            elevator_exit = self.require_pose(target_info, 'elevator_exit', self.target_floor_key)
            destination = self.require_pose(target_info, 'destination', self.target_floor_key)

        except Exception as e:
            self.get_logger().error(str(e))
            return

        self.get_logger().info(
            f'Mission requested. start_floor={self.start_floor}, '
            f'target_floor_key={self.target_floor_key}, '
            f'target_floor_signal={self.target_floor_signal}'
        )

        # 1) 3층 map 로드 및 시작 pose 세팅: room_front
        if not self.load_map(start_info['map_yaml']):
            return

        self.publish_initial_pose(room_front)
        self.spin_sleep(2.0)

        # 2) room_front -> elevator_btn_front: Nav2 이동
        if not self.go_to_pose(elevator_btn_front, f'{self.start_floor}F elevator_btn_front'):
            return

        # 3) elevator_btn_front에서 10초 대기 후 elevator_front로 이동
        # 3) elevator_btn_front에서 10초 대기 후 elevator_front로 강제 이동
        self.get_logger().info(
            f'Arrived at elevator_btn_front. Wait {self.front_button_wait_sec:.1f} sec.'
        )
        self.spin_sleep(self.front_button_wait_sec)

        # elevator_btn_front -> elevator_front도 Nav2가 아니라 cmd_vel 강제 이동으로 처리
        if not self.force_move_between_waypoints(
            elevator_btn_front,
            elevator_front,
            'Forced approach: elevator_btn_front -> elevator_front',
            self.front_approach_speed,
            distance_override=self.front_approach_distance_override,
            check_obstacle=self.forced_drive_check_obstacle
        ):
            return

        # self.get_logger().info(
        #     f'Arrived at elevator_btn_front. Wait {self.front_button_wait_sec:.1f} sec.'
        # )
        # self.spin_sleep(self.front_button_wait_sec)

        # if not self.go_to_pose(elevator_front, f'{self.start_floor}F elevator_front'):
        #     return

        # 4) 3층 엘리베이터 문 열림 대기
        if not self.wait_until_door_open(
            f'{self.start_floor}F elevator door',
            already_inside_pose=elevator_inside
        ):
            return

        # 5) elevator_front -> elevator_inside: 강제 이동
        if self.is_close_to_pose(elevator_inside, self.already_inside_radius):
            self.get_logger().info('Already inside elevator. Skip forced boarding move.')
            self.publish_initial_pose(elevator_inside)
            self.spin_sleep(self.initial_pose_sleep_sec)
        else:
            if not self.force_move_between_waypoints(
                elevator_front,
                elevator_inside,
                'Boarding elevator: elevator_front -> elevator_inside',
                self.boarding_speed,
                distance_override=self.boarding_distance_override,
                check_obstacle=self.forced_drive_check_obstacle
            ):
                return
            
        # 5-1) elevator_inside에 도착하면 즉시 층수 추정 노드 시작
        self.start_elevator_floor_estimation(self.target_floor_signal)

        # 6) elevator_inside -> elevator_btn_inside: 강제 이동
        if not self.force_move_between_waypoints(
            elevator_inside,
            elevator_btn_inside,
            'Move inside elevator: elevator_inside -> elevator_btn_inside',
            self.forced_move_speed,
            check_obstacle=self.forced_drive_check_obstacle
        ):
            return

        # 7) elevator_btn_inside에서 5초 대기
        self.get_logger().info(
            f'Stay at elevator_btn_inside for {self.inside_button_wait_sec:.1f} sec.'
        )
        self.spin_sleep(self.inside_button_wait_sec)


        # 9) elevator_btn_inside -> elevator_inside_for_exit: 강제 이동
        if not self.force_move_between_waypoints(
            elevator_btn_inside,
            elevator_inside_for_exit,
            'Move inside elevator: elevator_btn_inside -> elevator_inside_for_exit',
            self.forced_move_speed,
            check_obstacle=self.forced_drive_check_obstacle
        ):
            return

        # 10) current_floor가 0이 된 뒤에만 B1 문 열림을 인정한다.
        #     중간층에서 문이 열려도 current_floor != 0이면 stable_count를 세지 않는다.
        if not self.wait_until_target_floor_then_door_open(
            self.target_floor_signal,
            f'{self.target_floor_key} elevator door'
        ):
            return

        # 11) B1 map으로 전환
        if not self.load_map(target_info['map_yaml']):
            return

        # 12) 엘리베이터 밖으로 강제 직진 이동 후 B1 elevator_exit pose로 보정
        exit_distance = self.compute_exit_distance(start_info, target_info)

        if not self.drive_straight_simple(
            exit_distance,
            self.exit_speed,
            f'Exit elevator at {self.target_floor_key}: elevator_inside_for_exit -> elevator_exit',
            check_obstacle=self.forced_drive_check_obstacle
        ):
            return

        self.publish_initial_pose(elevator_exit)
        self.clear_costmaps()
        self.spin_sleep(2.0)

        # 13) B1 elevator_exit -> destination: Nav2 이동
        if not self.go_to_pose(destination, f'{self.target_floor_key} destination'):
            return

        self.get_logger().info('Mission complete.')


def main():
    rclpy.init()
    node = ElevatorDeliveryManager2()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()