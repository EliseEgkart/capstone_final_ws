#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
indoor_students_manager.py

Route:
  start -> elevator_btn_front -> unload_spot -> start -> ... 무한 반복

YAML:
maps:
  indoor:
    map_yaml: "indoor_map_final.yaml"
    start: {x: 0.106, y: 0.0254, yaw: -0.0761}
    elevator_btn_front: {x: 5.49, y: -1.29, yaw: 0.0082}
    unload_spot: {x: 4.68, y: -0.104, yaw: -3.1227}

Manipulator:
  elevator_btn_front 도착:
    /manipulator_task_cmd    String(data="INSIDE_BTN_FRONT")
    /manipulator_task_result String(data="INSIDE_BTN_DONE") 대기

  unload_spot 도착:
    /manipulator_task_cmd    String(data="DESTINATION_UNLOAD")
    /manipulator_task_result String(data="UNLOAD_DONE") 대기

Sound:
  최초 start 출발 전: starting_bgm.mp3 -> robot_for_move.mp3
  elevator_btn_front 도착: btn_clk_start.mp3
  unload_spot 도착: destination.mp3
  UNLOAD_DONE 수신: recover.mp3

중요:
  이 노드는 waypoint가 cost 영역에 있어 Nav2 goal이 거부되는 상황을 피하기 위해
  기본 이동을 Nav2가 아니라 cmd_vel open-loop 강제 이동으로 수행한다.
  즉, costmap/장애물 체크 없이 waypoint를 믿고 이동한다.
"""

import math
import os
import time
from typing import Any, Dict, Optional

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from std_msgs.msg import String


PoseDict = Dict[str, Any]


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_distance(a: PoseDict, b: PoseDict) -> float:
    return math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))


def pose_bearing(a: PoseDict, b: PoseDict) -> float:
    return math.atan2(float(b["y"]) - float(a["y"]), float(b["x"]) - float(a["x"]))


class LocalMp3Speaker:
    def __init__(self, node: Node, sound_path: str, enabled: bool = True):
        self.node = node
        self.sound_path = sound_path
        self.enabled = enabled
        self.mixer = None
        self.played_once_keys = set()

        if not self.enabled:
            self.node.get_logger().info("[Speaker] disabled.")
            return

        try:
            from pygame import mixer  # type: ignore

            self.mixer = mixer
            self.mixer.init()
            self.node.get_logger().info(f"[Speaker] sound_path={self.sound_path}")
        except Exception as e:
            self.enabled = False
            self.mixer = None
            self.node.get_logger().error(f"[Speaker] pygame mixer init failed: {e}")

    def _path(self, filename: str) -> str:
        return os.path.join(self.sound_path, filename)

    def is_busy(self) -> bool:
        if not self.enabled or self.mixer is None:
            return False
        try:
            return bool(self.mixer.music.get_busy())
        except Exception:
            return False

    def wait_until_idle(self, timeout_sec: Optional[float] = None) -> bool:
        if not self.enabled:
            return True

        start_time = time.time()
        while rclpy.ok() and self.is_busy():
            if timeout_sec is not None and time.time() - start_time > timeout_sec:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return True

    def play_once(
        self,
        filename: str,
        once_key: Optional[str] = None,
        wait_if_busy: bool = True,
        busy_timeout_sec: Optional[float] = None,
    ) -> bool:
        if once_key is not None and once_key in self.played_once_keys:
            return True

        if not self.enabled or self.mixer is None:
            return False

        if self.is_busy():
            if not wait_if_busy:
                self.node.get_logger().info(f"[Speaker] busy, skip: {filename}")
                return False
            if not self.wait_until_idle(timeout_sec=busy_timeout_sec):
                self.node.get_logger().warn(f"[Speaker] busy timeout, skip: {filename}")
                return False

        path = self._path(filename)
        if not os.path.isfile(path):
            self.node.get_logger().warn(f"[Speaker] sound file not found: {path}")
            return False

        try:
            self.mixer.music.load(path)
            self.mixer.music.play(0)
            if once_key is not None:
                self.played_once_keys.add(once_key)
            self.node.get_logger().info(f"[Speaker] play: {filename}")
            return True
        except Exception as e:
            self.node.get_logger().error(f"[Speaker] play failed: {filename}, error={e}")
            return False

    def shutdown(self):
        if not self.enabled or self.mixer is None:
            return
        try:
            self.mixer.music.stop()
        except Exception:
            pass


class IndoorStudentsManager(Node):
    def __init__(self):
        super().__init__("indoor_students_manager")

        # -------------------------
        # File / map
        # -------------------------
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("map_key", "indoor")
        self.declare_parameter("package_name", "amr_navigator")
        self.declare_parameter("config_dir_name", "config")
        self.declare_parameter("map_dir_name", "map")

        # -------------------------
        # Topics
        # -------------------------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("manipulator_task_cmd_topic", "/manipulator_task_cmd")
        self.declare_parameter("manipulator_task_result_topic", "/manipulator_task_result")
        self.declare_parameter("manipulator_task_state_topic", "/manipulator_task_state")

        # -------------------------
        # Manipulator command/result
        # -------------------------
        self.declare_parameter("inside_button_task_cmd", "INSIDE_BTN_FRONT")
        self.declare_parameter("inside_button_expected_result", "INSIDE_BTN_DONE")
        self.declare_parameter("destination_task_cmd", "DESTINATION_UNLOAD")
        self.declare_parameter("destination_expected_result", "UNLOAD_DONE")
        self.declare_parameter("manipulator_cmd_publish_count", 10)
        self.declare_parameter("manipulator_cmd_publish_interval_sec", 0.2)
        self.declare_parameter("manipulator_cmd_republish_interval_sec", 1.0)
        # 0.0이면 result를 받을 때까지 무한 대기
        self.declare_parameter("manipulator_task_timeout_sec", 0.0)

        # -------------------------
        # Forced movement
        # -------------------------
        self.declare_parameter("linear_speed", 0.18)
        self.declare_parameter("angular_speed", 0.35)
        self.declare_parameter("distance_scale", 1.0)
        self.declare_parameter("initial_pose_publish_count", 20)
        self.declare_parameter("initial_pose_publish_period_sec", 0.1)
        self.declare_parameter("initial_pose_sleep_sec", 0.5)
        self.declare_parameter("arrival_sleep_sec", 0.2)
        self.declare_parameter("publish_initial_pose_at_each_waypoint", True)

        # -------------------------
        # Sound
        # -------------------------
        default_sound_path = self._default_sound_path()
        self.declare_parameter("sound_enabled", True)
        self.declare_parameter("sound_path", default_sound_path)
        self.declare_parameter("starting_bgm_sound", "starting_bgm.mp3")
        self.declare_parameter("robot_for_move_sound", "robot_for_move.mp3")
        self.declare_parameter("btn_clk_start_sound", "btn_clk_start.mp3")
        self.declare_parameter("destination_sound", "destination.mp3")
        self.declare_parameter("recover_sound", "recover.mp3")

        # -------------------------
        # Read params
        # -------------------------
        self.waypoint_file_param = str(self.get_parameter("waypoint_file").value)
        self.map_key = str(self.get_parameter("map_key").value)
        self.package_name = str(self.get_parameter("package_name").value)
        self.config_dir_name = str(self.get_parameter("config_dir_name").value)
        self.map_dir_name = str(self.get_parameter("map_dir_name").value)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.initial_pose_topic = str(self.get_parameter("initial_pose_topic").value)
        self.amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self.manipulator_task_cmd_topic = str(
            self.get_parameter("manipulator_task_cmd_topic").value
        )
        self.manipulator_task_result_topic = str(
            self.get_parameter("manipulator_task_result_topic").value
        )
        self.manipulator_task_state_topic = str(
            self.get_parameter("manipulator_task_state_topic").value
        )

        self.inside_button_task_cmd = str(self.get_parameter("inside_button_task_cmd").value)
        self.inside_button_expected_result = str(
            self.get_parameter("inside_button_expected_result").value
        )
        self.destination_task_cmd = str(self.get_parameter("destination_task_cmd").value)
        self.destination_expected_result = str(
            self.get_parameter("destination_expected_result").value
        )
        self.manipulator_cmd_publish_count = max(
            1, int(self.get_parameter("manipulator_cmd_publish_count").value)
        )
        self.manipulator_cmd_publish_interval_sec = max(
            0.01, float(self.get_parameter("manipulator_cmd_publish_interval_sec").value)
        )
        self.manipulator_cmd_republish_interval_sec = max(
            0.1, float(self.get_parameter("manipulator_cmd_republish_interval_sec").value)
        )
        self.manipulator_task_timeout_sec = max(
            0.0, float(self.get_parameter("manipulator_task_timeout_sec").value)
        )

        self.linear_speed = abs(float(self.get_parameter("linear_speed").value))
        self.angular_speed = abs(float(self.get_parameter("angular_speed").value))
        self.distance_scale = max(0.01, float(self.get_parameter("distance_scale").value))
        self.initial_pose_publish_count = max(
            1, int(self.get_parameter("initial_pose_publish_count").value)
        )
        self.initial_pose_publish_period_sec = max(
            0.01, float(self.get_parameter("initial_pose_publish_period_sec").value)
        )
        self.initial_pose_sleep_sec = max(
            0.0, float(self.get_parameter("initial_pose_sleep_sec").value)
        )
        self.arrival_sleep_sec = max(0.0, float(self.get_parameter("arrival_sleep_sec").value))
        self.publish_initial_pose_at_each_waypoint = bool(
            self.get_parameter("publish_initial_pose_at_each_waypoint").value
        )

        self.sound_enabled = bool(self.get_parameter("sound_enabled").value)
        self.sound_path = str(self.get_parameter("sound_path").value)
        self.starting_bgm_sound = str(self.get_parameter("starting_bgm_sound").value)
        self.robot_for_move_sound = str(self.get_parameter("robot_for_move_sound").value)
        self.btn_clk_start_sound = str(self.get_parameter("btn_clk_start_sound").value)
        self.destination_sound = str(self.get_parameter("destination_sound").value)
        self.recover_sound = str(self.get_parameter("recover_sound").value)

        if self.linear_speed <= 0.0:
            self.linear_speed = 0.18
        if self.angular_speed <= 0.0:
            self.angular_speed = 0.35

        # -------------------------
        # Runtime state
        # -------------------------
        self.current_pose = None
        self.latest_manipulator_result: Optional[str] = None
        self.latest_manipulator_state: Optional[str] = None

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.amcl_pose_callback,
            10,
        )
        self.create_subscription(
            String,
            self.manipulator_task_result_topic,
            self.manipulator_task_result_callback,
            10,
        )
        self.create_subscription(
            String,
            self.manipulator_task_state_topic,
            self.manipulator_task_state_callback,
            10,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            10,
        )

        manipulator_qos = QoSProfile(depth=10)
        manipulator_qos.reliability = ReliabilityPolicy.RELIABLE
        manipulator_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.manipulator_task_cmd_pub = self.create_publisher(
            String,
            self.manipulator_task_cmd_topic,
            manipulator_qos,
        )

        self.load_map_client = self.create_client(LoadMap, "/map_server/load_map")
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )

        self.speaker = LocalMp3Speaker(
            node=self,
            sound_path=self.sound_path,
            enabled=self.sound_enabled,
        )

        # -------------------------
        # Load waypoint YAML
        # -------------------------
        self.waypoint_file = self.resolve_waypoint_file(self.waypoint_file_param)
        with open(self.waypoint_file, "r", encoding="utf-8") as f:
            self.wp = yaml.safe_load(f)

        self.get_logger().info(
            "IndoorStudentsManager ready. "
            f"waypoint_file={self.waypoint_file}, "
            f"map_key={self.map_key}, "
            f"cmd_vel_topic={self.cmd_vel_topic}, "
            f"initial_pose_topic={self.initial_pose_topic}, "
            f"task_cmd_topic={self.manipulator_task_cmd_topic}, "
            f"task_result_topic={self.manipulator_task_result_topic}, "
            f"linear_speed={self.linear_speed:.3f}, "
            f"angular_speed={self.angular_speed:.3f}, "
            f"distance_scale={self.distance_scale:.3f}"
        )

    # ------------------------------------------------------------------
    # Basic helpers
    # ------------------------------------------------------------------
    def _default_sound_path(self) -> str:
        try:
            return os.path.join(get_package_share_directory("tts_speaker"), "sounds")
        except PackageNotFoundError:
            return os.path.join(os.getcwd(), "sounds")

    def resolve_waypoint_file(self, waypoint_file_param: str) -> str:
        if waypoint_file_param:
            return waypoint_file_param

        config_dir = os.path.join(
            get_package_share_directory(self.package_name),
            self.config_dir_name,
        )
        return os.path.join(config_dir, "waypoints_indoor.yaml")

    def resolve_map_yaml_path(self, map_yaml: str) -> str:
        if os.path.isabs(map_yaml):
            return map_yaml

        return os.path.join(
            get_package_share_directory(self.package_name),
            self.map_dir_name,
            map_yaml,
        )

    def spin_sleep(self, duration_sec: float):
        end_time = time.time() + float(duration_sec)
        while rclpy.ok() and time.time() < end_time:
            remain = max(0.0, end_time - time.time())
            rclpy.spin_once(self, timeout_sec=min(0.1, remain))

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def manipulator_task_result_callback(self, msg: String):
        self.latest_manipulator_result = str(msg.data).strip()
        self.get_logger().info(
            f"Received manipulator result: data='{self.latest_manipulator_result}'"
        )

    def manipulator_task_state_callback(self, msg: String):
        self.latest_manipulator_state = str(msg.data).strip()
        self.get_logger().info(
            f"Received manipulator state: data='{self.latest_manipulator_state}'"
        )

    # ------------------------------------------------------------------
    # YAML / map
    # ------------------------------------------------------------------
    def get_indoor_info(self) -> Dict[str, Any]:
        maps = self.wp.get("maps", {})
        if self.map_key not in maps:
            raise KeyError(
                f"waypoints YAML에 maps.{self.map_key}가 없습니다. 현재 maps 키={list(maps.keys())}"
            )
        return maps[self.map_key]

    def require_pose(self, map_info: Dict[str, Any], pose_key: str) -> PoseDict:
        if pose_key not in map_info:
            raise KeyError(f"waypoints YAML에 '{pose_key}' waypoint가 없습니다.")

        pose = map_info[pose_key]
        for field in ("x", "y", "yaw"):
            if field not in pose:
                raise KeyError(f"waypoints YAML의 {pose_key}.{field} 값이 없습니다.")
        return pose

    def load_map(self, map_yaml: str) -> bool:
        map_path = self.resolve_map_yaml_path(map_yaml)
        self.get_logger().info(f"Load map: {map_path}")

        while rclpy.ok() and not self.load_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /map_server/load_map service...")

        req = LoadMap.Request()
        req.map_url = map_path
        future = self.load_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result is None:
            self.get_logger().error("LoadMap returned None.")
            return False

        self.get_logger().info(f"LoadMap result={result.result}")
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

    def publish_initial_pose(self, pose_dict: PoseDict):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(pose_dict["x"])
        msg.pose.pose.position.y = float(pose_dict["y"])
        msg.pose.pose.orientation = yaw_to_quaternion(float(pose_dict["yaw"]))

        # AMCL 초기 위치 공분산
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        self.get_logger().info(
            "Publish initial pose: "
            f"x={float(pose_dict['x']):.3f}, "
            f"y={float(pose_dict['y']):.3f}, "
            f"yaw={float(pose_dict['yaw']):.4f}"
        )

        for _ in range(self.initial_pose_publish_count):
            self.initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=self.initial_pose_publish_period_sec)

        if self.initial_pose_sleep_sec > 0.0:
            self.spin_sleep(self.initial_pose_sleep_sec)

    # ------------------------------------------------------------------
    # Forced movement
    # ------------------------------------------------------------------
    def stop_robot(self, repeat: int = 10):
        msg = Twist()
        for _ in range(repeat):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def rotate_relative_simple(self, angle_rad: float, label: str) -> bool:
        angle = normalize_angle(angle_rad)
        if abs(angle) < 1.0e-3:
            self.get_logger().info(f"{label}: rotate skip.")
            return True

        direction = 1.0 if angle >= 0.0 else -1.0
        angular_z = self.angular_speed * direction
        duration = abs(angle) / self.angular_speed

        self.get_logger().info(
            f"{label}: rotate angle={angle:.4f}rad, angular_z={angular_z:.3f}, "
            f"duration={duration:.2f}s"
        )

        msg = Twist()
        msg.angular.z = angular_z
        start_time = time.time()

        while rclpy.ok() and time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        return True

    def drive_straight_simple(self, distance_m: float, label: str) -> bool:
        distance = float(distance_m) * self.distance_scale
        if abs(distance) < 1.0e-3:
            self.get_logger().info(f"{label}: drive skip.")
            return True

        direction = 1.0 if distance >= 0.0 else -1.0
        linear_x = self.linear_speed * direction
        duration = abs(distance) / self.linear_speed

        self.get_logger().info(
            f"{label}: drive distance={distance:.3f}m, linear_x={linear_x:.3f}, "
            f"duration={duration:.2f}s"
        )

        msg = Twist()
        msg.linear.x = linear_x
        start_time = time.time()

        while rclpy.ok() and time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        return True

    def force_move_between_waypoints(
        self,
        from_pose: PoseDict,
        to_pose: PoseDict,
        label: str,
    ) -> bool:
        """
        costmap/장애물 체크 없이 waypoint를 믿고 강제 이동한다.
        1) 현재 waypoint yaw 기준으로 목표 waypoint 방향까지 회전
        2) 두 waypoint 사이 거리만큼 직진
        3) 목표 waypoint yaw로 회전
        4) AMCL pose를 목표 waypoint로 보정
        """
        from_yaw = float(from_pose["yaw"])
        to_yaw = float(to_pose["yaw"])
        bearing = pose_bearing(from_pose, to_pose)
        distance = pose_distance(from_pose, to_pose)

        first_turn = normalize_angle(bearing - from_yaw)
        final_turn = normalize_angle(to_yaw - bearing)

        self.get_logger().info(
            f"{label}: force move. "
            f"from=({float(from_pose['x']):.3f}, {float(from_pose['y']):.3f}, {from_yaw:.4f}), "
            f"to=({float(to_pose['x']):.3f}, {float(to_pose['y']):.3f}, {to_yaw:.4f}), "
            f"bearing={bearing:.4f}, distance={distance:.3f}, "
            f"first_turn={first_turn:.4f}, final_turn={final_turn:.4f}"
        )

        if not self.rotate_relative_simple(first_turn, f"{label} first_turn"):
            return False

        if not self.drive_straight_simple(distance, f"{label} straight"):
            return False

        if not self.rotate_relative_simple(final_turn, f"{label} final_turn"):
            return False

        if self.publish_initial_pose_at_each_waypoint:
            self.publish_initial_pose(to_pose)
            self.clear_costmaps()

        if self.arrival_sleep_sec > 0.0:
            self.spin_sleep(self.arrival_sleep_sec)

        self.get_logger().info(f"{label}: arrived.")
        return True

    # ------------------------------------------------------------------
    # Manipulator handshake
    # ------------------------------------------------------------------
    def publish_task_command(self, task_cmd: str, repeat: int = 1, interval_sec: float = 0.0):
        msg = String()
        msg.data = str(task_cmd)

        repeat = max(1, int(repeat))
        for idx in range(repeat):
            self.manipulator_task_cmd_pub.publish(msg)
            self.get_logger().info(
                f"Publish {self.manipulator_task_cmd_topic}: "
                f"data='{msg.data}' ({idx + 1}/{repeat})"
            )
            if idx < repeat - 1 and interval_sec > 0.0:
                self.spin_sleep(interval_sec)

    def is_task_result_match(
        self,
        received: Optional[str],
        expected_result: str,
        task_cmd: str,
    ) -> bool:
        if received is None:
            return False

        data = str(received).strip()
        expected = str(expected_result).strip()
        cmd = str(task_cmd).strip()

        if not data:
            return False
        if data == expected:
            return True

        data_l = data.lower()
        expected_l = expected.lower()
        cmd_l = cmd.lower()

        # 혹시 로봇팔 쪽에서 done/success 형태로 주는 경우까지 방어적으로 허용
        accepted = {
            expected_l,
            f"{cmd_l}_done",
            f"{cmd_l}:done",
            f"{cmd_l}:success",
            f"done:{cmd_l}",
            f"success:{cmd_l}",
            "done",
            "success",
            "completed",
            "true",
            "1",
        }
        return data_l in accepted

    def send_manipulator_task_and_wait(self, task_cmd: str, expected_result: str) -> bool:
        self.latest_manipulator_result = None

        self.get_logger().info(
            "Manipulator handshake start: "
            f"publish String(data='{task_cmd}') to {self.manipulator_task_cmd_topic}, "
            f"wait String(data='{expected_result}') from {self.manipulator_task_result_topic}"
        )

        # 이전 transient_local command 값이 남는 것을 줄이기 위해 빈 문자열을 먼저 발행
        self.publish_task_command("", repeat=2, interval_sec=0.05)
        self.spin_sleep(0.1)

        self.publish_task_command(
            task_cmd,
            repeat=self.manipulator_cmd_publish_count,
            interval_sec=self.manipulator_cmd_publish_interval_sec,
        )

        start_time = time.time()
        last_republish_time = time.time()
        last_log_time = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()

            if self.is_task_result_match(
                self.latest_manipulator_result,
                expected_result,
                task_cmd,
            ):
                self.get_logger().info(
                    f"Manipulator task done. received='{self.latest_manipulator_result}'"
                )
                self.publish_task_command("", repeat=3, interval_sec=0.05)
                return True

            if (
                self.manipulator_task_timeout_sec > 0.0
                and now - start_time > self.manipulator_task_timeout_sec
            ):
                self.get_logger().error(
                    "Timeout waiting manipulator task result. "
                    f"expected='{expected_result}', "
                    f"latest='{self.latest_manipulator_result}', "
                    f"timeout={self.manipulator_task_timeout_sec:.1f}s"
                )
                self.publish_task_command("", repeat=3, interval_sec=0.05)
                return False

            # 로봇팔 subscriber 연결 타이밍 문제를 줄이기 위해 완료될 때까지 주기적으로 재발행
            if now - last_republish_time >= self.manipulator_cmd_republish_interval_sec:
                self.publish_task_command(task_cmd, repeat=1, interval_sec=0.0)
                last_republish_time = now

            if now - last_log_time >= 1.0:
                elapsed = now - start_time
                self.get_logger().info(
                    "Waiting manipulator result: "
                    f"expected='{expected_result}', "
                    f"latest_result='{self.latest_manipulator_result}', "
                    f"latest_state='{self.latest_manipulator_state}', "
                    f"elapsed={elapsed:.1f}s"
                )
                last_log_time = now

        return False

    # ------------------------------------------------------------------
    # Main mission
    # ------------------------------------------------------------------
    def run(self):
        try:
            indoor_info = self.get_indoor_info()
            if "map_yaml" not in indoor_info:
                raise KeyError("waypoints YAML의 maps.indoor.map_yaml 값이 없습니다.")

            map_yaml = str(indoor_info["map_yaml"])
            start_pose = self.require_pose(indoor_info, "start")
            elevator_btn_front_pose = self.require_pose(indoor_info, "elevator_btn_front")
            unload_spot_pose = self.require_pose(indoor_info, "unload_spot")
        except Exception as e:
            self.get_logger().error(str(e))
            return

        if not self.load_map(map_yaml):
            return

        # 시작 위치를 start로 설정
        self.publish_initial_pose(start_pose)
        self.clear_costmaps()
        self.spin_sleep(1.0)

        # 처음 start에서 출발할 때만 1회씩 재생
        self.speaker.play_once(
            self.starting_bgm_sound,
            once_key="mission_starting_bgm_once",
            wait_if_busy=True,
        )
        self.speaker.play_once(
            self.robot_for_move_sound,
            once_key="mission_robot_for_move_once",
            wait_if_busy=True,
        )

        current_pose = start_pose
        current_name = "start"
        cycle = 1

        self.get_logger().info(
            "Indoor mission loop started. Ctrl+C로 종료. "
            "Route: start -> elevator_btn_front -> unload_spot -> start -> ..."
        )

        while rclpy.ok():
            self.get_logger().info(f"========== cycle {cycle} start ==========")

            # 1) start -> elevator_btn_front
            if not self.force_move_between_waypoints(
                current_pose,
                elevator_btn_front_pose,
                f"cycle {cycle}: {current_name} -> elevator_btn_front",
            ):
                self.get_logger().error("Failed to move to elevator_btn_front.")
                break

            current_pose = elevator_btn_front_pose
            current_name = "elevator_btn_front"

            # elevator_btn_front 도착음
            self.speaker.play_once(
                self.btn_clk_start_sound,
                once_key=f"cycle_{cycle}_btn_clk_start",
                wait_if_busy=True,
            )

            # 로봇팔 내부 버튼 동작 요청
            if not self.send_manipulator_task_and_wait(
                task_cmd=self.inside_button_task_cmd,
                expected_result=self.inside_button_expected_result,
            ):
                self.get_logger().error("Failed while waiting INSIDE_BTN_DONE.")
                break

            # 2) elevator_btn_front -> unload_spot
            if not self.force_move_between_waypoints(
                current_pose,
                unload_spot_pose,
                f"cycle {cycle}: elevator_btn_front -> unload_spot",
            ):
                self.get_logger().error("Failed to move to unload_spot.")
                break

            current_pose = unload_spot_pose
            current_name = "unload_spot"

            # unload_spot 도착음
            self.speaker.play_once(
                self.destination_sound,
                once_key=f"cycle_{cycle}_destination",
                wait_if_busy=True,
            )

            # 로봇팔 하역 동작 요청
            if not self.send_manipulator_task_and_wait(
                task_cmd=self.destination_task_cmd,
                expected_result=self.destination_expected_result,
            ):
                self.get_logger().error("Failed while waiting UNLOAD_DONE.")
                break

            # UNLOAD_DONE 수신 후 복귀 안내음
            self.speaker.play_once(
                self.recover_sound,
                once_key=f"cycle_{cycle}_recover",
                wait_if_busy=True,
            )

            # 3) unload_spot -> start
            if not self.force_move_between_waypoints(
                current_pose,
                start_pose,
                f"cycle {cycle}: unload_spot -> start",
            ):
                self.get_logger().error("Failed to return to start.")
                break

            current_pose = start_pose
            current_name = "start"

            self.get_logger().info(f"========== cycle {cycle} complete ==========")
            cycle += 1

        self.get_logger().info("Indoor mission loop ended.")


def main(args=None):
    rclpy.init(args=args)
    node = IndoorStudentsManager()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Stop indoor mission.")
    finally:
        node.stop_robot(repeat=10)
        node.speaker.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()