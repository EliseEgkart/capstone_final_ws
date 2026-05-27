#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
elevator_manager_final_with_manipulator.py

3F room_front -> elevator_btn_front -> elevator_front -> elevator_inside
-> elevator_btn_inside -> elevator_inside_for_exit -> B1 elevator_exit -> destination

메니퓰레이터 task manager 연동 반영 사항
AMR -> Manipulator:
  /manipulator_task_cmd      std_msgs/msg/String

Manipulator -> AMR:
  /manipulator_task_result   std_msgs/msg/String
  /manipulator_task_state    std_msgs/msg/String

작업 명령 문자열 기본값
1) elevator_btn_front 도착 시 /manipulator_task_cmd 로
   data: "OUTSIDE_BTN_FRONT" 발행
2) /manipulator_task_result 에서
   data: "OUTSIDE_BTN_DONE" 를 받으면 elevator_front로 다음 이동
3) elevator_btn_inside 도착 시 /manipulator_task_cmd 로
   data: "INSIDE_BTN_FRONT" 발행
4) /manipulator_task_result 에서
   data: "INSIDE_BTN_DONE" 를 받으면 elevator_inside_for_exit로 다음 이동
5) B1 destination 도착 시 /manipulator_task_cmd 로
   data: "DESTINATION_UNLOAD" 발행
   - 현재 요구사항에서는 destination_goal에 대한 완료 응답은 기다리지 않는다.

기존 유지 사항
- RealSense 카메라 노드(realsense2_camera)는 이 파일에서 실행하지 않는다.
- YOLO 관련 함수/파라미터는 호환성을 위해 남겨두었지만,
  메인 미션 흐름에서는 버튼 active 인식 대기 대신 메니퓰레이터 task result를 사용한다.
- 주요 이동 구간마다 robot_for_move.mp3 1회 재생
- room_front 출발 시 starting_bgm.mp3 1회 재생
- destination 도착 시 destination.mp3 1회, 5초 뒤 give_snack.mp3 1회 재생
"""

import math
import os
import re
import shlex
import signal
import subprocess
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String

try:
    from amr_msgs.msg import DetectionArray
except ImportError:
    # 일부 작업 환경에서 동일 인터페이스 패키지명이 interfaces_pkg로 들어온 경우를 위한 fallback
    from interfaces_pkg.msg import DetectionArray


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
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_distance(a: PoseDict, b: PoseDict) -> float:
    dx = float(b["x"]) - float(a["x"])
    dy = float(b["y"]) - float(a["y"])
    return math.hypot(dx, dy)


def pose_bearing(a: PoseDict, b: PoseDict) -> float:
    dx = float(b["x"]) - float(a["x"])
    dy = float(b["y"]) - float(a["y"])
    return math.atan2(dy, dx)


def normalize_label(text: str) -> str:
    """YOLO class_name 비교용 정규화. btn_down, btn-down, BtnDown을 비슷하게 처리한다."""
    return "".join(ch for ch in str(text).lower() if ch.isalnum())


class LocalMp3Speaker:
    """
    elevator_delivery_final.py 내부에서 직접 mp3를 재생한다.

    tts_speaker의 pygame 의존성을 그대로 활용하는 방식이다.
    핵심 정책은 다음과 같다.
    - 이미 재생 중이면 일반 안내음은 새로 시작하지 않는다.
    - 반드시 한 번 나가야 하는 안내음은 현재 재생이 끝날 때까지 기다린 뒤 1회 재생한다.
    - btn_not_perception 안내는 occlusion 상태가 유지될 때만 다음 회차를 재생한다.
      버튼이 다시 보이면 추가 반복을 멈추지만, 이미 나온 음성은 중간에 끊지 않는다.
    """

    def __init__(self, node: Node, sound_path: str, enabled: bool = True):
        self.node = node
        self.sound_path = sound_path
        self.enabled = enabled
        self.played_once_keys = set()
        self.last_alarm_play_time: Dict[str, float] = {}
        self.mixer = None

        if not self.enabled:
            self.node.get_logger().info("[Speaker] Local MP3 speaker disabled.")
            return

        try:
            from pygame import mixer  # type: ignore

            self.mixer = mixer
            self.mixer.init()
            self.node.get_logger().info(f"[Speaker] Using sound path: {self.sound_path}")
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

        start = time.time()
        while rclpy.ok() and self.is_busy():
            if timeout_sec is not None and (time.time() - start) > timeout_sec:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return True

    def play_once(
        self,
        filename: str,
        once_key: Optional[str] = None,
        wait_if_busy: bool = False,
        busy_timeout_sec: Optional[float] = None,
    ) -> bool:
        """
        filename을 1회 재생한다.
        once_key가 있으면 같은 key는 전체 미션 동안 한 번만 재생한다.
        """
        if once_key is not None and once_key in self.played_once_keys:
            return True

        if not self.enabled or self.mixer is None:
            return False

        if self.is_busy():
            if not wait_if_busy:
                self.node.get_logger().info(
                    f"[Speaker] Busy. Skip sound without interrupting: {filename}"
                )
                return False
            if not self.wait_until_idle(timeout_sec=busy_timeout_sec):
                self.node.get_logger().warn(
                    f"[Speaker] Busy timeout. Sound not played: {filename}"
                )
                return False

        path = self._path(filename)
        if not os.path.isfile(path):
            self.node.get_logger().warn(f"[Speaker] Sound file not found: {path}")
            return False

        try:
            self.mixer.music.load(path)
            self.mixer.music.play(0)
            if once_key is not None:
                self.played_once_keys.add(once_key)
            self.node.get_logger().info(f"[Speaker] Play once: {filename}")
            return True
        except Exception as e:
            self.node.get_logger().error(f"[Speaker] Play failed: {filename}, error={e}")
            return False

    def play_alarm_if_idle(
        self,
        filename: str,
        alarm_key: str,
        min_interval_sec: float = 0.5,
    ) -> bool:
        """
        반복 안내용. 현재 재생 중이면 아무것도 하지 않는다.
        재생이 끝났고 occlusion 상태가 유지될 때 wait loop가 다시 호출하면 다음 회차를 재생한다.
        """
        if not self.enabled or self.mixer is None:
            return False

        if self.is_busy():
            return False

        now = time.time()
        last = self.last_alarm_play_time.get(alarm_key, 0.0)
        if now - last < min_interval_sec:
            return False

        path = self._path(filename)
        if not os.path.isfile(path):
            self.node.get_logger().warn(f"[Speaker] Alarm file not found: {path}")
            self.last_alarm_play_time[alarm_key] = now
            return False

        try:
            self.mixer.music.load(path)
            self.mixer.music.play(0)
            self.last_alarm_play_time[alarm_key] = now
            self.node.get_logger().info(f"[Speaker] Play alarm: {filename}")
            return True
        except Exception as e:
            self.node.get_logger().error(f"[Speaker] Alarm play failed: {filename}, error={e}")
            return False

    def shutdown(self):
        if not self.enabled or self.mixer is None:
            return
        try:
            self.mixer.music.stop()
        except Exception:
            pass


class ElevatorDeliveryFinalWithManipulator(Node):
    """
    엘리베이터 버튼 인식 + waypoint/Nav2 + open-loop forced move 통합 미션 노드.
    """

    def __init__(self):
        super().__init__("elevator_delivery_final_with_manipulator")

        # ------------------------------------------------------------
        # Basic parameters
        # ------------------------------------------------------------
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("start_floor", 3)
        self.declare_parameter("target_floor_key", "B1")
        self.declare_parameter("target_floor_signal", 0)
        self.declare_parameter("scan_topic", "/rplidar1/scan_filtered")
        self.declare_parameter("current_floor_topic", "/current_floor")
        self.declare_parameter("elevator_start_topic", "/elevator/start")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")

        # ------------------------------------------------------------
        # Manipulator task manager parameters
        # ------------------------------------------------------------
        # AMR -> Manipulator task manager
        self.declare_parameter("manipulator_task_cmd_topic", "/manipulator_task_cmd")
        # Manipulator task manager -> AMR
        self.declare_parameter("manipulator_task_result_topic", "/manipulator_task_result")
        self.declare_parameter("manipulator_task_state_topic", "/manipulator_task_state")

        # AMR이 task manager로 보낼 작업 명령 문자열
        self.declare_parameter("outside_button_task_cmd", "OUTSIDE_BTN_FRONT")
        self.declare_parameter("inside_button_task_cmd", "INSIDE_BTN_FRONT")
        self.declare_parameter("destination_task_cmd", "DESTINATION_UNLOAD")

        # task manager가 완료 시 /manipulator_task_result로 보내야 하는 결과 문자열
        self.declare_parameter("outside_button_expected_result", "OUTSIDE_BTN_DONE")
        self.declare_parameter("inside_button_expected_result", "INSIDE_BTN_DONE")
        self.declare_parameter("destination_expected_result", "UNLOAD_DONE")

        # task command 발행 정책
        self.declare_parameter("manipulator_cmd_publish_count", 10)
        self.declare_parameter("manipulator_cmd_publish_interval_sec", 0.2)
        self.declare_parameter("manipulator_cmd_republish_interval_sec", 1.0)
        # 0.0이면 result를 받을 때까지 무한 대기
        self.declare_parameter("manipulator_task_timeout_sec", 0.0)

        # ------------------------------------------------------------
        # Door detection parameters
        # ------------------------------------------------------------
        self.declare_parameter("door_max_valid_range", 10.0)
        self.declare_parameter("door_min_valid_count", 8)
        self.declare_parameter("treat_max_range_as_open", False)
        self.declare_parameter("door_center_deg", 180.0)
        self.declare_parameter("door_half_width_deg", 8.0)
        self.declare_parameter("door_open_distance", 1.30)
        self.declare_parameter("door_min_open_ratio", 0.70)
        self.declare_parameter("door_stable_count_required", 20)
        self.declare_parameter("already_inside_radius", 0.60)

        # ------------------------------------------------------------
        # Direct / forced drive parameters
        # ------------------------------------------------------------
        self.declare_parameter("boarding_speed", 0.18)
        self.declare_parameter("forced_move_speed", 0.16)
        self.declare_parameter("exit_speed", 0.18)
        self.declare_parameter("rotate_speed", 0.35)
        self.declare_parameter("front_approach_speed", 0.18)
        self.declare_parameter("front_approach_distance_override", 0.0)
        self.declare_parameter("boarding_distance_override", 0.0)
        self.declare_parameter("exit_distance_override", 0.0)
        self.declare_parameter("forced_drive_check_obstacle", False)
        self.declare_parameter("direct_drive_stop_distance", 0.40)
        self.declare_parameter("publish_initial_pose_before_forced_move", True)
        self.declare_parameter("publish_initial_pose_after_forced_move", True)
        self.declare_parameter("initial_pose_sleep_sec", 0.5)

        # ------------------------------------------------------------
        # YOLO / RealSense perception parameters
        # ------------------------------------------------------------
        self.declare_parameter("camera_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("yolov8_package", "camera_perception_pkg")
        self.declare_parameter("yolov8_executable", "yolov8_node")
        self.declare_parameter("yolov8_command", "")
        self.declare_parameter("yolov8_startup_wait_sec", 2.0)
        self.declare_parameter("yolov8_stop_timeout_sec", 3.0)
        self.declare_parameter("button_detection_topics", ["/detections/best", "/detections"])
        self.declare_parameter("person_detection_topics", ["/detections/best_person", "/detections"])
        self.declare_parameter("detection_stale_timeout_sec", 1.0)
        self.declare_parameter("button_score_threshold", 0.35)
        self.declare_parameter("person_score_threshold", 0.35)
        # 버튼 active 확정 조건: active 상태가 이 시간 이상 연속 유지되어야 확정한다.
        self.declare_parameter("button_active_confirm_sec", 2.0)
        # 구버전 호환용. 시간 기반 확정 방식에서는 사용하지 않는다.
        self.declare_parameter("button_required_stable_count", 1)
        # target labels에는 active/deactive 접미사를 빼고 버튼의 기본 이름만 넣는다.
        # 실제 YOLO class_name이 btn_down_active / btn_down_deactive처럼 들어오면
        # manager가 상태 접미사를 분리해서 active만 통과시킨다.
        self.declare_parameter("front_button_active_labels", ["btn_down"])
        self.declare_parameter("inside_button_active_labels", ["elevator_btn_under1"])
        self.declare_parameter("person_labels", ["person"])
        self.declare_parameter("allow_button_state_unknown_as_active", False)

        # ------------------------------------------------------------
        # Sound parameters
        # ------------------------------------------------------------
        default_sound_path = self._default_sound_path()
        self.declare_parameter("sound_enabled", True)
        self.declare_parameter("sound_path", default_sound_path)
        self.declare_parameter("btn_not_perception_sound", "btn_not_perception.mp3")
        self.declare_parameter("destination_sound", "destination.mp3")
        self.declare_parameter("give_snack_sound", "give_snack.mp3")
        self.declare_parameter("robot_for_move_sound", "robot_for_move.mp3")
        self.declare_parameter("starting_bgm_sound", "starting_bgm.mp3")
        self.declare_parameter("give_snack_delay_sec", 5.0)
        self.declare_parameter("alarm_repeat_min_interval_sec", 0.5)

        # ------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------
        waypoint_file_param = str(self.get_parameter("waypoint_file").value)
        self.start_floor = int(self.get_parameter("start_floor").value)
        self.target_floor_key = str(self.get_parameter("target_floor_key").value)
        self.target_floor_signal = int(self.get_parameter("target_floor_signal").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.current_floor_topic = str(self.get_parameter("current_floor_topic").value)
        self.elevator_start_topic = str(self.get_parameter("elevator_start_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self.manipulator_task_cmd_topic = str(
            self.get_parameter("manipulator_task_cmd_topic").value
        )
        self.manipulator_task_result_topic = str(
            self.get_parameter("manipulator_task_result_topic").value
        )
        self.manipulator_task_state_topic = str(
            self.get_parameter("manipulator_task_state_topic").value
        )
        self.outside_button_task_cmd = str(self.get_parameter("outside_button_task_cmd").value)
        self.inside_button_task_cmd = str(self.get_parameter("inside_button_task_cmd").value)
        self.destination_task_cmd = str(self.get_parameter("destination_task_cmd").value)
        self.outside_button_expected_result = str(
            self.get_parameter("outside_button_expected_result").value
        )
        self.inside_button_expected_result = str(
            self.get_parameter("inside_button_expected_result").value
        )
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

        self.door_max_valid_range = float(self.get_parameter("door_max_valid_range").value)
        self.door_min_valid_count = int(self.get_parameter("door_min_valid_count").value)
        self.treat_max_range_as_open = bool(self.get_parameter("treat_max_range_as_open").value)
        self.door_center_deg = float(self.get_parameter("door_center_deg").value)
        self.door_half_width_deg = float(self.get_parameter("door_half_width_deg").value)
        self.door_open_distance = float(self.get_parameter("door_open_distance").value)
        self.door_min_open_ratio = float(self.get_parameter("door_min_open_ratio").value)
        self.door_stable_count_required = int(self.get_parameter("door_stable_count_required").value)
        self.already_inside_radius = float(self.get_parameter("already_inside_radius").value)

        self.boarding_speed = float(self.get_parameter("boarding_speed").value)
        self.forced_move_speed = float(self.get_parameter("forced_move_speed").value)
        self.exit_speed = float(self.get_parameter("exit_speed").value)
        self.rotate_speed = float(self.get_parameter("rotate_speed").value)
        self.front_approach_speed = float(self.get_parameter("front_approach_speed").value)
        self.front_approach_distance_override = float(
            self.get_parameter("front_approach_distance_override").value
        )
        self.boarding_distance_override = float(self.get_parameter("boarding_distance_override").value)
        self.exit_distance_override = float(self.get_parameter("exit_distance_override").value)
        self.forced_drive_check_obstacle = bool(self.get_parameter("forced_drive_check_obstacle").value)
        self.direct_drive_stop_distance = float(self.get_parameter("direct_drive_stop_distance").value)
        self.publish_initial_pose_before_forced_move = bool(
            self.get_parameter("publish_initial_pose_before_forced_move").value
        )
        self.publish_initial_pose_after_forced_move = bool(
            self.get_parameter("publish_initial_pose_after_forced_move").value
        )
        self.initial_pose_sleep_sec = float(self.get_parameter("initial_pose_sleep_sec").value)

        self.camera_image_topic = str(self.get_parameter("camera_image_topic").value)
        self.yolov8_package = str(self.get_parameter("yolov8_package").value)
        self.yolov8_executable = str(self.get_parameter("yolov8_executable").value)
        self.yolov8_command = str(self.get_parameter("yolov8_command").value).strip()
        self.yolov8_startup_wait_sec = float(self.get_parameter("yolov8_startup_wait_sec").value)
        self.yolov8_stop_timeout_sec = float(self.get_parameter("yolov8_stop_timeout_sec").value)
        self.button_detection_topics = self._get_string_list_param("button_detection_topics")
        self.person_detection_topics = self._get_string_list_param("person_detection_topics")
        self.detection_stale_timeout_sec = float(
            self.get_parameter("detection_stale_timeout_sec").value
        )
        self.button_score_threshold = float(self.get_parameter("button_score_threshold").value)
        self.person_score_threshold = float(self.get_parameter("person_score_threshold").value)
        self.button_active_confirm_sec = max(
            0.0, float(self.get_parameter("button_active_confirm_sec").value)
        )
        # 구버전 파라미터가 YAML에 남아 있어도 노드가 깨지지 않도록 읽기만 한다.
        self.button_required_stable_count = max(
            1, int(self.get_parameter("button_required_stable_count").value)
        )
        self.front_button_active_labels = self._get_string_list_param("front_button_active_labels")
        self.inside_button_active_labels = self._get_string_list_param("inside_button_active_labels")
        self.person_labels = self._get_string_list_param("person_labels")
        self.allow_button_state_unknown_as_active = bool(
            self.get_parameter("allow_button_state_unknown_as_active").value
        )

        self.sound_enabled = bool(self.get_parameter("sound_enabled").value)
        self.sound_path = str(self.get_parameter("sound_path").value)
        self.btn_not_perception_sound = str(self.get_parameter("btn_not_perception_sound").value)
        self.destination_sound = str(self.get_parameter("destination_sound").value)
        self.give_snack_sound = str(self.get_parameter("give_snack_sound").value)
        self.robot_for_move_sound = str(self.get_parameter("robot_for_move_sound").value)
        self.starting_bgm_sound = str(self.get_parameter("starting_bgm_sound").value)
        self.give_snack_delay_sec = float(self.get_parameter("give_snack_delay_sec").value)
        self.alarm_repeat_min_interval_sec = float(
            self.get_parameter("alarm_repeat_min_interval_sec").value
        )

        self.waypoint_file = self.resolve_waypoint_file(waypoint_file_param)
        with open(self.waypoint_file, "r", encoding="utf-8") as f:
            self.wp = yaml.safe_load(f)

        # ------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------
        self.scan: Optional[LaserScan] = None
        self.current_floor: Optional[int] = None
        self.current_pose = None
        self.yolov8_process: Optional[subprocess.Popen] = None
        self.latest_detections: Dict[str, Tuple[float, List[Any]]] = {}

        # Manipulator task manager state
        self.latest_manipulator_result: Optional[str] = None
        self.latest_manipulator_state: Optional[str] = None
        self.current_manipulator_task_cmd: Optional[str] = None
        self.current_manipulator_expected_result: Optional[str] = None

        # 버튼 active latch. 한 번 2초 이상 active로 확정되면 목표 층 도착 전까지 유지한다.
        # key 예: elevator_btn_front:btndown, elevator_btn_inside:elevatorbtnunder1
        self.latched_active_buttons: Dict[str, Dict[str, Any]] = {}

        # ------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Int32, self.current_floor_topic, self.floor_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
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

        all_detection_topics = []
        for topic in self.button_detection_topics + self.person_detection_topics:
            if topic not in all_detection_topics:
                all_detection_topics.append(topic)
        for topic in all_detection_topics:
            self.create_subscription(
                DetectionArray,
                topic,
                self._make_detection_callback(topic),
                10,
            )

        # ------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------
        self.elevator_start_pub = self.create_publisher(Int32, self.elevator_start_topic, 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # 메니퓰레이터 task command는 이벤트성 신호이므로 reliable + transient_local로 발행한다.
        # 단, 로봇팔 subscriber가 volatile QoS여도 받을 수 있도록 일정 시간 반복 발행도 함께 수행한다.
        self.manipulator_task_qos = QoSProfile(depth=10)
        self.manipulator_task_qos.reliability = ReliabilityPolicy.RELIABLE
        self.manipulator_task_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.manipulator_task_cmd_pub = self.create_publisher(
            String,
            self.manipulator_task_cmd_topic,
            self.manipulator_task_qos,
        )

        # ------------------------------------------------------------
        # Clients
        # ------------------------------------------------------------
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.load_map_client = self.create_client(LoadMap, "/map_server/load_map")
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )

        # ------------------------------------------------------------
        # Local speaker
        # ------------------------------------------------------------
        self.speaker = LocalMp3Speaker(
            node=self,
            sound_path=self.sound_path,
            enabled=self.sound_enabled,
        )

        self.get_logger().info(
            "ElevatorDeliveryFinalWithManipulator ready. "
            f"waypoint_file={self.waypoint_file}, "
            f"start_floor={self.start_floor}, "
            f"target_floor_key={self.target_floor_key}, "
            f"target_floor_signal={self.target_floor_signal}, "
            f"scan_topic={self.scan_topic}, "
            f"cmd_vel_topic={self.cmd_vel_topic}, "
            f"manipulator_task_cmd_topic={self.manipulator_task_cmd_topic}, "
            f"manipulator_task_result_topic={self.manipulator_task_result_topic}, "
            f"manipulator_task_state_topic={self.manipulator_task_state_topic}, "
            f"outside_button_task_cmd={self.outside_button_task_cmd}, "
            f"inside_button_task_cmd={self.inside_button_task_cmd}, "
            f"destination_task_cmd={self.destination_task_cmd}, "
            f"outside_button_expected_result={self.outside_button_expected_result}, "
            f"inside_button_expected_result={self.inside_button_expected_result}, "
            f"destination_expected_result={self.destination_expected_result}, "
            f"manipulator_task_timeout_sec={self.manipulator_task_timeout_sec}, "
            f"camera_image_topic={self.camera_image_topic}, "
            f"button_detection_topics={self.button_detection_topics}, "
            f"person_detection_topics={self.person_detection_topics}, "
            f"button_active_confirm_sec={self.button_active_confirm_sec}, "
            f"door_center_deg={self.door_center_deg}, "
            f"door_open_distance={self.door_open_distance}, "
            f"forced_drive_check_obstacle={self.forced_drive_check_obstacle}"
        )

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------
    def _default_sound_path(self) -> str:
        try:
            return os.path.join(get_package_share_directory("tts_speaker"), "sounds")
        except PackageNotFoundError:
            return os.path.join(os.getcwd(), "sounds")

    def _get_string_list_param(self, name: str) -> List[str]:
        value = self.get_parameter(name).value
        if value is None:
            return []
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        return [str(item) for item in list(value)]

    # ------------------------------------------------------------------
    # YAML helpers
    # ------------------------------------------------------------------
    def resolve_waypoint_file(self, waypoint_file_param: str) -> str:
        if waypoint_file_param:
            return waypoint_file_param

        config_dir = os.path.join(get_package_share_directory("amr_navigator"), "config")
        candidates = [
            "waypoint2.yaml",
            "waypoints2.yaml",
            "wayypoints2.yaml",
        ]
        for filename in candidates:
            path = os.path.join(config_dir, filename)
            if os.path.exists(path):
                return path
        return os.path.join(config_dir, "waypoint2.yaml")

    def _floor_key_candidates(self, floor: FloorKey) -> List[FloorKey]:
        candidates: List[FloorKey] = [floor]
        if isinstance(floor, int):
            candidates.append(str(floor))
            if floor == 0:
                candidates.extend(["B1", "b1", "Basement1", "basement1"])
        else:
            floor_str = str(floor)
            candidates.append(floor_str)
            if floor_str.isdigit():
                candidates.append(int(floor_str))
            if floor_str.upper() == "B1":
                candidates.extend([0, "0"])

        deduped: List[FloorKey] = []
        for item in candidates:
            if item not in deduped:
                deduped.append(item)
        return deduped

    def get_floor_info(self, floor: FloorKey) -> Dict[str, Any]:
        maps = self.wp.get("maps", {})
        for key in self._floor_key_candidates(floor):
            if key in maps:
                return maps[key]
        raise KeyError(
            f"waypoint YAML에 maps.{floor} 정보가 없습니다. 현재 maps 키={list(maps.keys())}"
        )

    def require_keys(self, data: Dict[str, Any], keys: List[str], label: str):
        missing = [key for key in keys if key not in data]
        if missing:
            raise KeyError(f"{label}에 필요한 키가 없습니다: {missing}")

    def require_pose(self, floor_info: Dict[str, Any], pose_key: str, label: str) -> PoseDict:
        if pose_key not in floor_info:
            raise KeyError(f"{label}에 {pose_key} waypoint가 없습니다.")
        pose = floor_info[pose_key]
        for field in ("x", "y", "yaw"):
            if field not in pose:
                raise KeyError(f"{label}.{pose_key}.{field} 값이 없습니다.")
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

    def _normalize_task_text(self, text: str) -> str:
        return str(text).strip()

    def _is_task_result_match(
        self,
        received: Optional[str],
        expected_result: str,
        task_cmd: str,
    ) -> bool:
        """
        /manipulator_task_result 문자열 판정.

        기본은 expected_result와 정확히 같은 문자열을 기대한다.
        다만 task manager 구현 중 문자열 포맷이 조금 다를 수 있으므로
        다음 형태도 성공으로 인정한다.
        - expected_result
        - <task_cmd>_done
        - <task_cmd>:done / <task_cmd>:success / <task_cmd>:completed
        - done:<task_cmd> / success:<task_cmd> / completed:<task_cmd>
        - done / success / completed / true / 1
        """
        if received is None:
            return False

        data = self._normalize_task_text(received)
        expected = self._normalize_task_text(expected_result)
        cmd = self._normalize_task_text(task_cmd)

        if not data:
            return False
        if data == expected:
            return True

        data_l = data.lower()
        expected_l = expected.lower()
        cmd_l = cmd.lower()

        accepted = {
            expected_l,
            f"{cmd_l}_done",
            f"{cmd_l}:done",
            f"{cmd_l}:success",
            f"{cmd_l}:completed",
            f"done:{cmd_l}",
            f"success:{cmd_l}",
            f"completed:{cmd_l}",
            "done",
            "success",
            "completed",
            "true",
            "1",
        }
        return data_l in accepted

    def manipulator_task_result_callback(self, msg: String):
        data = self._normalize_task_text(msg.data)
        self.latest_manipulator_result = data
        self.get_logger().info(
            f"Received manipulator task result from {self.manipulator_task_result_topic}: "
            f"data='{data}'"
        )

    def manipulator_task_state_callback(self, msg: String):
        data = self._normalize_task_text(msg.data)
        self.latest_manipulator_state = data
        self.get_logger().info(
            f"Received manipulator task state from {self.manipulator_task_state_topic}: "
            f"data='{data}'"
        )

    def _make_detection_callback(self, topic: str):
        def callback(msg: DetectionArray):
            self.latest_detections[topic] = (time.time(), list(msg.detections))

        return callback

    # ------------------------------------------------------------------
    # Pose / map
    # ------------------------------------------------------------------
    def make_pose(self, pose_dict: PoseDict, frame_id: str = "map") -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pose_dict["x"])
        msg.pose.position.y = float(pose_dict["y"])
        msg.pose.orientation = yaw_to_quaternion(float(pose_dict["yaw"]))
        return msg

    def publish_initial_pose(self, pose_dict: PoseDict):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(pose_dict["x"])
        msg.pose.pose.position.y = float(pose_dict["y"])
        msg.pose.pose.orientation = yaw_to_quaternion(float(pose_dict["yaw"]))
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        self.get_logger().info(
            "Publishing initial pose: "
            f"x={pose_dict['x']}, y={pose_dict['y']}, yaw={pose_dict['yaw']}"
        )
        for _ in range(20):
            self.initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def is_close_to_pose(self, pose_dict: PoseDict, radius: float) -> bool:
        if self.current_pose is None:
            return False
        dx = self.current_pose.position.x - float(pose_dict["x"])
        dy = self.current_pose.position.y - float(pose_dict["y"])
        dist = math.hypot(dx, dy)
        return dist <= radius

    def load_map(self, map_yaml_name: str) -> bool:
        map_path = os.path.join(get_package_share_directory("amr_navigator"), "map", map_yaml_name)
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
    def go_to_pose(self, pose_dict: PoseDict, name: str = "goal") -> bool:
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
        count_inf_as_max: bool = False,
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
            count_inf_as_max=False,
        )

        if len(values) < self.door_min_valid_count:
            return {
                "valid_count": len(values),
                "min": None,
                "median": None,
                "max": None,
                "open_count": 0,
                "open_ratio": 0.0,
                "is_open": False,
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
            "valid_count": n,
            "min": min_v,
            "median": median,
            "max": max_v,
            "open_count": open_count,
            "open_ratio": open_ratio,
            "is_open": is_open,
        }

    def wait_until_door_open(
        self,
        label: str = "door",
        already_inside_pose: Optional[PoseDict] = None,
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

            if already_inside_pose is not None:
                if self.is_close_to_pose(already_inside_pose, self.already_inside_radius):
                    self.get_logger().info(
                        f"Robot is already near target inside pose. Skip waiting for {label}."
                    )
                    return True

            stats = self.get_door_stats()
            if stats["is_open"]:
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

    def wait_until_target_floor_then_door_open(self, target_floor: int, label: str) -> bool:
        self.get_logger().info(
            f"Waiting target floor={target_floor} and {label} open. "
            "Door-open events before target floor will be ignored."
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
                        f"Not target floor yet. current_floor={self.current_floor}, "
                        f"target_floor={target_floor}. Ignore door state."
                    )
                    last_log_time = now
                continue

            stats = self.get_door_stats()
            if stats["is_open"]:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= self.door_stable_count_required:
                self.get_logger().info(
                    f"Target floor {target_floor} reached and {label} opened. "
                    f"valid={stats['valid_count']}, "
                    f"median={stats['median']}, "
                    f"open_ratio={stats['open_ratio']:.2f}"
                )
                return True

            if now - last_log_time > 1.0:
                self.get_logger().info(
                    f"At target floor={target_floor}; waiting for {label}... "
                    f"valid={stats['valid_count']}, "
                    f"median={stats['median']}, "
                    f"open_ratio={stats['open_ratio']:.2f}, "
                    f"stable_count={stable_count}/{self.door_stable_count_required}"
                )
                last_log_time = now

        return False

    # ------------------------------------------------------------------
    # Detection helpers
    # ------------------------------------------------------------------
    def _label_matches(self, class_name: str, target_labels: Sequence[str]) -> bool:
        """일반 class_name 비교용. person 같은 비버튼 객체에 사용한다."""
        class_norm = normalize_label(class_name)
        if not class_norm:
            return False

        for target in target_labels:
            target_norm = normalize_label(target)
            if not target_norm:
                continue
            if class_norm == target_norm or target_norm in class_norm:
                return True
        return False

    def _button_state_from_label(self, class_name: str) -> Optional[str]:
        """
        YOLO class_name에서 active/deactive 상태를 분리한다.

        중요:
        - 'deactive' 문자열 안에는 'active'가 포함되어 있으므로 deactive를 먼저 검사해야 한다.
        - btn_down_deactive를 btn_down_active로 오인하면 엘리베이터가 잘못 출발할 수 있다.
        """
        raw = str(class_name).lower()
        tokens = [t for t in re.split(r"[^a-z0-9]+", raw) if t]
        token_set = set(tokens)

        if token_set.intersection({"deactive", "inactive", "disabled", "disable", "off", "false"}):
            return "deactive"
        if token_set.intersection({"active", "enabled", "enable", "on", "true"}):
            return "active"

        norm = normalize_label(raw)
        # 구분자가 없는 class_name까지 방어적으로 처리한다.
        for suffix in ("deactive", "inactive", "disabled", "disable", "off", "false"):
            if norm.endswith(suffix):
                return "deactive"
        for suffix in ("active", "enabled", "enable", "on", "true"):
            if norm.endswith(suffix):
                return "active"
        return None

    def _button_base_label_norm(self, class_name: str) -> str:
        """
        btn_down_active -> btndown
        elevator_btn_under1_deactive -> elevatorbtnunder1
        처럼 상태 접미사를 제거한 버튼 기본 이름을 만든다.
        """
        raw = str(class_name).lower()
        tokens = [t for t in re.split(r"[^a-z0-9]+", raw) if t]
        state_tokens = {
            "active", "deactive", "inactive",
            "enabled", "enable", "disabled", "disable",
            "on", "off", "true", "false",
        }
        if len(tokens) >= 2:
            filtered = [t for t in tokens if t not in state_tokens]
            if filtered:
                return "".join(filtered)

        norm = normalize_label(raw)
        for suffix in (
            "deactive", "inactive", "disabled", "disable", "false",
            "active", "enabled", "enable", "true",
            "off", "on",
        ):
            if norm.endswith(suffix) and len(norm) > len(suffix):
                return norm[: -len(suffix)]
        return norm

    def _button_base_matches(self, class_name: str, target_labels: Sequence[str]) -> bool:
        class_base = self._button_base_label_norm(class_name)
        if not class_base:
            return False

        for target in target_labels:
            target_base = self._button_base_label_norm(target)
            if not target_base:
                continue
            if class_base == target_base or target_base in class_base:
                return True
        return False

    def _find_recent_detection(
        self,
        topics: Sequence[str],
        labels: Sequence[str],
        score_threshold: float,
    ) -> Tuple[bool, Optional[str], float]:
        now = time.time()
        best_label: Optional[str] = None
        best_score = 0.0

        for topic in topics:
            if topic not in self.latest_detections:
                continue
            stamp, detections = self.latest_detections[topic]
            if now - stamp > self.detection_stale_timeout_sec:
                continue

            for det in detections:
                class_name = str(getattr(det, "class_name", ""))
                score = float(getattr(det, "score", 0.0))
                if score < score_threshold:
                    continue
                if self._label_matches(class_name, labels):
                    if score >= best_score:
                        best_score = score
                        best_label = class_name

        return best_label is not None, best_label, best_score

    def _find_recent_button(
        self,
        topics: Sequence[str],
        labels: Sequence[str],
        score_threshold: float,
        required_state: Optional[str] = None,
    ) -> Tuple[bool, Optional[str], float, Optional[str]]:
        """
        required_state:
        - 'active'   : active 버튼만 True
        - 'deactive' : deactive 버튼만 True
        - None       : active/deactive 무관하게 target 버튼이 보이면 True
        """
        now = time.time()
        best_label: Optional[str] = None
        best_score = 0.0
        best_state: Optional[str] = None

        for topic in topics:
            if topic not in self.latest_detections:
                continue
            stamp, detections = self.latest_detections[topic]
            if now - stamp > self.detection_stale_timeout_sec:
                continue

            for det in detections:
                class_name = str(getattr(det, "class_name", ""))
                score = float(getattr(det, "score", 0.0))
                if score < score_threshold:
                    continue
                if not self._button_base_matches(class_name, labels):
                    continue

                state = self._button_state_from_label(class_name)
                if required_state == "active":
                    if state != "active":
                        if state is not None or not self.allow_button_state_unknown_as_active:
                            continue
                elif required_state == "deactive":
                    if state != "deactive":
                        continue

                if score >= best_score:
                    best_score = score
                    best_label = class_name
                    best_state = state

        return best_label is not None, best_label, best_score, best_state

    def has_button_active(self, labels: Sequence[str]) -> Tuple[bool, Optional[str], float, Optional[str]]:
        return self._find_recent_button(
            topics=self.button_detection_topics,
            labels=labels,
            score_threshold=self.button_score_threshold,
            required_state="active",
        )

    def has_target_button_any_state(
        self,
        labels: Sequence[str],
    ) -> Tuple[bool, Optional[str], float, Optional[str]]:
        return self._find_recent_button(
            topics=self.button_detection_topics,
            labels=labels,
            score_threshold=self.button_score_threshold,
            required_state=None,
        )

    def has_person(self) -> Tuple[bool, Optional[str], float]:
        return self._find_recent_detection(
            topics=self.person_detection_topics,
            labels=self.person_labels,
            score_threshold=self.person_score_threshold,
        )

    def _button_latch_key(self, labels: Sequence[str], context_label: str) -> str:
        base_labels = [self._button_base_label_norm(label) for label in labels]
        base_labels = [label for label in base_labels if label]
        base = "|".join(sorted(set(base_labels))) if base_labels else "unknown_button"
        return f"{context_label}:{base}"

    def is_button_latched_active(self, labels: Sequence[str], context_label: str) -> bool:
        return self._button_latch_key(labels, context_label) in self.latched_active_buttons

    def latch_button_active(
        self,
        labels: Sequence[str],
        context_label: str,
        class_name: Optional[str],
        score: float,
    ):
        key = self._button_latch_key(labels, context_label)
        if key in self.latched_active_buttons:
            return
        self.latched_active_buttons[key] = {
            "context": context_label,
            "labels": list(labels),
            "class_name": class_name,
            "score": float(score),
            "latched_at": time.time(),
        }
        self.get_logger().info(
            f"Button latched active until target floor: key={key}, "
            f"class={class_name}, score={score:.3f}"
        )

    def clear_button_latches(self, reason: str):
        if not self.latched_active_buttons:
            return
        keys = list(self.latched_active_buttons.keys())
        self.latched_active_buttons.clear()
        self.get_logger().info(f"Cleared button active latches. reason={reason}, keys={keys}")

    def wait_until_button_active(
        self,
        labels: Sequence[str],
        button_label_for_log: str,
        context_label: str,
    ) -> bool:
        latch_key = self._button_latch_key(labels, context_label)
        if self.is_button_latched_active(labels, context_label):
            self.get_logger().info(
                f"{button_label_for_log} is already latched active at {context_label}. "
                f"Skip perception re-check. latch_key={latch_key}"
            )
            return True

        self.get_logger().info(
            f"Waiting for {button_label_for_log} active at {context_label}. "
            f"button_topics={self.button_detection_topics}, person_topics={self.person_detection_topics}, "
            f"target_base_labels={list(labels)}, "
            f"active_confirm_sec={self.button_active_confirm_sec:.2f}, "
            f"latch_key={latch_key}, "
            f"allow_unknown_state_as_active={self.allow_button_state_unknown_as_active}"
        )

        active_since: Optional[float] = None
        last_active_class: Optional[str] = None
        last_active_score = 0.0
        last_log_time = 0.0
        alarm_key = f"btn_not_perception:{context_label}"

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.is_button_latched_active(labels, context_label):
                self.get_logger().info(
                    f"{button_label_for_log} became latched active while waiting. "
                    f"Skip perception re-check. latch_key={latch_key}"
                )
                return True

            button_active, active_class, active_score, active_state = self.has_button_active(labels)
            target_visible, target_class, target_score, target_state = self.has_target_button_any_state(labels)
            person_visible, person_class, person_score = self.has_person()

            now = time.time()
            active_elapsed = 0.0

            if button_active:
                # active가 처음 보인 시점을 저장하고, 이후 연속으로 2초 이상 유지될 때만 확정한다.
                if active_since is None:
                    active_since = now
                    last_active_class = active_class
                    last_active_score = active_score
                    self.get_logger().info(
                        f"{button_label_for_log} active candidate started at {context_label}. "
                        f"class={active_class}, state={active_state}, score={active_score:.3f}"
                    )
                else:
                    # active 클래스가 바뀌더라도 같은 목표 버튼의 active라면 연속 active로 인정한다.
                    last_active_class = active_class
                    last_active_score = active_score

                active_elapsed = now - active_since
                if active_elapsed >= self.button_active_confirm_sec:
                    self.latch_button_active(
                        labels=labels,
                        context_label=context_label,
                        class_name=last_active_class,
                        score=last_active_score,
                    )
                    self.get_logger().info(
                        f"{button_label_for_log} active confirmed at {context_label}. "
                        f"continuous_active_sec={active_elapsed:.2f}, "
                        f"class={last_active_class}, score={last_active_score:.3f}. "
                        "From now until target floor, this button is treated as active."
                    )
                    return True
            else:
                if active_since is not None:
                    self.get_logger().info(
                        f"{button_label_for_log} active candidate reset at {context_label}. "
                        f"It was active for {now - active_since:.2f}s, "
                        f"required={self.button_active_confirm_sec:.2f}s."
                    )
                active_since = None
                last_active_class = None
                last_active_score = 0.0

                # 버튼이 deactive라도 보이는 상황이면 '버튼이 안 보임' 안내는 하지 않는다.
                # person 때문에 목표 버튼 자체가 안 보이는 경우에만 안내한다.
                if person_visible and not target_visible:
                    self.speaker.play_alarm_if_idle(
                        self.btn_not_perception_sound,
                        alarm_key=alarm_key,
                        min_interval_sec=self.alarm_repeat_min_interval_sec,
                    )

            if now - last_log_time > 1.0:
                self.get_logger().info(
                    f"Waiting {button_label_for_log} at {context_label}: "
                    f"button_active={button_active}, "
                    f"active_class={active_class}, "
                    f"active_state={active_state}, "
                    f"active_score={active_score:.3f}, "
                    f"active_elapsed={active_elapsed:.2f}/{self.button_active_confirm_sec:.2f}s, "
                    f"latched={self.is_button_latched_active(labels, context_label)}, "
                    f"target_visible={target_visible}, "
                    f"target_class={target_class}, "
                    f"target_state={target_state}, "
                    f"target_score={target_score:.3f}, "
                    f"person_visible={person_visible}, "
                    f"person_class={person_class}, "
                    f"person_score={person_score:.3f}"
                )
                last_log_time = now

        return False

    # ------------------------------------------------------------------
    # YOLO process control
    # ------------------------------------------------------------------
    def _build_yolov8_command(self) -> List[str]:
        if self.yolov8_command:
            return shlex.split(self.yolov8_command)

        return [
            "ros2",
            "run",
            self.yolov8_package,
            self.yolov8_executable,
            "--ros-args",
            "-r",
            f"camera/camera/color/image_raw:={self.camera_image_topic}",
        ]

    def start_yolov8_node(self) -> bool:
        if self.yolov8_process is not None and self.yolov8_process.poll() is None:
            self.get_logger().info("yolov8_node is already running by this manager.")
            return True

        cmd = self._build_yolov8_command()
        self.get_logger().info(f"Starting yolov8_node: {' '.join(cmd)}")

        try:
            self.yolov8_process = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start yolov8_node: {e}")
            self.yolov8_process = None
            return False

        self.spin_sleep(self.yolov8_startup_wait_sec)

        if self.yolov8_process.poll() is not None:
            self.get_logger().error(
                f"yolov8_node exited immediately. returncode={self.yolov8_process.returncode}"
            )
            return False

        self.get_logger().info("yolov8_node started.")
        return True

    def stop_yolov8_node(self):
        if self.yolov8_process is None:
            return

        if self.yolov8_process.poll() is not None:
            self.get_logger().info("yolov8_node is already stopped.")
            self.yolov8_process = None
            return

        self.get_logger().info("Stopping yolov8_node...")
        try:
            os.killpg(os.getpgid(self.yolov8_process.pid), signal.SIGINT)
            self.yolov8_process.wait(timeout=self.yolov8_stop_timeout_sec)
        except subprocess.TimeoutExpired:
            self.get_logger().warn("yolov8_node did not stop by SIGINT. Terminating...")
            try:
                os.killpg(os.getpgid(self.yolov8_process.pid), signal.SIGTERM)
                self.yolov8_process.wait(timeout=1.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.yolov8_process.pid), signal.SIGKILL)
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().warn(f"Failed while stopping yolov8_node: {e}")
        finally:
            self.yolov8_process = None
            self.get_logger().info("yolov8_node stopped.")

    # ------------------------------------------------------------------
    # Direct / forced drive
    # ------------------------------------------------------------------
    def get_front_min_distance(self) -> Optional[float]:
        values = self.get_sector_ranges(
            center_deg=self.door_center_deg,
            half_width_deg=10.0,
            count_inf_as_max=False,
        )
        if not values:
            return None
        return min(values)

    def stop_robot(self, repeat: int = 20):
        stop = Twist()
        for _ in range(repeat):
            self.cmd_vel_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

    def rotate_relative_simple(self, angle_rad: float, label: str = "Rotate") -> bool:
        angle = normalize_angle(angle_rad)
        if abs(angle) < 1.0e-3:
            self.get_logger().info(f"{label}: angle is almost zero. Skip rotate.")
            return True

        if self.rotate_speed <= 0.0:
            self.get_logger().error("rotate_speed must be positive.")
            return False

        direction = 1.0 if angle >= 0.0 else -1.0
        angular_z = abs(self.rotate_speed) * direction
        duration = abs(angle) / abs(self.rotate_speed)

        self.get_logger().info(
            f"{label}: rotate start. "
            f"angle={angle:.3f} rad, angular_z={angular_z:.3f} rad/s, duration={duration:.2f} s"
        )

        twist = Twist()
        twist.angular.z = angular_z
        start = time.time()
        while rclpy.ok() and (time.time() - start) < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        self.get_logger().info(f"{label}: rotate done.")
        return True

    def drive_straight_simple(
        self,
        distance_m: float,
        speed_mps: float,
        label: str,
        check_obstacle: Optional[bool] = None,
    ) -> bool:
        if check_obstacle is None:
            check_obstacle = self.forced_drive_check_obstacle

        if abs(speed_mps) < 1.0e-6:
            self.get_logger().error("speed_mps is zero. Cannot drive.")
            return False

        direction = 1.0 if distance_m >= 0.0 else -1.0
        speed = abs(speed_mps) * direction
        duration = abs(distance_m) / abs(speed_mps)

        self.get_logger().info(
            f"{label}: direct drive start. "
            f"distance={distance_m:.3f} m, speed={speed:.3f} m/s, "
            f"duration={duration:.2f} s, check_obstacle={check_obstacle}"
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

    def play_robot_for_move_once(self, move_key: str):
        self.speaker.play_once(
            self.robot_for_move_sound,
            once_key=f"robot_for_move:{move_key}",
            wait_if_busy=True,
            busy_timeout_sec=None,
        )

    def force_move_between_waypoints(
        self,
        from_pose: PoseDict,
        to_pose: PoseDict,
        label: str,
        speed_mps: float,
        distance_override: float = 0.0,
        check_obstacle: Optional[bool] = None,
        move_sound_key: Optional[str] = None,
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

        if move_sound_key is not None:
            self.play_robot_for_move_once(move_sound_key)

        if self.publish_initial_pose_before_forced_move:
            self.publish_initial_pose(from_pose)
            self.spin_sleep(self.initial_pose_sleep_sec)

        from_yaw = float(from_pose["yaw"])
        to_yaw = float(to_pose["yaw"])
        bearing = pose_bearing(from_pose, to_pose)
        distance = float(distance_override) if distance_override > 0.0 else pose_distance(from_pose, to_pose)

        self.get_logger().info(
            f"{label}: forced move. "
            f"from_yaw={from_yaw:.3f}, bearing={bearing:.3f}, "
            f"to_yaw={to_yaw:.3f}, distance={distance:.3f}"
        )

        first_turn = normalize_angle(bearing - from_yaw)
        if not self.rotate_relative_simple(first_turn, f"{label} first turn"):
            return False

        if distance > 1.0e-3:
            if not self.drive_straight_simple(
                distance,
                speed_mps,
                label,
                check_obstacle=check_obstacle,
            ):
                return False

        final_turn = normalize_angle(to_yaw - bearing)
        if not self.rotate_relative_simple(final_turn, f"{label} final turn"):
            return False

        if self.publish_initial_pose_after_forced_move:
            self.publish_initial_pose(to_pose)
            self.spin_sleep(self.initial_pose_sleep_sec)

        return True

    # ------------------------------------------------------------------
    # Manipulator task manager handshake
    # ------------------------------------------------------------------
    def publish_task_command(
        self,
        task_cmd: str,
        repeat: int = 1,
        interval_sec: float = 0.0,
    ):
        """
        /manipulator_task_cmd 로 std_msgs/String 작업 명령을 발행한다.

        예)
        - data: "OUTSIDE_BTN_FRONT"
        - data: "INSIDE_BTN_FRONT"
        - data: "DESTINATION_UNLOAD"

        작업 명령을 해제하거나 이전 latched 값을 지울 때는 빈 문자열 data: ""를 발행한다. task manager는 빈 명령을 무시한다.
        """
        msg = String()
        msg.data = str(task_cmd)

        repeat = max(1, int(repeat))
        for idx in range(repeat):
            self.manipulator_task_cmd_pub.publish(msg)
            self.get_logger().info(
                f"Publish manipulator task command to {self.manipulator_task_cmd_topic}: "
                f"data='{msg.data}' ({idx + 1}/{repeat})"
            )
            if idx < repeat - 1 and interval_sec > 0.0:
                self.spin_sleep(interval_sec)

    def publish_manipulator_task_only(self, task_cmd: str):
        """
        result 응답을 기다리지 않는 작업 명령 발행.
        현재 요구사항에서는 B1 destination 도착 후 destination_goal에 사용한다.
        """
        self.publish_task_command(
            "",
            repeat=2,
            interval_sec=0.05,
        )
        self.spin_sleep(0.1)
        self.publish_task_command(
            task_cmd,
            repeat=self.manipulator_cmd_publish_count,
            interval_sec=self.manipulator_cmd_publish_interval_sec,
        )

    def send_manipulator_task_and_wait(
        self,
        task_cmd: str,
        expected_result: str,
    ) -> bool:
        """
        AMR -> Manipulator:
          /manipulator_task_cmd String(data=<task_cmd>) 발행

        Manipulator -> AMR:
          /manipulator_task_result String(data=<expected_result>) 수신 대기

        예)
        - task_cmd="OUTSIDE_BTN_FRONT"
        - expected_result="OUTSIDE_BTN_DONE"
        """
        self.current_manipulator_task_cmd = task_cmd
        self.current_manipulator_expected_result = expected_result
        self.latest_manipulator_result = None

        self.get_logger().info(
            f"Manipulator task handshake start: publish {self.manipulator_task_cmd_topic} "
            f"String(data='{task_cmd}'), wait {self.manipulator_task_result_topic} "
            f"String(data='{expected_result}')"
        )

        # 이전 latched command 문자열이 남아 있을 가능성을 줄이기 위해
        # task manager가 무시하는 빈 문자열을 먼저 짧게 발행한다.
        self.publish_task_command(
            "",
            repeat=2,
            interval_sec=0.05,
        )
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

            if self._is_task_result_match(
                self.latest_manipulator_result,
                expected_result,
                task_cmd,
            ):
                self.get_logger().info(
                    f"Manipulator task done: received result='{self.latest_manipulator_result}'. "
                    f"Clear command with empty String(data='') and continue navigation."
                )
                self.publish_task_command(
                    "",
                    repeat=3,
                    interval_sec=0.05,
                )
                self.current_manipulator_task_cmd = None
                self.current_manipulator_expected_result = None
                return True

            if (
                self.manipulator_task_timeout_sec > 0.0
                and now - start_time > self.manipulator_task_timeout_sec
            ):
                self.get_logger().error(
                    f"Timeout waiting for manipulator task result. "
                    f"expected='{expected_result}', latest='{self.latest_manipulator_result}', "
                    f"timeout={self.manipulator_task_timeout_sec:.1f}s"
                )
                self.publish_task_command(
                    "",
                    repeat=3,
                    interval_sec=0.05,
                )
                self.current_manipulator_task_cmd = None
                self.current_manipulator_expected_result = None
                return False

            # 로봇팔 subscriber가 volatile QoS이거나 순간적으로 연결이 늦는 경우를 대비해
            # result가 올 때까지 task_cmd 문자열을 주기적으로 재발행한다.
            if now - last_republish_time >= self.manipulator_cmd_republish_interval_sec:
                msg = String()
                msg.data = task_cmd
                self.manipulator_task_cmd_pub.publish(msg)
                last_republish_time = now

            if now - last_log_time > 1.0:
                elapsed = now - start_time
                self.get_logger().info(
                    f"Waiting manipulator task result: expected='{expected_result}', "
                    f"latest_result='{self.latest_manipulator_result}', "
                    f"latest_state='{self.latest_manipulator_state}', "
                    f"elapsed={elapsed:.1f}s, command='{task_cmd}'"
                )
                last_log_time = now

        return False

    # ------------------------------------------------------------------
    # Elevator floor estimator control
    # ------------------------------------------------------------------
    def start_elevator_floor_estimation(self, target_floor: int):
        self.get_logger().info(
            f"Notify elevator_floor_node: start_floor={self.start_floor}, "
            f"target_floor_signal={target_floor}"
        )

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

    def compute_exit_distance(
        self,
        start_floor_info: Dict[str, Any],
        target_floor_info: Dict[str, Any],
    ) -> float:
        if self.exit_distance_override > 0.0:
            self.get_logger().info(f"Use exit_distance_override={self.exit_distance_override:.3f} m")
            return self.exit_distance_override

        for inside_key in ("elevator_inside_for_exit", "elevator_inside"):
            if inside_key in target_floor_info and "elevator_exit" in target_floor_info:
                distance = pose_distance(target_floor_info[inside_key], target_floor_info["elevator_exit"])
                self.get_logger().info(
                    f"Use target floor {inside_key}->elevator_exit distance={distance:.3f} m"
                )
                return distance

        if "elevator_inside_for_exit" in start_floor_info and "elevator_front" in start_floor_info:
            distance = pose_distance(start_floor_info["elevator_inside_for_exit"], start_floor_info["elevator_front"])
            self.get_logger().warning(
                "Target floor has no elevator_inside/elevator_inside_for_exit. "
                "Use start floor elevator_inside_for_exit -> elevator_front distance as exit distance. "
                f"distance={distance:.3f} m. Tune exit_distance_override if needed."
            )
            return distance

        if "elevator_inside" in start_floor_info and "elevator_front" in start_floor_info:
            distance = pose_distance(start_floor_info["elevator_inside"], start_floor_info["elevator_front"])
            self.get_logger().warning(
                "Target floor has no elevator_inside/elevator_inside_for_exit. "
                "Use start floor elevator_inside -> elevator_front distance as exit distance. "
                f"distance={distance:.3f} m. Tune exit_distance_override if needed."
            )
            return distance

        raise KeyError(
            "Cannot compute exit distance. Add B1 elevator_inside/elevator_inside_for_exit "
            "to waypoint YAML or pass exit_distance_override."
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
                    "map_yaml",
                    "room_front",
                    "elevator_btn_front",
                    "elevator_front",
                    "elevator_inside",
                    "elevator_btn_inside",
                    "elevator_inside_for_exit",
                ],
                f"{self.start_floor}F map info",
            )
            self.require_keys(
                target_info,
                ["map_yaml", "elevator_exit", "destination"],
                f"{self.target_floor_key} map info",
            )

            room_front = self.require_pose(start_info, "room_front", f"{self.start_floor}F")
            elevator_btn_front = self.require_pose(
                start_info,
                "elevator_btn_front",
                f"{self.start_floor}F",
            )
            elevator_front = self.require_pose(start_info, "elevator_front", f"{self.start_floor}F")
            elevator_inside = self.require_pose(start_info, "elevator_inside", f"{self.start_floor}F")
            elevator_btn_inside = self.require_pose(
                start_info,
                "elevator_btn_inside",
                f"{self.start_floor}F",
            )
            elevator_inside_for_exit = self.require_pose(
                start_info,
                "elevator_inside_for_exit",
                f"{self.start_floor}F",
            )
            elevator_exit = self.require_pose(target_info, "elevator_exit", self.target_floor_key)
            destination = self.require_pose(target_info, "destination", self.target_floor_key)
        except Exception as e:
            self.get_logger().error(str(e))
            return

        self.get_logger().info(
            f"Mission requested. start_floor={self.start_floor}, "
            f"target_floor_key={self.target_floor_key}, "
            f"target_floor_signal={self.target_floor_signal}"
        )

        try:
            # 1) 시작 층 map 로드 및 시작 pose 세팅: room_front
            if not self.load_map(start_info["map_yaml"]):
                return

            self.publish_initial_pose(room_front)
            self.spin_sleep(2.0)

            # room_front 출발 시 starting_bgm 1회
            self.speaker.play_once(
                self.starting_bgm_sound,
                once_key="starting_bgm",
                wait_if_busy=False,
            )

            # 2) room_front -> elevator_btn_front: Nav2 이동
            # 요구사항에 robot_for_move가 명시된 구간은 아니므로 여기서는 재생하지 않는다.
            if not self.go_to_pose(elevator_btn_front, f"{self.start_floor}F elevator_btn_front"):
                return

            # 3) elevator_btn_front 도착 시 로봇팔 task manager에 외부 버튼 조작 task 전달
            # 기존: btn_down_active가 2초 이상 유지되면 다음 이동
            # 변경: /manipulator_task_cmd로 outside_button_task_cmd 발행 후
            #       /manipulator_task_result에서 outside_button_expected_result 수신 시 다음 이동
            if not self.send_manipulator_task_and_wait(
                task_cmd=self.outside_button_task_cmd,
                expected_result=self.outside_button_expected_result,
            ):
                return

            # 4) 로봇팔의 외부 버튼 동작 완료 후 elevator_front로 강제 이동
            if not self.force_move_between_waypoints(
                elevator_btn_front,
                elevator_front,
                "Forced approach: elevator_btn_front -> elevator_front",
                self.front_approach_speed,
                distance_override=self.front_approach_distance_override,
                check_obstacle=self.forced_drive_check_obstacle,
                move_sound_key="elevator_btn_front_to_elevator_front",
            ):
                return

            # 5) 시작 층 엘리베이터 문 열림 대기
            if not self.wait_until_door_open(
                f"{self.start_floor}F elevator door",
                already_inside_pose=elevator_inside,
            ):
                return

            # 6) elevator_front -> elevator_inside: 강제 이동
            if self.is_close_to_pose(elevator_inside, self.already_inside_radius):
                self.get_logger().info("Already inside elevator. Skip forced boarding move.")
                self.publish_initial_pose(elevator_inside)
                self.spin_sleep(self.initial_pose_sleep_sec)
            else:
                if not self.force_move_between_waypoints(
                    elevator_front,
                    elevator_inside,
                    "Boarding elevator: elevator_front -> elevator_inside",
                    self.boarding_speed,
                    distance_override=self.boarding_distance_override,
                    check_obstacle=self.forced_drive_check_obstacle,
                    move_sound_key="elevator_front_to_elevator_inside",
                ):
                    return

            # 6-1) elevator_inside 도착 즉시 층수 추정 시작
            self.start_elevator_floor_estimation(self.target_floor_signal)

            # 7) elevator_inside -> elevator_btn_inside: 강제 이동
            if not self.force_move_between_waypoints(
                elevator_inside,
                elevator_btn_inside,
                "Move inside elevator: elevator_inside -> elevator_btn_inside",
                self.forced_move_speed,
                check_obstacle=self.forced_drive_check_obstacle,
                move_sound_key="elevator_inside_to_elevator_btn_inside",
            ):
                return

            # 8) elevator_btn_inside 도착 시 로봇팔 task manager에 내부 버튼 조작 task 전달
            # 기존: elevator_btn_under1_active가 2초 이상 유지되면 다음 이동
            # 변경: /manipulator_task_cmd로 inside_button_task_cmd 발행 후
            #       /manipulator_task_result에서 inside_button_expected_result 수신 시 다음 이동
            if not self.send_manipulator_task_and_wait(
                task_cmd=self.inside_button_task_cmd,
                expected_result=self.inside_button_expected_result,
            ):
                return

            # 9) 로봇팔의 내부 버튼 동작 완료 후 elevator_inside_for_exit로 강제 이동
            if not self.force_move_between_waypoints(
                elevator_btn_inside,
                elevator_inside_for_exit,
                "Move inside elevator: elevator_btn_inside -> elevator_inside_for_exit",
                self.forced_move_speed,
                check_obstacle=self.forced_drive_check_obstacle,
                move_sound_key="elevator_btn_inside_to_elevator_inside_for_exit",
            ):
                return

            # 10) elevator_inside_for_exit 도착 시 혹시 실행 중인 YOLO 프로세스가 있으면 종료
            self.stop_yolov8_node()

            # 10) 목표 층 도착 + 문 열림 대기
            if not self.wait_until_target_floor_then_door_open(
                self.target_floor_signal,
                f"{self.target_floor_key} elevator door",
            ):
                return

            # 목표 층에 도달했으므로 버튼 active latch는 더 이상 유지할 필요가 없다.
            self.clear_button_latches("target floor reached")

            # 11) 목표 층 map으로 전환
            if not self.load_map(target_info["map_yaml"]):
                return

            # 12) elevator_inside_for_exit -> elevator_exit: 강제 직진 이동 후 pose 보정
            exit_distance = self.compute_exit_distance(start_info, target_info)
            self.play_robot_for_move_once("elevator_inside_for_exit_to_elevator_exit")
            if not self.drive_straight_simple(
                exit_distance,
                self.exit_speed,
                f"Exit elevator at {self.target_floor_key}: elevator_inside_for_exit -> elevator_exit",
                check_obstacle=self.forced_drive_check_obstacle,
            ):
                return

            self.publish_initial_pose(elevator_exit)
            self.clear_costmaps()
            self.spin_sleep(2.0)

            # 13) elevator_exit -> destination: Nav2 이동
            if not self.go_to_pose(destination, f"{self.target_floor_key} destination"):
                return

            # 14) B1 destination 도착 시 로봇팔 task manager에 목적지 하역 작업 명령 전달
            # task manager 기준: DESTINATION_UNLOAD -> UNLOAD_DONE
            if not self.send_manipulator_task_and_wait(
                task_cmd=self.destination_task_cmd,
                expected_result=self.destination_expected_result,
            ):
                return

            # 15) destination 도착 안내 1회, 5초 뒤 간식 전달 안내 1회
            self.speaker.play_once(
                self.destination_sound,
                once_key="destination",
                wait_if_busy=True,
            )
            self.spin_sleep(self.give_snack_delay_sec)
            self.speaker.play_once(
                self.give_snack_sound,
                once_key="give_snack",
                wait_if_busy=True,
            )

            self.get_logger().info("Mission complete.")

        finally:
            # 실패/중단 시에도 YOLO 프로세스가 남지 않도록 정리한다.
            self.stop_yolov8_node()
            self.stop_robot(repeat=5)


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorDeliveryFinalWithManipulator()
    try:
        node.run()
    finally:
        node.speaker.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()