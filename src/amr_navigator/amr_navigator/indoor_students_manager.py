#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
indoor_students_manager.py

Nav2 기반 좁은 실내 반복 미션 매니저.

Route:
  start -> elevator_btn_front -> unload_spot -> start -> ... 무한 반복

핵심 의도:
  - 임의 cmd_vel 강제 이동을 하지 않는다.
  - 모든 이동은 NavigateToPose action으로 수행한다.
  - global path planning + local path planning을 유지한다.
  - 단, 목표 근처에서 inflation/cost 때문에 로봇이 주춤거리며 waypoint로 못 들어가는 경우,
    goal을 성공 처리하거나 cancel하지 않고, final-approach 모드로 cost 회피 성향만 낮춘다.
  - final-approach 모드에서도 Nav2 action은 계속 유지되며, 실제 NavigateToPose 성공을 기다린다.

YAML example:
maps:
  indoor:
    map_yaml: "indoor_map_final.yaml"
    start:
      x: 0.106
      y: 0.0254
      yaw: -0.0761
    elevator_btn_front:
      x: 5.49
      y: -1.29
      yaw: 0.0082
    unload_spot:
      x: 4.68
      y: -0.104
      yaw: -3.1227

Manipulator handshake:
  elevator_btn_front 도착:
    publish /manipulator_task_cmd    String(data="INSIDE_BTN_FRONT")
    wait    /manipulator_task_result String(data="INSIDE_BTN_DONE")

  unload_spot 도착:
    publish /manipulator_task_cmd    String(data="DESTINATION_UNLOAD")
    wait    /manipulator_task_result String(data="UNLOAD_DONE")

Sound:
  최초 start 출발 전: starting_bgm.mp3 -> robot_for_move.mp3 각각 1회
  elevator_btn_front 도착: btn_clk_start.mp3 1회
  unload_spot 도착: destination.mp3 1회
  UNLOAD_DONE 수신: recover.mp3 1회
"""

import math
import os
import time
from typing import Any, Dict, List, Optional

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from std_msgs.msg import String


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


class LocalMp3Speaker:
    def __init__(self, node: Node, sound_path: str, enabled: bool = True):
        self.node = node
        self.sound_path = sound_path
        self.enabled = enabled
        self.played_once_keys = set()
        self.mixer = None

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

        # ------------------------------------------------------------
        # File / map parameters
        # ------------------------------------------------------------
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("map_key", "indoor")
        self.declare_parameter("package_name", "amr_navigator")
        self.declare_parameter("config_dir_name", "config")
        self.declare_parameter("map_dir_name", "map")

        # ------------------------------------------------------------
        # ROS topic parameters
        # ------------------------------------------------------------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("manipulator_task_cmd_topic", "/manipulator_task_cmd")
        self.declare_parameter("manipulator_task_result_topic", "/manipulator_task_result")
        self.declare_parameter("manipulator_task_state_topic", "/manipulator_task_state")

        # ------------------------------------------------------------
        # Manipulator parameters
        # ------------------------------------------------------------
        self.declare_parameter("inside_button_task_cmd", "INSIDE_BTN_FRONT")
        self.declare_parameter("inside_button_expected_result", "INSIDE_BTN_DONE")
        self.declare_parameter("destination_task_cmd", "DESTINATION_UNLOAD")
        self.declare_parameter("destination_expected_result", "UNLOAD_DONE")
        self.declare_parameter("manipulator_cmd_publish_count", 10)
        self.declare_parameter("manipulator_cmd_publish_interval_sec", 0.2)
        self.declare_parameter("manipulator_cmd_republish_interval_sec", 1.0)
        # 0.0이면 result를 받을 때까지 무한 대기
        self.declare_parameter("manipulator_task_timeout_sec", 0.0)

        # ------------------------------------------------------------
        # Nav2 action parameters
        # ------------------------------------------------------------
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter("nav_server_wait_sec", 2.0)
        # 0이면 성공할 때까지 계속 재시도
        self.declare_parameter("nav_max_retries", 0)
        # 0.0이면 action timeout으로 cancel하지 않음
        self.declare_parameter("nav_goal_timeout_sec", 0.0)
        self.declare_parameter("clear_costmap_before_each_goal", True)
        self.declare_parameter("clear_costmap_after_each_goal", True)
        self.declare_parameter("retry_sleep_sec", 0.5)
        self.declare_parameter("publish_initial_pose_on_start", True)

        # ------------------------------------------------------------
        # Final approach parameters
        # ------------------------------------------------------------
        # 이 반경 안에 들어오면 도착 성공 처리하지 않고, cost 회피 성향만 낮춘 final-approach 모드로 전환한다.
        self.declare_parameter("final_approach_enable", True)
        self.declare_parameter("final_approach_start_radius", 0.70)
        self.declare_parameter("final_approach_restore_after_goal", True)
        self.declare_parameter("final_clear_costmap_on_enter", True)

        # 실제 Nav2 노드 이름. 너의 all_in_one_launch.py/nav2_params.yaml에서 이름이 다르면 여기만 바꾸면 된다.
        self.declare_parameter("controller_server_node", "/controller_server")
        self.declare_parameter("local_costmap_node", "/local_costmap/local_costmap")
        self.declare_parameter("global_costmap_node", "/global_costmap/global_costmap")

        # DWB가 costmap 위 trajectory를 꺼리는 정도를 낮춘다.
        # Nav2 YAML에서 FollowPath 플러그인 이름을 바꾸지 않았다면 보통 이 이름이 맞다.
        self.declare_parameter("final_dwb_base_obstacle_scale_param", "FollowPath.BaseObstacle.scale")
        self.declare_parameter("final_dwb_base_obstacle_scale", 0.0)

        # Inflation cost를 끈다. 실제 장애물 cell은 그대로 남기고 inflated cost만 낮추는 목적이다.
        # 너의 costmap plugin 이름이 inflation_layer가 아니면 이 파라미터 이름을 바꿔야 한다.
        self.declare_parameter("final_local_inflation_enabled_param", "inflation_layer.enabled")
        self.declare_parameter("final_global_inflation_enabled_param", "inflation_layer.enabled")
        self.declare_parameter("final_disable_local_inflation", True)
        self.declare_parameter("final_disable_global_inflation", True)

        # 기본은 obstacle/static layer를 끄지 않는다.
        # inflation 영역이 아니라 실제 obstacle layer cost 때문에 못 들어가는 경우에만 True로 바꾼다.
        self.declare_parameter("final_local_obstacle_enabled_param", "obstacle_layer.enabled")
        self.declare_parameter("final_global_obstacle_enabled_param", "obstacle_layer.enabled")
        self.declare_parameter("final_disable_local_obstacle", False)
        self.declare_parameter("final_disable_global_obstacle", False)

        self.declare_parameter("final_local_static_enabled_param", "static_layer.enabled")
        self.declare_parameter("final_global_static_enabled_param", "static_layer.enabled")
        self.declare_parameter("final_disable_local_static", False)
        self.declare_parameter("final_disable_global_static", False)

        # ------------------------------------------------------------
        # Sound parameters
        # ------------------------------------------------------------
        default_sound_path = self._default_sound_path()
        self.declare_parameter("sound_enabled", True)
        self.declare_parameter("sound_path", default_sound_path)
        self.declare_parameter("starting_bgm_sound", "starting_bgm.mp3")
        self.declare_parameter("robot_for_move_sound", "robot_for_move.mp3")
        self.declare_parameter("btn_clk_start_sound", "btn_clk_start.mp3")
        self.declare_parameter("destination_sound", "destination.mp3")
        self.declare_parameter("recover_sound", "recover.mp3")

        # ------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------
        self.waypoint_file_param = str(self.get_parameter("waypoint_file").value)
        self.map_key = str(self.get_parameter("map_key").value)
        self.package_name = str(self.get_parameter("package_name").value)
        self.config_dir_name = str(self.get_parameter("config_dir_name").value)
        self.map_dir_name = str(self.get_parameter("map_dir_name").value)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.initial_pose_topic = str(self.get_parameter("initial_pose_topic").value)
        self.amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self.manipulator_task_cmd_topic = str(self.get_parameter("manipulator_task_cmd_topic").value)
        self.manipulator_task_result_topic = str(self.get_parameter("manipulator_task_result_topic").value)
        self.manipulator_task_state_topic = str(self.get_parameter("manipulator_task_state_topic").value)

        self.inside_button_task_cmd = str(self.get_parameter("inside_button_task_cmd").value)
        self.inside_button_expected_result = str(self.get_parameter("inside_button_expected_result").value)
        self.destination_task_cmd = str(self.get_parameter("destination_task_cmd").value)
        self.destination_expected_result = str(self.get_parameter("destination_expected_result").value)
        self.manipulator_cmd_publish_count = max(1, int(self.get_parameter("manipulator_cmd_publish_count").value))
        self.manipulator_cmd_publish_interval_sec = max(0.01, float(self.get_parameter("manipulator_cmd_publish_interval_sec").value))
        self.manipulator_cmd_republish_interval_sec = max(0.1, float(self.get_parameter("manipulator_cmd_republish_interval_sec").value))
        self.manipulator_task_timeout_sec = max(0.0, float(self.get_parameter("manipulator_task_timeout_sec").value))

        self.nav_action_name = str(self.get_parameter("nav_action_name").value)
        self.nav_server_wait_sec = max(0.1, float(self.get_parameter("nav_server_wait_sec").value))
        self.nav_max_retries = max(0, int(self.get_parameter("nav_max_retries").value))
        self.nav_goal_timeout_sec = max(0.0, float(self.get_parameter("nav_goal_timeout_sec").value))
        self.clear_costmap_before_each_goal = bool(self.get_parameter("clear_costmap_before_each_goal").value)
        self.clear_costmap_after_each_goal = bool(self.get_parameter("clear_costmap_after_each_goal").value)
        self.retry_sleep_sec = max(0.0, float(self.get_parameter("retry_sleep_sec").value))
        self.publish_initial_pose_on_start = bool(self.get_parameter("publish_initial_pose_on_start").value)

        self.final_approach_enable = bool(self.get_parameter("final_approach_enable").value)
        self.final_approach_start_radius = max(0.05, float(self.get_parameter("final_approach_start_radius").value))
        self.final_approach_restore_after_goal = bool(self.get_parameter("final_approach_restore_after_goal").value)
        self.final_clear_costmap_on_enter = bool(self.get_parameter("final_clear_costmap_on_enter").value)
        self.controller_server_node = str(self.get_parameter("controller_server_node").value)
        self.local_costmap_node = str(self.get_parameter("local_costmap_node").value)
        self.global_costmap_node = str(self.get_parameter("global_costmap_node").value)
        self.final_dwb_base_obstacle_scale_param = str(self.get_parameter("final_dwb_base_obstacle_scale_param").value)
        self.final_dwb_base_obstacle_scale = float(self.get_parameter("final_dwb_base_obstacle_scale").value)
        self.final_local_inflation_enabled_param = str(self.get_parameter("final_local_inflation_enabled_param").value)
        self.final_global_inflation_enabled_param = str(self.get_parameter("final_global_inflation_enabled_param").value)
        self.final_disable_local_inflation = bool(self.get_parameter("final_disable_local_inflation").value)
        self.final_disable_global_inflation = bool(self.get_parameter("final_disable_global_inflation").value)
        self.final_local_obstacle_enabled_param = str(self.get_parameter("final_local_obstacle_enabled_param").value)
        self.final_global_obstacle_enabled_param = str(self.get_parameter("final_global_obstacle_enabled_param").value)
        self.final_disable_local_obstacle = bool(self.get_parameter("final_disable_local_obstacle").value)
        self.final_disable_global_obstacle = bool(self.get_parameter("final_disable_global_obstacle").value)
        self.final_local_static_enabled_param = str(self.get_parameter("final_local_static_enabled_param").value)
        self.final_global_static_enabled_param = str(self.get_parameter("final_global_static_enabled_param").value)
        self.final_disable_local_static = bool(self.get_parameter("final_disable_local_static").value)
        self.final_disable_global_static = bool(self.get_parameter("final_disable_global_static").value)

        self.sound_enabled = bool(self.get_parameter("sound_enabled").value)
        self.sound_path = str(self.get_parameter("sound_path").value)
        self.starting_bgm_sound = str(self.get_parameter("starting_bgm_sound").value)
        self.robot_for_move_sound = str(self.get_parameter("robot_for_move_sound").value)
        self.btn_clk_start_sound = str(self.get_parameter("btn_clk_start_sound").value)
        self.destination_sound = str(self.get_parameter("destination_sound").value)
        self.recover_sound = str(self.get_parameter("recover_sound").value)

        # ------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------
        self.current_pose = None
        self.latest_manipulator_result: Optional[str] = None
        self.latest_manipulator_state: Optional[str] = None
        self.final_mode_active = False
        self.saved_final_params: Dict[str, Dict[str, Any]] = {}

        # ------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self.amcl_pose_callback, 10)
        self.create_subscription(String, self.manipulator_task_result_topic, self.manipulator_task_result_callback, 10)
        self.create_subscription(String, self.manipulator_task_state_topic, self.manipulator_task_state_callback, 10)

        # ------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, self.initial_pose_topic, 10)

        manipulator_qos = QoSProfile(depth=10)
        manipulator_qos.reliability = ReliabilityPolicy.RELIABLE
        manipulator_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.manipulator_task_cmd_pub = self.create_publisher(String, self.manipulator_task_cmd_topic, manipulator_qos)

        # ------------------------------------------------------------
        # Clients / action
        # ------------------------------------------------------------
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)
        self.load_map_client = self.create_client(LoadMap, "/map_server/load_map")
        self.clear_global_costmap_client = self.create_client(ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")
        self.clear_local_costmap_client = self.create_client(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")

        self.controller_get_param_client = self.create_client(
            GetParameters, self._remote_param_service_name(self.controller_server_node, "get_parameters")
        )
        self.controller_set_param_client = self.create_client(
            SetParameters, self._remote_param_service_name(self.controller_server_node, "set_parameters")
        )
        self.local_costmap_get_param_client = self.create_client(
            GetParameters, self._remote_param_service_name(self.local_costmap_node, "get_parameters")
        )
        self.local_costmap_set_param_client = self.create_client(
            SetParameters, self._remote_param_service_name(self.local_costmap_node, "set_parameters")
        )
        self.global_costmap_get_param_client = self.create_client(
            GetParameters, self._remote_param_service_name(self.global_costmap_node, "get_parameters")
        )
        self.global_costmap_set_param_client = self.create_client(
            SetParameters, self._remote_param_service_name(self.global_costmap_node, "set_parameters")
        )

        # ------------------------------------------------------------
        # Speaker
        # ------------------------------------------------------------
        self.speaker = LocalMp3Speaker(node=self, sound_path=self.sound_path, enabled=self.sound_enabled)

        # ------------------------------------------------------------
        # Load waypoint YAML
        # ------------------------------------------------------------
        self.waypoint_file = self.resolve_waypoint_file(self.waypoint_file_param)
        with open(self.waypoint_file, "r", encoding="utf-8") as f:
            self.wp = yaml.safe_load(f)

        self.get_logger().info(
            "IndoorStudentsManager ready. "
            f"waypoint_file={self.waypoint_file}, map_key={self.map_key}, "
            f"nav_action_name={self.nav_action_name}, cmd_vel_topic={self.cmd_vel_topic}, "
            f"final_approach_enable={self.final_approach_enable}, "
            f"final_approach_start_radius={self.final_approach_start_radius:.3f}, "
            f"controller_server_node={self.controller_server_node}, "
            f"local_costmap_node={self.local_costmap_node}, global_costmap_node={self.global_costmap_node}, "
            f"task_cmd_topic={self.manipulator_task_cmd_topic}, task_result_topic={self.manipulator_task_result_topic}"
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
        config_dir = os.path.join(get_package_share_directory(self.package_name), self.config_dir_name)
        return os.path.join(config_dir, "waypoints_indoor.yaml")

    def resolve_map_yaml_path(self, map_yaml: str) -> str:
        if os.path.isabs(map_yaml):
            return map_yaml
        return os.path.join(get_package_share_directory(self.package_name), self.map_dir_name, map_yaml)

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
        self.get_logger().info(f"Received manipulator result: data='{self.latest_manipulator_result}'")

    def manipulator_task_state_callback(self, msg: String):
        self.latest_manipulator_state = str(msg.data).strip()
        self.get_logger().info(f"Received manipulator state: data='{self.latest_manipulator_state}'")

    # ------------------------------------------------------------------
    # YAML / map helpers
    # ------------------------------------------------------------------
    def get_indoor_info(self) -> Dict[str, Any]:
        maps = self.wp.get("maps", {})
        if self.map_key not in maps:
            raise KeyError(f"waypoints YAML에 maps.{self.map_key}가 없습니다. 현재 maps 키={list(maps.keys())}")
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
        else:
            self.get_logger().warn("Global costmap clear service not available.")

        if self.clear_local_costmap_client.wait_for_service(timeout_sec=0.5):
            future = self.clear_local_costmap_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().info("Requested local costmap clear.")
        else:
            self.get_logger().warn("Local costmap clear service not available.")

    # ------------------------------------------------------------------
    # Pose helpers
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
            "Publish initial pose: "
            f"x={float(pose_dict['x']):.3f}, y={float(pose_dict['y']):.3f}, yaw={float(pose_dict['yaw']):.4f}"
        )
        for _ in range(20):
            self.initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def distance_to_pose(self, pose_dict: PoseDict) -> Optional[float]:
        if self.current_pose is None:
            return None
        dx = self.current_pose.position.x - float(pose_dict["x"])
        dy = self.current_pose.position.y - float(pose_dict["y"])
        return math.hypot(dx, dy)

    def yaw_error_to_pose(self, pose_dict: PoseDict) -> Optional[float]:
        if self.current_pose is None:
            return None
        current_yaw = quaternion_to_yaw(self.current_pose.orientation)
        target_yaw = float(pose_dict["yaw"])
        return normalize_angle(target_yaw - current_yaw)

    # ------------------------------------------------------------------
    # Remote parameter helpers for final approach
    # ------------------------------------------------------------------
    def _remote_param_service_name(self, node_name: str, service_name: str) -> str:
        """
        node_name='/controller_server', service_name='set_parameters'
        -> '/controller_server/set_parameters'
        """
        base = str(node_name).strip()
        if not base.startswith("/"):
            base = "/" + base
        return f"{base}/{service_name}"

    def _parameter_value_to_python(self, value_msg: ParameterValue) -> Any:
        t = int(value_msg.type)
        if t == ParameterType.PARAMETER_BOOL:
            return bool(value_msg.bool_value)
        if t == ParameterType.PARAMETER_INTEGER:
            return int(value_msg.integer_value)
        if t == ParameterType.PARAMETER_DOUBLE:
            return float(value_msg.double_value)
        if t == ParameterType.PARAMETER_STRING:
            return str(value_msg.string_value)
        if t == ParameterType.PARAMETER_BYTE_ARRAY:
            return list(value_msg.byte_array_value)
        if t == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(value_msg.bool_array_value)
        if t == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(value_msg.integer_array_value)
        if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(value_msg.double_array_value)
        if t == ParameterType.PARAMETER_STRING_ARRAY:
            return list(value_msg.string_array_value)
        return None

    def _python_to_parameter_value(self, value: Any) -> ParameterValue:
        pv = ParameterValue()

        # bool은 int의 subclass라서 반드시 int보다 먼저 검사해야 한다.
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = bool(value)
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        elif isinstance(value, str):
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = str(value)
        elif isinstance(value, list):
            if all(isinstance(v, bool) for v in value):
                pv.type = ParameterType.PARAMETER_BOOL_ARRAY
                pv.bool_array_value = list(value)
            elif all(isinstance(v, int) and not isinstance(v, bool) for v in value):
                pv.type = ParameterType.PARAMETER_INTEGER_ARRAY
                pv.integer_array_value = list(value)
            elif all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
                pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                pv.double_array_value = [float(v) for v in value]
            else:
                pv.type = ParameterType.PARAMETER_STRING_ARRAY
                pv.string_array_value = [str(v) for v in value]
        else:
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = str(value)

        return pv

    def _wait_service(self, client, service_label: str, timeout_sec: float = 0.5) -> bool:
        try:
            return bool(client.wait_for_service(timeout_sec=timeout_sec))
        except Exception as e:
            self.get_logger().warn(f"Service wait failed for {service_label}: {e}")
            return False

    def _get_remote_params(
        self,
        get_client,
        node_name: str,
        param_names: List[str],
    ) -> Dict[str, Any]:
        result_values: Dict[str, Any] = {}
        names = [p for p in param_names if p]
        if not names:
            return result_values

        service_label = self._remote_param_service_name(node_name, "get_parameters")
        if not self._wait_service(get_client, service_label):
            self.get_logger().warn(f"Parameter get service not available: {service_label}")
            return result_values

        req = GetParameters.Request()
        req.names = names

        try:
            future = get_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            result = future.result()
            if result is None:
                self.get_logger().warn(f"Get parameters returned None from {node_name}: {names}")
                return result_values

            for name, value_msg in zip(names, result.values):
                py_value = self._parameter_value_to_python(value_msg)
                if py_value is not None:
                    result_values[name] = py_value

        except Exception as e:
            self.get_logger().warn(f"Get parameters failed from {node_name}: names={names}, error={e}")

        return result_values

    def _set_remote_params(
        self,
        set_client,
        node_name: str,
        params: Dict[str, Any],
    ) -> bool:
        if not params:
            return True

        service_label = self._remote_param_service_name(node_name, "set_parameters")
        if not self._wait_service(set_client, service_label):
            self.get_logger().warn(f"Parameter set service not available: {service_label}")
            return False

        req = SetParameters.Request()
        for name, value in params.items():
            if value is None or name == "":
                continue
            param = ParameterMsg()
            param.name = str(name)
            param.value = self._python_to_parameter_value(value)
            req.parameters.append(param)

        if not req.parameters:
            return True

        try:
            future = set_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            result = future.result()
            if result is None:
                self.get_logger().warn(f"Set parameters returned None from {node_name}: {params}")
                return False

            ok = True
            for set_result, param in zip(result.results, req.parameters):
                if not set_result.successful:
                    ok = False
                    self.get_logger().warn(
                        f"Set parameter failed on {node_name}: {param.name}, reason={set_result.reason}"
                    )
            if ok:
                self.get_logger().info(f"Set parameters on {node_name}: {params}")
            return ok

        except Exception as e:
            self.get_logger().warn(f"Set parameters failed on {node_name}: params={params}, error={e}")
            return False

    def enter_final_approach_mode(self, name: str):
        if not self.final_approach_enable or self.final_mode_active:
            return

        self.get_logger().warn(
            f"{name}: ENTER FINAL-APPROACH MODE. "
            "Nav2 goal은 cancel하지 않고 유지한 채, cost 회피 성향만 낮춘다."
        )

        self.saved_final_params = {
            self.controller_server_node: self._get_remote_params(
                self.controller_get_param_client,
                self.controller_server_node,
                [self.final_dwb_base_obstacle_scale_param],
            ),
            self.local_costmap_node: self._get_remote_params(
                self.local_costmap_get_param_client,
                self.local_costmap_node,
                [
                    self.final_local_inflation_enabled_param,
                    self.final_local_obstacle_enabled_param,
                    self.final_local_static_enabled_param,
                ],
            ),
            self.global_costmap_node: self._get_remote_params(
                self.global_costmap_get_param_client,
                self.global_costmap_node,
                [
                    self.final_global_inflation_enabled_param,
                    self.final_global_obstacle_enabled_param,
                    self.final_global_static_enabled_param,
                ],
            ),
        }

        controller_updates: Dict[str, Any] = {}
        if self.final_dwb_base_obstacle_scale_param:
            controller_updates[self.final_dwb_base_obstacle_scale_param] = self.final_dwb_base_obstacle_scale

        local_updates: Dict[str, Any] = {}
        if self.final_disable_local_inflation and self.final_local_inflation_enabled_param:
            local_updates[self.final_local_inflation_enabled_param] = False
        if self.final_disable_local_obstacle and self.final_local_obstacle_enabled_param:
            local_updates[self.final_local_obstacle_enabled_param] = False
        if self.final_disable_local_static and self.final_local_static_enabled_param:
            local_updates[self.final_local_static_enabled_param] = False

        global_updates: Dict[str, Any] = {}
        if self.final_disable_global_inflation and self.final_global_inflation_enabled_param:
            global_updates[self.final_global_inflation_enabled_param] = False
        if self.final_disable_global_obstacle and self.final_global_obstacle_enabled_param:
            global_updates[self.final_global_obstacle_enabled_param] = False
        if self.final_disable_global_static and self.final_global_static_enabled_param:
            global_updates[self.final_global_static_enabled_param] = False

        self._set_remote_params(self.controller_set_param_client, self.controller_server_node, controller_updates)
        self._set_remote_params(self.local_costmap_set_param_client, self.local_costmap_node, local_updates)
        self._set_remote_params(self.global_costmap_set_param_client, self.global_costmap_node, global_updates)

        self.final_mode_active = True
        if self.final_clear_costmap_on_enter:
            self.clear_costmaps()
            self.spin_sleep(0.2)

    def exit_final_approach_mode(self, name: str):
        if not self.final_mode_active:
            return

        if not self.final_approach_restore_after_goal:
            self.get_logger().warn(
                f"{name}: final-approach mode remains active because final_approach_restore_after_goal=False"
            )
            return

        self.get_logger().info(f"{name}: EXIT FINAL-APPROACH MODE. Restore Nav2 parameters.")
        saved_controller = self.saved_final_params.get(self.controller_server_node, {})
        saved_local = self.saved_final_params.get(self.local_costmap_node, {})
        saved_global = self.saved_final_params.get(self.global_costmap_node, {})

        self._set_remote_params(self.controller_set_param_client, self.controller_server_node, saved_controller)
        self._set_remote_params(self.local_costmap_set_param_client, self.local_costmap_node, saved_local)
        self._set_remote_params(self.global_costmap_set_param_client, self.global_costmap_node, saved_global)

        self.final_mode_active = False
        self.saved_final_params = {}
        self.clear_costmaps()
        self.spin_sleep(0.2)

    # ------------------------------------------------------------------
    # Nav2 navigation
    # ------------------------------------------------------------------
    def stop_robot(self, repeat: int = 10):
        msg = Twist()
        for _ in range(repeat):
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def cancel_goal_and_wait(self, goal_handle, label: str):
        try:
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            self.get_logger().info(f"{label}: cancel requested.")
        except Exception as e:
            self.get_logger().warn(f"{label}: cancel failed: {e}")
        self.stop_robot(repeat=5)

    def go_to_pose_nav2(self, pose_dict: PoseDict, name: str) -> bool:
        """
        NavigateToPose로 이동한다.
        cmd_vel open-loop 이동은 사용하지 않는다.

        목표 근처에서 cost 때문에 주춤거리면:
          - goal cancel / 성공 처리 X
          - final-approach mode로 전환
          - Nav2 action은 계속 유지
          - 실제 STATUS_SUCCEEDED를 기다림
        """
        attempt = 1

        while rclpy.ok():
            self.exit_final_approach_mode(f"{name}: before attempt")

            if self.nav_max_retries > 0 and attempt > self.nav_max_retries:
                self.get_logger().error(f"{name}: exceeded nav_max_retries={self.nav_max_retries}.")
                return False

            self.get_logger().info(f"{name}: Nav2 attempt {attempt}")

            if self.clear_costmap_before_each_goal:
                self.clear_costmaps()
                self.spin_sleep(0.2)

            if not self.nav_client.wait_for_server(timeout_sec=self.nav_server_wait_sec):
                self.get_logger().error(f"{name}: Nav2 action server '{self.nav_action_name}' not available.")
                self.spin_sleep(self.retry_sleep_sec)
                attempt += 1
                continue

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.make_pose(pose_dict)

            self.get_logger().info(
                f"{name}: send NavigateToPose goal "
                f"x={float(pose_dict['x']):.3f}, y={float(pose_dict['y']):.3f}, yaw={float(pose_dict['yaw']):.4f}"
            )

            send_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()

            if goal_handle is None:
                self.get_logger().error(f"{name}: goal_handle is None.")
                self.spin_sleep(self.retry_sleep_sec)
                attempt += 1
                continue

            if not goal_handle.accepted:
                self.get_logger().error(f"{name}: goal rejected. Clear costmaps and retry.")
                self.clear_costmaps()
                self.spin_sleep(self.retry_sleep_sec)
                attempt += 1
                continue

            self.get_logger().info(f"{name}: goal accepted.")
            result_future = goal_handle.get_result_async()
            start_time = time.time()
            last_log_time = 0.0

            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                now = time.time()
                dist = self.distance_to_pose(pose_dict)
                yaw_error = self.yaw_error_to_pose(pose_dict)

                if (
                    self.final_approach_enable
                    and not self.final_mode_active
                    and dist is not None
                    and dist <= self.final_approach_start_radius
                ):
                    self.enter_final_approach_mode(name)

                if self.nav_goal_timeout_sec > 0.0 and now - start_time > self.nav_goal_timeout_sec:
                    self.get_logger().warn(
                        f"{name}: nav_goal_timeout_sec reached. elapsed={now - start_time:.1f}s, "
                        f"dist={dist}, yaw_error={yaw_error}. Retry."
                    )
                    self.cancel_goal_and_wait(goal_handle, name)
                    break

                if now - last_log_time >= 2.0:
                    self.get_logger().info(
                        f"{name}: navigating... "
                        f"dist={None if dist is None else round(dist, 3)}, "
                        f"yaw_error={None if yaw_error is None else round(yaw_error, 3)}, "
                        f"final_mode_active={self.final_mode_active}, "
                        f"final_radius={self.final_approach_start_radius:.3f}"
                    )
                    last_log_time = now

            if not result_future.done():
                self.exit_final_approach_mode(f"{name}: retry after timeout")
                self.clear_costmaps()
                self.spin_sleep(self.retry_sleep_sec)
                attempt += 1
                continue

            wrapped_result = result_future.result()
            if wrapped_result is None:
                self.get_logger().error(f"{name}: result is None. Retry.")
                self.exit_final_approach_mode(f"{name}: result None")
                self.clear_costmaps()
                self.spin_sleep(self.retry_sleep_sec)
                attempt += 1
                continue

            status = wrapped_result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"{name}: Nav2 succeeded at actual waypoint goal.")
                self.stop_robot(repeat=5)
                self.exit_final_approach_mode(f"{name}: success")
                if self.clear_costmap_after_each_goal:
                    self.clear_costmaps()
                return True

            self.get_logger().error(
                f"{name}: Nav2 failed with status={status}. "
                "도착으로 인정하지 않고, cost parameter 조정 후 계속 재시도한다."
            )
            self.exit_final_approach_mode(f"{name}: failed attempt")
            self.clear_costmaps()
            self.spin_sleep(self.retry_sleep_sec)
            attempt += 1

        return False

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
                f"Publish {self.manipulator_task_cmd_topic}: data='{msg.data}' ({idx + 1}/{repeat})"
            )
            if idx < repeat - 1 and interval_sec > 0.0:
                self.spin_sleep(interval_sec)

    def is_task_result_match(self, received: Optional[str], expected_result: str, task_cmd: str) -> bool:
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

    def send_manipulator_task_and_wait(self, task_cmd: str, expected_result: str) -> bool:
        self.latest_manipulator_result = None
        self.get_logger().info(
            "Manipulator handshake start: "
            f"publish String(data='{task_cmd}') to {self.manipulator_task_cmd_topic}, "
            f"wait String(data='{expected_result}') from {self.manipulator_task_result_topic}"
        )

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

            if self.is_task_result_match(self.latest_manipulator_result, expected_result, task_cmd):
                self.get_logger().info(f"Manipulator task done. received='{self.latest_manipulator_result}'")
                self.publish_task_command("", repeat=3, interval_sec=0.05)
                return True

            if self.manipulator_task_timeout_sec > 0.0 and now - start_time > self.manipulator_task_timeout_sec:
                self.get_logger().error(
                    "Timeout waiting manipulator task result. "
                    f"expected='{expected_result}', latest='{self.latest_manipulator_result}', "
                    f"timeout={self.manipulator_task_timeout_sec:.1f}s"
                )
                self.publish_task_command("", repeat=3, interval_sec=0.05)
                return False

            if now - last_republish_time >= self.manipulator_cmd_republish_interval_sec:
                self.publish_task_command(task_cmd, repeat=1, interval_sec=0.0)
                last_republish_time = now

            if now - last_log_time >= 1.0:
                elapsed = now - start_time
                self.get_logger().info(
                    "Waiting manipulator result: "
                    f"expected='{expected_result}', latest_result='{self.latest_manipulator_result}', "
                    f"latest_state='{self.latest_manipulator_state}', elapsed={elapsed:.1f}s"
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

        if self.publish_initial_pose_on_start:
            self.publish_initial_pose(start_pose)
            self.spin_sleep(1.0)
            self.clear_costmaps()

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

        cycle = 1
        self.get_logger().info(
            "Indoor Nav2 mission loop started. Ctrl+C로 종료. "
            "Route: start -> elevator_btn_front -> unload_spot -> start -> ..."
        )

        while rclpy.ok():
            self.get_logger().info(f"========== cycle {cycle} start ==========")

            if not self.go_to_pose_nav2(elevator_btn_front_pose, f"cycle {cycle}: start -> elevator_btn_front"):
                self.get_logger().error("Failed to navigate to elevator_btn_front.")
                break

            self.speaker.play_once(
                self.btn_clk_start_sound,
                once_key=f"cycle_{cycle}_btn_clk_start",
                wait_if_busy=True,
            )

            if not self.send_manipulator_task_and_wait(
                task_cmd=self.inside_button_task_cmd,
                expected_result=self.inside_button_expected_result,
            ):
                self.get_logger().error("Failed while waiting INSIDE_BTN_DONE.")
                break

            if not self.go_to_pose_nav2(unload_spot_pose, f"cycle {cycle}: elevator_btn_front -> unload_spot"):
                self.get_logger().error("Failed to navigate to unload_spot.")
                break

            self.speaker.play_once(
                self.destination_sound,
                once_key=f"cycle_{cycle}_destination",
                wait_if_busy=True,
            )

            if not self.send_manipulator_task_and_wait(
                task_cmd=self.destination_task_cmd,
                expected_result=self.destination_expected_result,
            ):
                self.get_logger().error("Failed while waiting UNLOAD_DONE.")
                break

            self.speaker.play_once(
                self.recover_sound,
                once_key=f"cycle_{cycle}_recover",
                wait_if_busy=True,
            )

            if not self.go_to_pose_nav2(start_pose, f"cycle {cycle}: unload_spot -> start"):
                self.get_logger().error("Failed to navigate to start.")
                break

            self.get_logger().info(f"========== cycle {cycle} complete ==========")
            cycle += 1

        self.get_logger().info("Indoor Nav2 mission loop ended.")


def main(args=None):
    rclpy.init(args=args)
    node = IndoorStudentsManager()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Stop indoor mission.")
    finally:
        node.exit_final_approach_mode("shutdown")
        node.stop_robot(repeat=10)
        node.speaker.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()