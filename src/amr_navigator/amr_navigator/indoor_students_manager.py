#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import math
import os
import time
from typing import Any, Dict, Optional

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
        self.declare_parameter("require_manipulator_active_state_before_result", True)
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
        self.require_manipulator_active_state_before_result = bool(
            self.get_parameter("require_manipulator_active_state_before_result").value
        )
        self.manipulator_task_timeout_sec = max(0.0, float(self.get_parameter("manipulator_task_timeout_sec").value))

        self.nav_action_name = str(self.get_parameter("nav_action_name").value)
        self.nav_server_wait_sec = max(0.1, float(self.get_parameter("nav_server_wait_sec").value))
        self.nav_max_retries = max(0, int(self.get_parameter("nav_max_retries").value))
        self.nav_goal_timeout_sec = max(0.0, float(self.get_parameter("nav_goal_timeout_sec").value))
        self.clear_costmap_before_each_goal = bool(self.get_parameter("clear_costmap_before_each_goal").value)
        self.clear_costmap_after_each_goal = bool(self.get_parameter("clear_costmap_after_each_goal").value)
        self.retry_sleep_sec = max(0.0, float(self.get_parameter("retry_sleep_sec").value))
        self.publish_initial_pose_on_start = bool(self.get_parameter("publish_initial_pose_on_start").value)

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
        self.latest_manipulator_result_time: Optional[float] = None
        self.latest_manipulator_state_time: Optional[float] = None

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

        # Command topics must be volatile. A latched command can replay an old
        # manipulator action when a node restarts or reconnects.
        manipulator_qos = QoSProfile(depth=10)
        manipulator_qos.reliability = ReliabilityPolicy.RELIABLE
        manipulator_qos.durability = DurabilityPolicy.VOLATILE
        self.manipulator_task_cmd_pub = self.create_publisher(String, self.manipulator_task_cmd_topic, manipulator_qos)

        # ------------------------------------------------------------
        # Clients / action
        # ------------------------------------------------------------
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)
        self.load_map_client = self.create_client(LoadMap, "/map_server/load_map")
        self.clear_global_costmap_client = self.create_client(ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")
        self.clear_local_costmap_client = self.create_client(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")

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
            f"task_cmd_topic={self.manipulator_task_cmd_topic}, "
            f"task_result_topic={self.manipulator_task_result_topic}, "
            f"task_state_topic={self.manipulator_task_state_topic}, "
            f"inside_button=({self.inside_button_task_cmd}->{self.inside_button_expected_result}), "
            f"destination=({self.destination_task_cmd}->{self.destination_expected_result}), "
            f"cmd_publish_count={self.manipulator_cmd_publish_count}, "
            f"cmd_republish_interval={self.manipulator_cmd_republish_interval_sec:.2f}s, "
            f"require_active_state={self.require_manipulator_active_state_before_result}, "
            f"manipulator_task_timeout={self.manipulator_task_timeout_sec:.1f}s"
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
        self.latest_manipulator_result_time = time.time()
        self.get_logger().info(f"Received manipulator result: data='{self.latest_manipulator_result}'")

    def manipulator_task_state_callback(self, msg: String):
        self.latest_manipulator_state = str(msg.data).strip()
        self.latest_manipulator_state_time = time.time()
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
        """
        attempt = 1

        while rclpy.ok():
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
                        f"yaw_error={None if yaw_error is None else round(yaw_error, 3)}"
                    )
                    last_log_time = now

            if not result_future.done():
                self.clear_costmaps()
                self.spin_sleep(self.retry_sleep_sec)
                attempt += 1
                continue

            wrapped_result = result_future.result()
            if wrapped_result is None:
                self.get_logger().error(f"{name}: result is None. Retry.")
                self.clear_costmaps()
                self.spin_sleep(self.retry_sleep_sec)
                attempt += 1
                continue

            status = wrapped_result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"{name}: Nav2 succeeded at actual waypoint goal.")
                self.stop_robot(repeat=5)
                if self.clear_costmap_after_each_goal:
                    self.clear_costmaps()
                return True

            self.get_logger().error(
                f"{name}: Nav2 failed with status={status}. "
                "도착으로 인정하지 않고 계속 재시도한다."
            )
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
        if not data:
            return False
        return data == expected

    def expected_active_states_for_task(self, task_cmd: str) -> set:
        cmd = str(task_cmd).strip().upper()
        if cmd == str(self.inside_button_task_cmd).strip().upper():
            return {
                "INSIDE_ALIGNING",
                "INSIDE_MARKER_SETTLE",
                "BUTTON_PREPRESSING",
                "BUTTON_HOMING",
            }
        if cmd == str(self.destination_task_cmd).strip().upper():
            return {
                "UNLOAD_PREPARE",
                "UNLOAD_EXECUTE",
            }
        return set()

    def reset_manipulator_handshake_state(self):
        self.latest_manipulator_result = None
        self.latest_manipulator_result_time = None
        self.latest_manipulator_state = None
        self.latest_manipulator_state_time = None

    def received_after(self, stamp: Optional[float], start_time: float) -> bool:
        return stamp is not None and stamp >= start_time

    def send_manipulator_task_and_wait(self, task_cmd: str, expected_result: str) -> bool:
        self.reset_manipulator_handshake_state()
        self.get_logger().info(
            "Manipulator handshake start: "
            f"publish String(data='{task_cmd}') to {self.manipulator_task_cmd_topic}, "
            f"wait String(data='{expected_result}') from {self.manipulator_task_result_topic}"
        )

        start_time = time.time()
        last_publish_time = 0.0
        last_log_time = 0.0
        publish_attempts = 0
        observed_active_state = False
        active_states = self.expected_active_states_for_task(task_cmd)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()

            if (
                self.latest_manipulator_state in active_states
                and self.received_after(self.latest_manipulator_state_time, start_time)
            ):
                observed_active_state = True

            result_is_current = self.received_after(self.latest_manipulator_result_time, start_time)
            result_matches = self.is_task_result_match(
                self.latest_manipulator_result,
                expected_result,
                task_cmd,
            )
            state_gate_ok = (
                observed_active_state
                or not self.require_manipulator_active_state_before_result
                or not active_states
            )

            if result_is_current and result_matches and state_gate_ok:
                self.get_logger().info(f"Manipulator task done. received='{self.latest_manipulator_result}'")
                return True

            if self.manipulator_task_timeout_sec > 0.0 and now - start_time > self.manipulator_task_timeout_sec:
                self.get_logger().error(
                    "Timeout waiting manipulator task result. "
                    f"expected='{expected_result}', latest='{self.latest_manipulator_result}', "
                    f"latest_state='{self.latest_manipulator_state}', "
                    f"observed_active_state={observed_active_state}, "
                    f"timeout={self.manipulator_task_timeout_sec:.1f}s"
                )
                return False

            should_publish = (
                publish_attempts == 0
                or (
                    not observed_active_state
                    and publish_attempts < self.manipulator_cmd_publish_count
                    and now - last_publish_time >= self.manipulator_cmd_republish_interval_sec
                )
            )
            if should_publish:
                self.publish_task_command(task_cmd, repeat=1, interval_sec=0.0)
                last_publish_time = now
                publish_attempts += 1

            if now - last_log_time >= 1.0:
                elapsed = now - start_time
                self.get_logger().info(
                    "Waiting manipulator result: "
                    f"expected='{expected_result}', latest_result='{self.latest_manipulator_result}', "
                    f"latest_state='{self.latest_manipulator_state}', "
                    f"observed_active_state={observed_active_state}, "
                    f"publish_attempts={publish_attempts}/{self.manipulator_cmd_publish_count}, "
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
        node.stop_robot(repeat=10)
        node.speaker.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
