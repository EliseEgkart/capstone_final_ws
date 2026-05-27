#!/usr/bin/env python3
'''
상태 확인
ros2 topic pub /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'status'}" --once
단순 이동 테스트
ros2 topic pub /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'go'}" --once
버튼 누르기 실행
ros2 topic pub /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'press'}" --once
취소
ros2 topic pub /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'cancel'}" --once
마커 초기화
ros2 topic pub /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'clear'}" --once
'''
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from statistics import mean, median
from typing import Deque, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException


@dataclass
class MarkerSample:
    point: PointStamped
    rx_time: Time


@dataclass
class MotionStage:
    name: str
    target: PointStamped
    velocity_scaling: float
    acceleration_scaling: float
    position_tolerance_m: float
    hold_after: bool = False


class MarkerButtonPressCommander(Node):
    """
    Marker 기반 MoveIt2 로봇팔 제어 노드.

    주요 기능:
    - /object_3d_marker 로 들어온 Marker 위치 캐싱
    - go 명령: 마커 위치로 단순 이동
    - press 명령: 버튼 누르기 FSM 실행
      APPROACH -> PRESS -> HOLD -> RETREAT
    - status, clear, cancel 명령 지원
    - result/state topic 발행
    """

    def __init__(self) -> None:
        super().__init__("marker_button_press_commander")

        # =========================================================
        # Topic parameters
        # =========================================================
        self.declare_parameter("marker_topic", "/object_3d_marker")
        self.declare_parameter("cmd_topic", "/marker_button_press_commander/cmd")
        self.declare_parameter("result_topic", "/marker_button_press_commander/result")
        self.declare_parameter("state_topic", "/marker_button_press_commander/state")
        self.declare_parameter("move_action_name", "/move_action")

        # =========================================================
        # MoveIt parameters
        # =========================================================
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("ee_link", "ee_link")
        self.declare_parameter("target_frame", "link1")

        self.declare_parameter("position_tolerance_m", 0.01)
        self.declare_parameter("press_position_tolerance_m", 0.004)

        self.declare_parameter("max_velocity_scaling", 0.2)
        self.declare_parameter("max_acceleration_scaling", 0.2)
        self.declare_parameter("allowed_planning_time_sec", 3.0)
        self.declare_parameter("num_planning_attempts", 5)

        self.declare_parameter("plan_only", False)

        # 4축 로봇팔에서는 orientation constraint가 planning 실패 원인이 될 수 있음.
        # 먼저 위치 기반 접근을 성공시키고, 필요할 때만 true로 켜는 것을 권장.
        self.declare_parameter("use_orientation_constraint", False)

        # =========================================================
        # Simple move offset
        # go 명령에서만 사용하는 단순 이동 보정값
        # =========================================================
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)

        # =========================================================
        # Button press parameters
        # =========================================================
        self.declare_parameter("button_offset_x", 0.0)
        self.declare_parameter("button_offset_y", 0.0)
        self.declare_parameter("button_offset_z", 0.0)

        self.declare_parameter("press_axis", "-x")
        self.declare_parameter("approach_distance_m", 0.06)
        self.declare_parameter("press_depth_m", 0.008)
        self.declare_parameter("retreat_distance_m", 0.06)
        self.declare_parameter("press_hold_sec", 0.3)

        self.declare_parameter("press_intermediate_steps", 1)

        self.declare_parameter("approach_velocity_scaling", 0.15)
        self.declare_parameter("press_velocity_scaling", 0.04)
        self.declare_parameter("retreat_velocity_scaling", 0.15)

        self.declare_parameter("approach_acceleration_scaling", 0.15)
        self.declare_parameter("press_acceleration_scaling", 0.04)
        self.declare_parameter("retreat_acceleration_scaling", 0.15)

        # =========================================================
        # Orientation constraint
        # 버튼 누르기에서는 이 값이 매우 중요함.
        # 기본값은 넓게 두되, 실제 운용에서는 tolerance를 줄이는 것을 권장.
        # =========================================================
        self.declare_parameter("goal_qx", 0.0)
        self.declare_parameter("goal_qy", 0.0)
        self.declare_parameter("goal_qz", 0.0)
        self.declare_parameter("goal_qw", 1.0)

        self.declare_parameter("ori_tol_x", 0.5)
        self.declare_parameter("ori_tol_y", 0.5)
        self.declare_parameter("ori_tol_z", 0.5)

        # =========================================================
        # Marker filtering
        # =========================================================
        self.declare_parameter("marker_timeout_sec", 30.0)
        self.declare_parameter("marker_window_size", 5)
        self.declare_parameter("smoothing_method", "median")

        # =========================================================
        # MoveIt replanning
        # =========================================================
        self.declare_parameter("replan", True)
        self.declare_parameter("replan_attempts", 1)
        self.declare_parameter("replan_delay_sec", 1.0)
        self.declare_parameter("wait_for_server_sec", 2.0)

        # =========================================================
        # Read parameters
        # =========================================================
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.move_action_name = str(self.get_parameter("move_action_name").value)

        self.planning_group = str(self.get_parameter("planning_group").value)
        self.ee_link = str(self.get_parameter("ee_link").value)
        self.target_frame = str(self.get_parameter("target_frame").value)

        self.position_tolerance_m = float(
            self.get_parameter("position_tolerance_m").value
        )
        self.press_position_tolerance_m = float(
            self.get_parameter("press_position_tolerance_m").value
        )

        self.max_velocity_scaling = float(
            self.get_parameter("max_velocity_scaling").value
        )
        self.max_acceleration_scaling = float(
            self.get_parameter("max_acceleration_scaling").value
        )
        self.allowed_planning_time_sec = float(
            self.get_parameter("allowed_planning_time_sec").value
        )
        self.num_planning_attempts = int(
            self.get_parameter("num_planning_attempts").value
        )

        self.plan_only = bool(self.get_parameter("plan_only").value)
        self.use_orientation_constraint = bool(
            self.get_parameter("use_orientation_constraint").value
        )

        self.offset_x = float(self.get_parameter("offset_x").value)
        self.offset_y = float(self.get_parameter("offset_y").value)
        self.offset_z = float(self.get_parameter("offset_z").value)

        self.button_offset_x = float(self.get_parameter("button_offset_x").value)
        self.button_offset_y = float(self.get_parameter("button_offset_y").value)
        self.button_offset_z = float(self.get_parameter("button_offset_z").value)

        self.press_axis = str(self.get_parameter("press_axis").value).strip().lower()
        self.approach_distance_m = float(
            self.get_parameter("approach_distance_m").value
        )
        self.press_depth_m = float(self.get_parameter("press_depth_m").value)
        self.retreat_distance_m = float(
            self.get_parameter("retreat_distance_m").value
        )
        self.press_hold_sec = float(self.get_parameter("press_hold_sec").value)

        self.press_intermediate_steps = int(
            self.get_parameter("press_intermediate_steps").value
        )
        self.press_intermediate_steps = max(1, min(20, self.press_intermediate_steps))

        self.approach_velocity_scaling = float(
            self.get_parameter("approach_velocity_scaling").value
        )
        self.press_velocity_scaling = float(
            self.get_parameter("press_velocity_scaling").value
        )
        self.retreat_velocity_scaling = float(
            self.get_parameter("retreat_velocity_scaling").value
        )

        self.approach_acceleration_scaling = float(
            self.get_parameter("approach_acceleration_scaling").value
        )
        self.press_acceleration_scaling = float(
            self.get_parameter("press_acceleration_scaling").value
        )
        self.retreat_acceleration_scaling = float(
            self.get_parameter("retreat_acceleration_scaling").value
        )

        self.goal_qx = float(self.get_parameter("goal_qx").value)
        self.goal_qy = float(self.get_parameter("goal_qy").value)
        self.goal_qz = float(self.get_parameter("goal_qz").value)
        self.goal_qw = float(self.get_parameter("goal_qw").value)

        self.ori_tol_x = float(self.get_parameter("ori_tol_x").value)
        self.ori_tol_y = float(self.get_parameter("ori_tol_y").value)
        self.ori_tol_z = float(self.get_parameter("ori_tol_z").value)

        self.marker_timeout_sec = float(
            self.get_parameter("marker_timeout_sec").value
        )
        self.marker_window_size = int(self.get_parameter("marker_window_size").value)
        self.marker_window_size = max(1, min(30, self.marker_window_size))

        self.smoothing_method = str(
            self.get_parameter("smoothing_method").value
        ).strip().lower()

        self.replan = bool(self.get_parameter("replan").value)
        self.replan_attempts = int(self.get_parameter("replan_attempts").value)
        self.replan_delay_sec = float(self.get_parameter("replan_delay_sec").value)
        self.wait_for_server_sec = float(
            self.get_parameter("wait_for_server_sec").value
        )

        # =========================================================
        # Runtime state
        # =========================================================
        self._marker_buffer: Deque[MarkerSample] = deque(
            maxlen=self.marker_window_size
        )

        self._active_goal = False
        self._goal_handle = None
        self._cancel_requested = False

        self._operation = "idle"
        self._state = "IDLE"

        self._sequence: List[MotionStage] = []
        self._sequence_index = 0
        self._current_stage: Optional[MotionStage] = None
        self._hold_timer = None

        # =========================================================
        # TF / MoveIt / ROS interfaces
        # =========================================================
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.move_action_client = ActionClient(
            self,
            MoveGroup,
            self.move_action_name,
        )

        self.marker_sub = self.create_subscription(
            Marker,
            self.marker_topic,
            self._marker_cb,
            10,
        )

        self.cmd_sub = self.create_subscription(
            String,
            self.cmd_topic,
            self._cmd_cb,
            10,
        )

        self.result_pub = self.create_publisher(
            String,
            self.result_topic,
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            self.state_topic,
            10,
        )

        # =========================================================
        # Startup log
        # =========================================================
        self.get_logger().info("[button_commander] ready")
        self.get_logger().info(f"[button_commander] marker_topic={self.marker_topic}")
        self.get_logger().info(f"[button_commander] cmd_topic={self.cmd_topic}")
        self.get_logger().info(f"[button_commander] result_topic={self.result_topic}")
        self.get_logger().info(f"[button_commander] state_topic={self.state_topic}")
        self.get_logger().info(
            f"[button_commander] move_action={self.move_action_name}"
        )
        self.get_logger().info(
            "[button_commander] commands: go, press, status, clear, cancel"
        )
        self.get_logger().info(
            f"[button_commander] press_axis={self.press_axis}, "
            f"approach={self.approach_distance_m:.3f} m, "
            f"press_depth={self.press_depth_m:.3f} m, "
            f"retreat={self.retreat_distance_m:.3f} m"
        )
        self.get_logger().info(
            f"[button_commander] use_orientation_constraint="
            f"{self.use_orientation_constraint}, "
            f"ori_tol=({self.ori_tol_x:.3f}, {self.ori_tol_y:.3f}, {self.ori_tol_z:.3f})"
        )

        self._publish_state("IDLE")

    # =============================================================
    # ROS callbacks
    # =============================================================
    def _marker_cb(self, msg: Marker) -> None:
        if msg.action != Marker.ADD:
            return

        if not msg.header.frame_id:
            self.get_logger().warn("[marker] received marker with empty frame_id")
            return

        p = PointStamped()
        p.header = msg.header
        p.point.x = msg.pose.position.x
        p.point.y = msg.pose.position.y
        p.point.z = msg.pose.position.z

        self._marker_buffer.append(
            MarkerSample(
                point=p,
                rx_time=self.get_clock().now(),
            )
        )

    def _cmd_cb(self, msg: String) -> None:
        cmd = msg.data.strip().lower()

        if not cmd:
            return

        if cmd in ("go", "move", "exec", "execute"):
            self._execute_single_marker_move()
            return

        if cmd in ("press", "button", "push", "click"):
            self._start_button_press_sequence()
            return

        if cmd == "status":
            self._print_status()
            return

        if cmd == "clear":
            self._clear_markers()
            return

        if cmd in ("cancel", "stop"):
            self._cancel_active_goal()
            return

        self.get_logger().warn(f"[button_commander] unknown cmd='{cmd}'")

    # =============================================================
    # Command handlers
    # =============================================================
    def _execute_single_marker_move(self) -> None:
        if self._is_busy():
            self.get_logger().warn(
                f"[single_move] rejected because operation is running: {self._operation}"
            )
            return

        target = self._get_smoothed_marker_target()
        if target is None:
            return

        target = self._add_xyz_offset(
            target,
            self.offset_x,
            self.offset_y,
            self.offset_z,
        )

        stage = MotionStage(
            name="SINGLE_MOVE",
            target=target,
            velocity_scaling=self.max_velocity_scaling,
            acceleration_scaling=self.max_acceleration_scaling,
            position_tolerance_m=self.position_tolerance_m,
            hold_after=False,
        )

        self._operation = "single_move"
        self._state = "SINGLE_MOVE"
        self._publish_state(self._state)

        self._send_goal_for_stage(stage)

    def _start_button_press_sequence(self) -> None:
        if self._is_busy():
            reason = f"busy:{self._operation}"
            self.get_logger().warn(
                f"[press] rejected because operation is running: {self._operation}"
            )
            self._publish_result(f"button_press_failed:{reason}")
            return

        if self.approach_distance_m <= 0.0:
            reason = "invalid_approach_distance"
            self.get_logger().error("[press] approach_distance_m must be positive")
            self._publish_result(f"button_press_failed:{reason}")
            return

        if self.retreat_distance_m <= 0.0:
            reason = "invalid_retreat_distance"
            self.get_logger().error("[press] retreat_distance_m must be positive")
            self._publish_result(f"button_press_failed:{reason}")
            return

        if self.press_depth_m < 0.0:
            reason = "invalid_press_depth"
            self.get_logger().error("[press] press_depth_m must be zero or positive")
            self._publish_result(f"button_press_failed:{reason}")
            return

        button = self._get_smoothed_marker_target()
        if button is None:
            reason = "no_valid_marker_target"
            self._publish_result(f"button_press_failed:{reason}")
            return

        button = self._add_xyz_offset(
            button,
            self.button_offset_x,
            self.button_offset_y,
            self.button_offset_z,
        )

        press_dir = self._axis_to_vector(self.press_axis)
        if press_dir is None:
            reason = f"invalid_press_axis:{self.press_axis}"
            self.get_logger().error(
                f"[press] invalid press_axis='{self.press_axis}'. "
                "Use one of: +x, -x, +y, -y, +z, -z"
            )
            self._publish_result(f"button_press_failed:{reason}")
            return

        approach = self._offset_along_vector(
            button,
            press_dir,
            -self.approach_distance_m,
        )

        final_press = self._offset_along_vector(
            button,
            press_dir,
            self.press_depth_m,
        )

        retreat = self._offset_along_vector(
            button,
            press_dir,
            -self.retreat_distance_m,
        )

        sequence: List[MotionStage] = []

        sequence.append(
            MotionStage(
                name="APPROACH",
                target=approach,
                velocity_scaling=self.approach_velocity_scaling,
                acceleration_scaling=self.approach_acceleration_scaling,
                position_tolerance_m=self.position_tolerance_m,
                hold_after=False,
            )
        )

        for step in range(1, self.press_intermediate_steps + 1):
            alpha = step / float(self.press_intermediate_steps)
            press_point = self._interpolate_points(approach, final_press, alpha)

            if self.press_intermediate_steps == 1:
                name = "PRESS"
            else:
                name = f"PRESS_{step:02d}_OF_{self.press_intermediate_steps:02d}"

            sequence.append(
                MotionStage(
                    name=name,
                    target=press_point,
                    velocity_scaling=self.press_velocity_scaling,
                    acceleration_scaling=self.press_acceleration_scaling,
                    position_tolerance_m=self.press_position_tolerance_m,
                    hold_after=(step == self.press_intermediate_steps),
                )
            )

        sequence.append(
            MotionStage(
                name="RETREAT",
                target=retreat,
                velocity_scaling=self.retreat_velocity_scaling,
                acceleration_scaling=self.retreat_acceleration_scaling,
                position_tolerance_m=self.position_tolerance_m,
                hold_after=False,
            )
        )

        self._sequence = sequence
        self._sequence_index = 0
        self._current_stage = None
        self._operation = "press_sequence"
        self._state = "PRESS_SEQUENCE_START"
        self._cancel_requested = False

        self._publish_state(self._state)

        p = button.point
        self.get_logger().info(
            f"[press] button target in {button.header.frame_id}: "
            f"xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), "
            f"axis={self.press_axis}, steps={len(self._sequence)}"
        )

        self._send_next_sequence_goal()

    def _print_status(self) -> None:
        self.get_logger().info(
            f"[status] state={self._state}, "
            f"operation={self._operation}, "
            f"active_goal={self._active_goal}, "
            f"sequence_index={self._sequence_index}/{len(self._sequence)}, "
            f"markers={len(self._marker_buffer)}"
        )

        if not self._marker_buffer:
            self.get_logger().info("[status] no marker cached")
            return

        last = self._marker_buffer[-1]
        age_sec = (self.get_clock().now() - last.rx_time).nanoseconds / 1e9
        p = last.point.point

        self.get_logger().info(
            f"[status] last marker: frame={last.point.header.frame_id}, "
            f"xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), "
            f"age={age_sec:.2f}s"
        )

    def _clear_markers(self) -> None:
        self._marker_buffer.clear()
        self.get_logger().info("[marker] cleared marker buffer")

    def _cancel_active_goal(self) -> None:
        if not self._is_busy():
            self.get_logger().info("[cancel] no active operation")
            return

        self._cancel_requested = True

        if self._hold_timer is not None:
            try:
                self._hold_timer.cancel()
            except Exception:
                pass
            self._hold_timer = None
            self._publish_result("cancelled")
            self._reset_runtime_state()
            self.get_logger().info("[cancel] cancelled during hold")
            return

        if self._goal_handle is None:
            self.get_logger().info("[cancel] cancel requested before goal handle ready")
            return

        self._goal_handle.cancel_goal_async()
        self.get_logger().info("[cancel] cancel requested")

    # =============================================================
    # Sequence execution
    # =============================================================
    def _send_next_sequence_goal(self) -> None:
        if self._operation != "press_sequence":
            return

        if self._cancel_requested:
            self._publish_result("cancelled")
            self._reset_runtime_state()
            return

        if self._sequence_index >= len(self._sequence):
            self._finish_press_sequence()
            return

        stage = self._sequence[self._sequence_index]
        self._send_goal_for_stage(stage)

    def _send_goal_for_stage(self, stage: MotionStage) -> None:
        if self._active_goal:
            self.get_logger().warn(
                f"[moveit] cannot send stage={stage.name}; another goal is active"
            )
            return

        if not self.move_action_client.wait_for_server(
            timeout_sec=self.wait_for_server_sec
        ):
            self._fail_current_operation(
                f"move_action_server_not_ready:{self.move_action_name}"
            )
            return

        self._current_stage = stage
        self._state = stage.name
        self._publish_state(self._state)

        goal = self._build_goal(stage)

        self._active_goal = True
        send_future = self.move_action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

        p = stage.target.point
        self.get_logger().info(
            f"[moveit] send stage={stage.name}, "
            f"frame={stage.target.header.frame_id}, "
            f"xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), "
            f"vel={stage.velocity_scaling:.3f}, "
            f"acc={stage.acceleration_scaling:.3f}, "
            f"tol={stage.position_tolerance_m:.4f}"
        )

    def _goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._active_goal = False
            self._fail_current_operation(f"send_goal_failed:{exc}")
            return

        if not goal_handle.accepted:
            self._active_goal = False
            self._fail_current_operation("goal_rejected")
            return

        self._goal_handle = goal_handle

        if self._cancel_requested:
            self.get_logger().info("[moveit] goal accepted, sending cancel request")
            goal_handle.cancel_goal_async()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

        stage_name = self._current_stage.name if self._current_stage else "UNKNOWN"
        self.get_logger().info(f"[moveit] goal accepted: stage={stage_name}")

    def _result_cb(self, future) -> None:
        self._active_goal = False
        self._goal_handle = None

        try:
            wrapped = future.result()
        except Exception as exc:
            self._fail_current_operation(f"get_result_failed:{exc}")
            return

        result = wrapped.result
        status = wrapped.status
        moveit_code = result.error_code.val

        if self._cancel_requested:
            self.get_logger().info(
                f"[moveit] cancelled: status={self._goal_status_name(status)}"
            )
            self._publish_result("cancelled")
            self._reset_runtime_state()
            return

        ok = (
            status == GoalStatus.STATUS_SUCCEEDED
            and moveit_code == MoveItErrorCodes.SUCCESS
        )

        stage_name = self._current_stage.name if self._current_stage else "UNKNOWN"

        if not ok:
            self._fail_current_operation(
                f"stage_failed:{stage_name}, "
                f"action_status={self._goal_status_name(status)}, "
                f"moveit_error={moveit_code}"
            )
            return

        self.get_logger().info(f"[moveit] stage succeeded: {stage_name}")

        if self._operation == "single_move":
            self._publish_result("move_done")
            self._reset_runtime_state()
            return

        if self._operation == "press_sequence":
            stage = self._current_stage
            self._sequence_index += 1

            if stage is not None and stage.hold_after:
                self._start_hold()
                return

            self._send_next_sequence_goal()
            return

        self._reset_runtime_state()

    def _start_hold(self) -> None:
        if self.plan_only:
            self._send_next_sequence_goal()
            return

        if self.press_hold_sec <= 0.0:
            self._send_next_sequence_goal()
            return

        self._state = "HOLD"
        self._publish_state(self._state)

        self.get_logger().info(f"[press] hold for {self.press_hold_sec:.2f}s")

        if self._hold_timer is not None:
            try:
                self._hold_timer.cancel()
            except Exception:
                pass
            self._hold_timer = None

        self._hold_timer = self.create_timer(
            self.press_hold_sec,
            self._hold_done_cb,
        )

    def _hold_done_cb(self) -> None:
        if self._hold_timer is not None:
            try:
                self._hold_timer.cancel()
            except Exception:
                pass
            self._hold_timer = None

        if self._cancel_requested:
            self._publish_result("cancelled")
            self._reset_runtime_state()
            return

        self.get_logger().info("[press] hold done")
        self._send_next_sequence_goal()

    def _finish_press_sequence(self) -> None:
        self.get_logger().info("[press] button press sequence done")
        self._publish_result("button_press_done")
        self._reset_runtime_state()

    def _fail_current_operation(self, reason: str) -> None:
        self.get_logger().warn(f"[operation] failed: {reason}")

        if self._operation == "press_sequence":
            self._publish_result(f"button_press_failed:{reason}")
        elif self._operation == "single_move":
            self._publish_result(f"move_failed:{reason}")
        else:
            self._publish_result(f"operation_failed:{reason}")

        self._reset_runtime_state()

    def _reset_runtime_state(self) -> None:
        if self._hold_timer is not None:
            try:
                self._hold_timer.cancel()
            except Exception:
                pass
            self._hold_timer = None

        self._active_goal = False
        self._goal_handle = None
        self._cancel_requested = False

        self._operation = "idle"
        self._state = "IDLE"

        self._sequence.clear()
        self._sequence_index = 0
        self._current_stage = None

        self._publish_state(self._state)

    def _is_busy(self) -> bool:
        return (
            self._operation != "idle"
            or self._active_goal
            or self._hold_timer is not None
        )

    # =============================================================
    # Marker / TF utilities
    # =============================================================
    def _get_smoothed_marker_target(self) -> Optional[PointStamped]:
        if not self._marker_buffer:
            self.get_logger().warn("[marker] no cached marker yet")
            return None

        now = self.get_clock().now()
        transformed_points: List[PointStamped] = []

        for sample in list(self._marker_buffer):
            age_sec = (now - sample.rx_time).nanoseconds / 1e9

            if age_sec > self.marker_timeout_sec:
                continue

            target = self._transform_to_target_frame(sample.point, warn=False)
            if target is not None:
                transformed_points.append(target)

        if not transformed_points:
            last = self._marker_buffer[-1]
            age_sec = (now - last.rx_time).nanoseconds / 1e9
            self.get_logger().warn(
                f"[marker] no valid marker after timeout/TF filtering. "
                f"last_frame={last.point.header.frame_id}, "
                f"last_age={age_sec:.2f}s"
            )
            return None

        xs = [p.point.x for p in transformed_points]
        ys = [p.point.y for p in transformed_points]
        zs = [p.point.z for p in transformed_points]

        if self.smoothing_method == "mean":
            fx = mean(xs)
            fy = mean(ys)
            fz = mean(zs)
        else:
            fx = median(xs)
            fy = median(ys)
            fz = median(zs)

        out = PointStamped()
        out.header.frame_id = self.target_frame
        out.header.stamp = now.to_msg()
        out.point.x = float(fx)
        out.point.y = float(fy)
        out.point.z = float(fz)

        self.get_logger().info(
            f"[marker] selected target: frame={out.header.frame_id}, "
            f"xyz=({out.point.x:.3f}, {out.point.y:.3f}, {out.point.z:.3f}), "
            f"samples={len(transformed_points)}, method={self.smoothing_method}"
        )

        return out

    def _transform_to_target_frame(
        self,
        point: PointStamped,
        warn: bool = True,
    ) -> Optional[PointStamped]:
        if point.header.frame_id == self.target_frame:
            return self._clone_point(point, self.target_frame)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                point.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.1),
            )
            return do_transform_point(point, tf)

        except TransformException as exc:
            if warn:
                self.get_logger().warn(
                    f"[tf] failed: src={point.header.frame_id}, "
                    f"dst={self.target_frame}, err={exc}"
                )
            return None

    def _clone_point(
        self,
        point: PointStamped,
        frame_id: Optional[str] = None,
    ) -> PointStamped:
        out = PointStamped()
        out.header = point.header
        if frame_id is not None:
            out.header.frame_id = frame_id
        out.point.x = point.point.x
        out.point.y = point.point.y
        out.point.z = point.point.z
        return out

    # =============================================================
    # Geometry utilities
    # =============================================================
    def _add_xyz_offset(
        self,
        point: PointStamped,
        dx: float,
        dy: float,
        dz: float,
    ) -> PointStamped:
        out = self._clone_point(point)
        out.point.x += dx
        out.point.y += dy
        out.point.z += dz
        return out

    def _offset_along_vector(
        self,
        point: PointStamped,
        direction: Tuple[float, float, float],
        distance: float,
    ) -> PointStamped:
        out = self._clone_point(point)
        out.point.x += direction[0] * distance
        out.point.y += direction[1] * distance
        out.point.z += direction[2] * distance
        return out

    def _interpolate_points(
        self,
        start: PointStamped,
        end: PointStamped,
        alpha: float,
    ) -> PointStamped:
        alpha = max(0.0, min(1.0, alpha))

        out = PointStamped()
        out.header.frame_id = self.target_frame
        out.header.stamp = self.get_clock().now().to_msg()

        out.point.x = start.point.x + (end.point.x - start.point.x) * alpha
        out.point.y = start.point.y + (end.point.y - start.point.y) * alpha
        out.point.z = start.point.z + (end.point.z - start.point.z) * alpha

        return out

    def _axis_to_vector(
        self,
        axis: str,
    ) -> Optional[Tuple[float, float, float]]:
        axis = axis.strip().lower()

        axis_map = {
            "+x": (1.0, 0.0, 0.0),
            "x": (1.0, 0.0, 0.0),
            "-x": (-1.0, 0.0, 0.0),
            "+y": (0.0, 1.0, 0.0),
            "y": (0.0, 1.0, 0.0),
            "-y": (0.0, -1.0, 0.0),
            "+z": (0.0, 0.0, 1.0),
            "z": (0.0, 0.0, 1.0),
            "-z": (0.0, 0.0, -1.0),
        }

        return axis_map.get(axis)

    # =============================================================
    # MoveIt goal builder
    # =============================================================
    def _build_goal(self, stage: MotionStage) -> MoveGroup.Goal:
        target_point = stage.target

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target_point.header.frame_id
        position_constraint.link_name = self.ee_link
        position_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [max(0.0005, stage.position_tolerance_m)]

        region_pose = Pose()
        region_pose.position.x = target_point.point.x
        region_pose.position.y = target_point.point.y
        region_pose.position.z = target_point.point.z
        region_pose.orientation.w = 1.0

        position_constraint.constraint_region.primitives.append(sphere)
        position_constraint.constraint_region.primitive_poses.append(region_pose)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target_point.header.frame_id
        orientation_constraint.link_name = self.ee_link

        orientation_constraint.orientation.x = self.goal_qx
        orientation_constraint.orientation.y = self.goal_qy
        orientation_constraint.orientation.z = self.goal_qz
        orientation_constraint.orientation.w = self.goal_qw

        orientation_constraint.absolute_x_axis_tolerance = self.ori_tol_x
        orientation_constraint.absolute_y_axis_tolerance = self.ori_tol_y
        orientation_constraint.absolute_z_axis_tolerance = self.ori_tol_z
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)

        if self.use_orientation_constraint:
            constraints.orientation_constraints.append(orientation_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = self.num_planning_attempts
        goal.request.allowed_planning_time = self.allowed_planning_time_sec
        goal.request.max_velocity_scaling_factor = stage.velocity_scaling
        goal.request.max_acceleration_scaling_factor = stage.acceleration_scaling
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = self.plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = self.replan
        goal.planning_options.replan_attempts = self.replan_attempts
        goal.planning_options.replan_delay = self.replan_delay_sec

        return goal

    # =============================================================
    # Publishers / formatting
    # =============================================================
    def _publish_result(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)
        self.get_logger().info(f"[result] {text}")

    def _publish_state(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def _goal_status_name(self, status: int) -> str:
        status_map = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        return status_map.get(status, f"STATUS_{status}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarkerButtonPressCommander()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()