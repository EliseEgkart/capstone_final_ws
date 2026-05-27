#!/usr/bin/env python3
"""
marker_button_press_commander.py

이 버전은 잘 동작했던 marker_moveit_commander 로직을 그대로 기반으로 한다.

핵심 로직:
  - /object_3d_marker의 마지막 Marker 1개만 사용
  - Marker를 target_frame으로 TF 변환
  - 명령에 따라 outside/inside offset을 선택해 단일 목표점 생성
  - PositionConstraint + OrientationConstraint를 항상 같이 사용
  - MoveGroup action goal 1개만 전송
  - 성공 시:
      press 명령이면 button_press_done
      go 명령이면 move_done
  - 실패 시:
      press 명령이면 button_press_failed:...
      go 명령이면 move_failed:...

주의:
  - 기존 APPROACH -> PRESS -> HOLD -> RETREAT 분할 방식은 사용하지 않는다.
  - 버튼 누르기 깊이/방향은 press_axis, press_depth가 아니라 outside/inside offset으로 조정한다.
  - home 복귀는 manipulator_task_manager.py가 담당한다.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

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


class MarkerButtonPressCommander(Node):
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

        self.declare_parameter("marker_timeout_sec", 30.0)
        self.declare_parameter("position_tolerance_m", 0.01)
        self.declare_parameter("max_velocity_scaling", 0.2)
        self.declare_parameter("max_acceleration_scaling", 0.2)
        self.declare_parameter("allowed_planning_time_sec", 3.0)
        self.declare_parameter("num_planning_attempts", 5)

        # =========================================================
        # Offset parameters
        # =========================================================
        # Legacy offset_* parameters are kept for backward compatibility.
        # If outside_offset_* is not configured, outside mode falls back to offset_*.
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)

        # New mode-based offsets.
        # - press_outside uses outside_offset_*
        # - press_inside uses inside_offset_*
        # - press uses default_offset_mode
        self.declare_parameter("default_offset_mode", "outside")
        self.declare_parameter("outside_offset_x", 0.0)
        self.declare_parameter("outside_offset_y", 0.0)
        self.declare_parameter("outside_offset_z", 0.0)
        self.declare_parameter("inside_offset_x", 0.0)
        self.declare_parameter("inside_offset_y", 0.0)
        self.declare_parameter("inside_offset_z", 0.0)

        # Additional fine correction applied after outside/inside offset selection.
        # prefer_button_offset is kept for legacy compatibility:
        #   true  -> use button_offset_* only
        #   false -> use selected mode offset + button_offset_*
        self.declare_parameter("button_offset_x", 0.0)
        self.declare_parameter("button_offset_y", 0.0)
        self.declare_parameter("button_offset_z", 0.0)
        self.declare_parameter("prefer_button_offset", False)

        # 잘 되던 코드와 동일하게 orientation constraint를 항상 사용한다.
        # tolerance 기본값도 3.14로 둔다.
        self.declare_parameter("goal_qx", 0.0)
        self.declare_parameter("goal_qy", 0.0)
        self.declare_parameter("goal_qz", 0.0)
        self.declare_parameter("goal_qw", 1.0)
        self.declare_parameter("ori_tol_x", 3.14)
        self.declare_parameter("ori_tol_y", 3.14)
        self.declare_parameter("ori_tol_z", 3.14)

        self.declare_parameter("replan", True)
        self.declare_parameter("replan_attempts", 1)
        self.declare_parameter("replan_delay_sec", 1.0)
        self.declare_parameter("wait_for_server_sec", 2.0)

        # plan only는 테스트용으로 유지한다.
        self.declare_parameter("plan_only", False)

        # =========================================================
        # Load parameters
        # =========================================================
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.move_action_name = str(self.get_parameter("move_action_name").value)

        self.planning_group = str(self.get_parameter("planning_group").value)
        self.ee_link = str(self.get_parameter("ee_link").value)
        self.target_frame = str(self.get_parameter("target_frame").value)

        self.marker_timeout_sec = float(self.get_parameter("marker_timeout_sec").value)
        self.position_tolerance_m = float(
            self.get_parameter("position_tolerance_m").value
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

        self.offset_x = float(self.get_parameter("offset_x").value)
        self.offset_y = float(self.get_parameter("offset_y").value)
        self.offset_z = float(self.get_parameter("offset_z").value)
        self.legacy_offset = (self.offset_x, self.offset_y, self.offset_z)

        self.default_offset_mode = self._normalize_offset_mode(
            str(self.get_parameter("default_offset_mode").value)
        )

        self.outside_offset = (
            float(self.get_parameter("outside_offset_x").value),
            float(self.get_parameter("outside_offset_y").value),
            float(self.get_parameter("outside_offset_z").value),
        )

        self.inside_offset = (
            float(self.get_parameter("inside_offset_x").value),
            float(self.get_parameter("inside_offset_y").value),
            float(self.get_parameter("inside_offset_z").value),
        )

        # Backward compatibility: if the new outside offset is left at zero but
        # legacy offset_* is configured, use the legacy offset for outside mode.
        if self.outside_offset == (0.0, 0.0, 0.0) and self.legacy_offset != (0.0, 0.0, 0.0):
            self.outside_offset = self.legacy_offset

        self.button_offset_x = float(self.get_parameter("button_offset_x").value)
        self.button_offset_y = float(self.get_parameter("button_offset_y").value)
        self.button_offset_z = float(self.get_parameter("button_offset_z").value)
        self.button_offset = (
            self.button_offset_x,
            self.button_offset_y,
            self.button_offset_z,
        )
        self.prefer_button_offset = bool(
            self.get_parameter("prefer_button_offset").value
        )

        self.goal_qx = float(self.get_parameter("goal_qx").value)
        self.goal_qy = float(self.get_parameter("goal_qy").value)
        self.goal_qz = float(self.get_parameter("goal_qz").value)
        self.goal_qw = float(self.get_parameter("goal_qw").value)
        self.ori_tol_x = float(self.get_parameter("ori_tol_x").value)
        self.ori_tol_y = float(self.get_parameter("ori_tol_y").value)
        self.ori_tol_z = float(self.get_parameter("ori_tol_z").value)

        self.replan = bool(self.get_parameter("replan").value)
        self.replan_attempts = int(self.get_parameter("replan_attempts").value)
        self.replan_delay_sec = float(self.get_parameter("replan_delay_sec").value)
        self.wait_for_server_sec = float(
            self.get_parameter("wait_for_server_sec").value
        )
        self.plan_only = bool(self.get_parameter("plan_only").value)

        # =========================================================
        # Runtime state
        # =========================================================
        self._last_marker_point: Optional[PointStamped] = None
        self._last_marker_rx_time = None

        self._active_goal = False
        self._goal_handle = None
        self._active_cmd = "idle"  # idle | go | press
        self._active_offset_mode = self.default_offset_mode
        self._state = "IDLE"

        # =========================================================
        # ROS interfaces
        # =========================================================
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.move_action_client = ActionClient(self, MoveGroup, self.move_action_name)

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

        self.get_logger().info("[button_commander] ready")
        self.get_logger().info("[button_commander] logic=LAST_MARKER_SINGLE_MOVE_GOAL")
        self.get_logger().info(f"[button_commander] marker_topic={self.marker_topic}")
        self.get_logger().info(f"[button_commander] cmd_topic={self.cmd_topic}")
        self.get_logger().info(f"[button_commander] result_topic={self.result_topic}")
        self.get_logger().info(f"[button_commander] state_topic={self.state_topic}")
        self.get_logger().info(f"[button_commander] move_action={self.move_action_name}")
        self.get_logger().info(
            "[button_commander] commands: go|move|exec|execute, "
            "press|press_outside|press_inside|button|push|click, "
            "status, clear, cancel"
        )
        self.get_logger().info(
            f"[button_commander] default_offset_mode={self.default_offset_mode}, "
            f"outside_offset=({self.outside_offset[0]:.4f}, "
            f"{self.outside_offset[1]:.4f}, {self.outside_offset[2]:.4f}), "
            f"inside_offset=({self.inside_offset[0]:.4f}, "
            f"{self.inside_offset[1]:.4f}, {self.inside_offset[2]:.4f})"
        )
        self.get_logger().info(
            f"[button_commander] legacy_offset=({self.offset_x:.4f}, "
            f"{self.offset_y:.4f}, {self.offset_z:.4f}), "
            f"button_offset=({self.button_offset_x:.4f}, "
            f"{self.button_offset_y:.4f}, {self.button_offset_z:.4f}), "
            f"prefer_button_offset={self.prefer_button_offset}"
        )
        self.get_logger().info(
            f"[button_commander] pos_tol={self.position_tolerance_m:.4f}, "
            f"vel={self.max_velocity_scaling:.3f}, "
            f"acc={self.max_acceleration_scaling:.3f}"
        )
        self.get_logger().info(
            f"[button_commander] orientation constraint always ON, "
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

        # 잘 되던 코드와 동일하게 마지막 Marker 하나만 사용한다.
        self._last_marker_point = p
        self._last_marker_rx_time = self.get_clock().now()

    def _cmd_cb(self, msg: String) -> None:
        cmd = msg.data.strip().lower()

        if not cmd:
            return

        if cmd in ("go", "move", "exec", "execute"):
            self._execute_last_marker(
                active_cmd="go",
                offset_mode=self.default_offset_mode,
            )
            return

        if cmd in ("press", "button", "push", "click"):
            self._execute_last_marker(
                active_cmd="press",
                offset_mode=self.default_offset_mode,
            )
            return

        if cmd in ("press_outside", "outside_press", "button_outside", "push_outside"):
            self._execute_last_marker(active_cmd="press", offset_mode="outside")
            return

        if cmd in ("press_inside", "inside_press", "button_inside", "push_inside"):
            self._execute_last_marker(active_cmd="press", offset_mode="inside")
            return

        if cmd == "status":
            self._print_status()
            return

        if cmd == "clear":
            self._clear_marker()
            return

        if cmd in ("cancel", "stop"):
            self._cancel_active_goal()
            return

        self.get_logger().warn(f"[button_commander] unknown cmd='{cmd}'")

    # =============================================================
    # Command handlers
    # =============================================================
    def _print_status(self) -> None:
        self.get_logger().info(
            f"[status] state={self._state}, active_cmd={self._active_cmd}, "
            f"offset_mode={self._active_offset_mode}, "
            f"active_goal={self._active_goal}"
        )

        if self._last_marker_point is None or self._last_marker_rx_time is None:
            self.get_logger().info("[status] no marker cached")
            return

        age_sec = (
            (self.get_clock().now() - self._last_marker_rx_time).nanoseconds / 1e9
        )
        p = self._last_marker_point.point
        self.get_logger().info(
            f"[status] last marker: frame={self._last_marker_point.header.frame_id}, "
            f"xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), age={age_sec:.2f}s"
        )

    def _clear_marker(self) -> None:
        self._last_marker_point = None
        self._last_marker_rx_time = None
        self.get_logger().info("[marker] cleared last marker")

    def _cancel_active_goal(self) -> None:
        if self._goal_handle is None:
            self.get_logger().info("[cancel] no active goal to cancel")
            return

        self._goal_handle.cancel_goal_async()
        self.get_logger().info("[cancel] cancel requested")

    def _execute_last_marker(
        self,
        active_cmd: str,
        offset_mode: Optional[str] = None,
    ) -> None:
        if self._active_goal:
            self.get_logger().warn("[button_commander] move is already running")
            if active_cmd == "press":
                self._publish_result("button_press_failed:busy")
            return

        if self._last_marker_point is None or self._last_marker_rx_time is None:
            self.get_logger().warn("[button_commander] no cached marker yet")
            if active_cmd == "press":
                self._publish_result("button_press_failed:no_valid_marker_target")
            return

        age_sec = (
            (self.get_clock().now() - self._last_marker_rx_time).nanoseconds / 1e9
        )
        if age_sec > self.marker_timeout_sec:
            self.get_logger().warn(
                f"[button_commander] cached marker too old "
                f"({age_sec:.2f}s > {self.marker_timeout_sec:.2f}s)"
            )
            if active_cmd == "press":
                self._publish_result("button_press_failed:marker_timeout")
            return

        target = self._transform_to_target_frame(self._last_marker_point)
        if target is None:
            if active_cmd == "press":
                self._publish_result("button_press_failed:tf_failed")
            return

        if not self.move_action_client.wait_for_server(
            timeout_sec=self.wait_for_server_sec
        ):
            self.get_logger().error(
                f"[button_commander] move action server not ready: {self.move_action_name}"
            )
            if active_cmd == "press":
                self._publish_result(
                    f"button_press_failed:move_action_server_not_ready:{self.move_action_name}"
                )
            return

        self._active_cmd = active_cmd
        self._active_offset_mode = self._normalize_offset_mode(
            offset_mode or self.default_offset_mode
        )
        self._state = "DIRECT_PRESS" if active_cmd == "press" else "SINGLE_MOVE"
        self._publish_state(self._state)

        goal = self._build_goal(target)

        self._active_goal = True
        send_future = self.move_action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    # =============================================================
    # TF / MoveIt goal
    # =============================================================
    def _transform_to_target_frame(
        self,
        point: PointStamped,
    ) -> Optional[PointStamped]:
        if point.header.frame_id == self.target_frame:
            return point

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            out = do_transform_point(point, tf)
            return out

        except TransformException as exc:
            self.get_logger().warn(
                f"[button_commander] TF failed: "
                f"src={point.header.frame_id}, dst={self.target_frame}, err={exc}"
            )
            return None

    def _normalize_offset_mode(self, mode: str) -> str:
        normalized = str(mode).strip().lower()

        if normalized in ("inside", "in", "elevator_inside"):
            return "inside"

        if normalized in ("outside", "out", "elevator_outside"):
            return "outside"

        self.get_logger().warn(
            f"[button_commander] unknown offset_mode='{mode}', "
            f"fallback={self.default_offset_mode}"
        )
        return "inside" if self.default_offset_mode == "inside" else "outside"

    def _base_offset_for_mode(self, mode: str):
        normalized = self._normalize_offset_mode(mode)

        if normalized == "inside":
            return self.inside_offset

        return self.outside_offset

    def _selected_offset(self):
        # Legacy compatibility path: if explicitly requested, use button_offset_* only.
        if self.prefer_button_offset:
            return self.button_offset

        base_x, base_y, base_z = self._base_offset_for_mode(self._active_offset_mode)
        fine_x, fine_y, fine_z = self.button_offset

        return (
            base_x + fine_x,
            base_y + fine_y,
            base_z + fine_z,
        )

    def _build_goal(self, target_point: PointStamped) -> MoveGroup.Goal:
        dx, dy, dz = self._selected_offset()

        x = target_point.point.x + dx
        y = target_point.point.y + dy
        z = target_point.point.z + dz

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.target_frame
        position_constraint.link_name = self.ee_link
        position_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance_m]

        region_pose = Pose()
        region_pose.position.x = x
        region_pose.position.y = y
        region_pose.position.z = z
        region_pose.orientation.w = 1.0

        position_constraint.constraint_region.primitives.append(sphere)
        position_constraint.constraint_region.primitive_poses.append(region_pose)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.target_frame
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

        # 잘 되던 marker_moveit_commander와 동일하게 orientation constraint를 항상 넣는다.
        constraints.orientation_constraints.append(orientation_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = self.num_planning_attempts
        goal.request.allowed_planning_time = self.allowed_planning_time_sec
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = self.plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = self.replan
        goal.planning_options.replan_attempts = self.replan_attempts
        goal.planning_options.replan_delay = self.replan_delay_sec

        self.get_logger().info(
            f"[button_commander] send goal: frame={self.target_frame}, "
            f"xyz=({x:.3f}, {y:.3f}, {z:.3f}), "
            f"offset_mode={self._active_offset_mode}, "
            f"offset=({dx:.4f}, {dy:.4f}, {dz:.4f}), "
            f"cmd={self._active_cmd}, "
            f"plan_only={self.plan_only}"
        )

        return goal

    # =============================================================
    # MoveIt callbacks
    # =============================================================
    def _goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._active_goal = False
            self._publish_failed_result(f"send_goal_failed:{exc}")
            return

        if not goal_handle.accepted:
            self._active_goal = False
            self._publish_failed_result("goal_rejected")
            return

        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

        self.get_logger().info(f"[button_commander] goal accepted: cmd={self._active_cmd}")

    def _result_cb(self, future) -> None:
        self._active_goal = False
        self._goal_handle = None

        try:
            wrapped = future.result()
        except Exception as exc:
            self._publish_failed_result(f"get_result_failed:{exc}")
            return

        result = wrapped.result
        status = wrapped.status
        code = result.error_code.val

        ok = (
            status == GoalStatus.STATUS_SUCCEEDED
            and code == MoveItErrorCodes.SUCCESS
        )

        if ok:
            self.get_logger().info(
                f"[button_commander] execution succeeded: cmd={self._active_cmd}"
            )

            if self._active_cmd == "press":
                self._publish_result("button_press_done")
            else:
                self._publish_result("move_done")

            self._reset_runtime_state()
            return

        self._publish_failed_result(
            f"execution_failed:status={self._goal_status_name(status)},moveit_error={code}"
        )

    # =============================================================
    # Runtime helpers
    # =============================================================
    def _publish_failed_result(self, reason: str) -> None:
        self.get_logger().warn(f"[button_commander] failed: {reason}")

        if self._active_cmd == "press":
            self._publish_result(f"button_press_failed:{reason}")
        elif self._active_cmd == "go":
            self._publish_result(f"move_failed:{reason}")
        else:
            self._publish_result(f"operation_failed:{reason}")

        self._reset_runtime_state()

    def _reset_runtime_state(self) -> None:
        self._active_goal = False
        self._goal_handle = None
        self._active_cmd = "idle"
        self._active_offset_mode = self.default_offset_mode
        self._state = "IDLE"
        self._publish_state(self._state)

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