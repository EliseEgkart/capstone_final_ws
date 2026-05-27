#!/usr/bin/env python3
# 로봇팔의 기본동작, 탐지동작 1,2 를 지정함.
'''
외부 버튼 인식 자세: ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'outside_scan'}"

내부 층수 버튼 인식 자세: ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'inside_scan'}"

기본 자세: ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'home'}"

상태 확인: ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'status'}"

취소: ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'cancel'}"

결과 확인: ros2 topic echo /arm_pose_commander/done
'''
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from std_msgs.msg import String


class ArmPoseCommander(Node):
    """
    MoveIt-based fixed pose commander for manipulator scan/home poses.

    Supported modes:
      - outside_scan: pose for elevator outside call button perception
      - inside_scan : pose for elevator inside floor button perception
      - home        : default safe/basic pose
    """

    def __init__(self) -> None:
        super().__init__("arm_pose_commander")

        # =====================================================
        # Parameters
        # =====================================================
        self.declare_parameter("flag_topic", "/arm_pose_commander/flag")
        self.declare_parameter("done_topic", "/arm_pose_commander/done")
        self.declare_parameter("move_action_name", "/move_action")

        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4"])

        self.declare_parameter("outside_scan_positions", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("inside_scan_positions", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("home_positions", [0.0, 0.0, 0.0, 0.0])

        self.declare_parameter("joint_tolerance", 0.02)
        self.declare_parameter("max_velocity_scaling", 0.2)
        self.declare_parameter("max_acceleration_scaling", 0.2)
        self.declare_parameter("allowed_planning_time_sec", 3.0)
        self.declare_parameter("num_planning_attempts", 5)

        self.declare_parameter("replan", True)
        self.declare_parameter("replan_attempts", 1)
        self.declare_parameter("replan_delay_sec", 1.0)
        self.declare_parameter("wait_for_server_sec", 2.0)

        # =====================================================
        # Load parameters
        # =====================================================
        self.flag_topic = self.get_parameter("flag_topic").value
        self.done_topic = self.get_parameter("done_topic").value
        self.move_action_name = self.get_parameter("move_action_name").value

        self.planning_group = self.get_parameter("planning_group").value
        self.joint_names = list(self.get_parameter("joint_names").value)

        self.pose_map: Dict[str, List[float]] = {
            "outside_scan": list(self.get_parameter("outside_scan_positions").value),
            "inside_scan": list(self.get_parameter("inside_scan_positions").value),
            "home": list(self.get_parameter("home_positions").value),
        }

        self.joint_tolerance = float(self.get_parameter("joint_tolerance").value)
        self.max_velocity_scaling = float(self.get_parameter("max_velocity_scaling").value)
        self.max_acceleration_scaling = float(
            self.get_parameter("max_acceleration_scaling").value
        )
        self.allowed_planning_time_sec = float(
            self.get_parameter("allowed_planning_time_sec").value
        )
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)

        self.replan = bool(self.get_parameter("replan").value)
        self.replan_attempts = int(self.get_parameter("replan_attempts").value)
        self.replan_delay_sec = float(self.get_parameter("replan_delay_sec").value)
        self.wait_for_server_sec = float(self.get_parameter("wait_for_server_sec").value)

        # =====================================================
        # ROS interfaces
        # =====================================================
        self.move_action_client = ActionClient(self, MoveGroup, self.move_action_name)

        self.flag_sub = self.create_subscription(
            String,
            self.flag_topic,
            self._flag_cb,
            10
        )

        self.done_pub = self.create_publisher(
            String,
            self.done_topic,
            10
        )

        self._active_goal = False
        self._goal_handle = None
        self._current_mode: Optional[str] = None

        self._validate_pose_map()

        self.get_logger().info("[arm_pose_commander] ready")
        self.get_logger().info(f"[arm_pose_commander] flag_topic={self.flag_topic}")
        self.get_logger().info(f"[arm_pose_commander] done_topic={self.done_topic}")
        self.get_logger().info(f"[arm_pose_commander] move_action={self.move_action_name}")
        self.get_logger().info(f"[arm_pose_commander] joint_names={self.joint_names}")
        self.get_logger().info(
            "[arm_pose_commander] flags: outside_scan, inside_scan, home, status, cancel"
        )

    # =========================================================
    # Validation
    # =========================================================
    def _validate_pose_map(self) -> None:
        expected_len = len(self.joint_names)

        for mode, positions in self.pose_map.items():
            if len(positions) != expected_len:
                raise ValueError(
                    f"Pose '{mode}' has {len(positions)} positions, "
                    f"but joint_names has {expected_len} joints."
                )

    # =========================================================
    # Topic callback
    # =========================================================
    def _flag_cb(self, msg: String) -> None:
        flag = msg.data.strip().lower()

        if not flag:
            return

        if flag == "status":
            self._print_status()
            return

        if flag == "cancel":
            self._cancel_active_goal()
            return

        if flag not in self.pose_map:
            self.get_logger().warn(f"[arm_pose_commander] unknown flag='{flag}'")
            return

        self._move_to_pose(flag)

    # =========================================================
    # Main behavior
    # =========================================================
    def _move_to_pose(self, mode: str) -> None:
        if self._active_goal:
            self.get_logger().warn(
                f"[arm_pose_commander] move already running. ignored mode='{mode}'"
            )
            return

        if not self.move_action_client.wait_for_server(
            timeout_sec=self.wait_for_server_sec
        ):
            self.get_logger().error(
                f"[arm_pose_commander] move action server not ready: {self.move_action_name}"
            )
            self._publish_done(f"{mode}_failed")
            return

        goal = self._build_joint_goal(mode)

        self._active_goal = True
        self._current_mode = mode

        send_future = self.move_action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

        self.get_logger().info(f"[arm_pose_commander] sent pose goal: {mode}")

    def _build_joint_goal(self, mode: str) -> MoveGroup.Goal:
        positions = self.pose_map[mode]

        constraints = Constraints()
        constraints.name = f"{mode}_joint_goal"

        for joint_name, position in zip(self.joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.tolerance_above = self.joint_tolerance
            jc.tolerance_below = self.joint_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = self.num_planning_attempts
        goal.request.allowed_planning_time = self.allowed_planning_time_sec
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = self.replan
        goal.planning_options.replan_attempts = self.replan_attempts
        goal.planning_options.replan_delay = self.replan_delay_sec

        return goal

    # =========================================================
    # Action callbacks
    # =========================================================
    def _goal_response_cb(self, future) -> None:
        mode = self._current_mode or "unknown"

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._active_goal = False
            self.get_logger().error(
                f"[arm_pose_commander] send goal failed: {exc}"
            )
            self._publish_done(f"{mode}_failed")
            return

        if not goal_handle.accepted:
            self._active_goal = False
            self.get_logger().warn(f"[arm_pose_commander] goal rejected: {mode}")
            self._publish_done(f"{mode}_rejected")
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

        self.get_logger().info(f"[arm_pose_commander] goal accepted: {mode}")

    def _result_cb(self, future) -> None:
        mode = self._current_mode or "unknown"

        self._active_goal = False
        self._goal_handle = None
        self._current_mode = None

        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"[arm_pose_commander] get result failed: {exc}"
            )
            self._publish_done(f"{mode}_failed")
            return

        result = wrapped.result
        status = wrapped.status
        code = result.error_code.val

        ok = (
            status == GoalStatus.STATUS_SUCCEEDED
            and code == MoveItErrorCodes.SUCCESS
        )

        if ok:
            self.get_logger().info(f"[arm_pose_commander] pose reached: {mode}")
            self._publish_done(f"{mode}_done")
            return

        self.get_logger().warn(
            f"[arm_pose_commander] pose failed: mode={mode}, "
            f"status={status}, moveit_error={code}"
        )
        self._publish_done(f"{mode}_failed")

    # =========================================================
    # Utility
    # =========================================================
    def _cancel_active_goal(self) -> None:
        if self._goal_handle is None:
            self.get_logger().info("[arm_pose_commander] no active goal to cancel")
            return

        self._goal_handle.cancel_goal_async()
        self.get_logger().info("[arm_pose_commander] cancel requested")

    def _print_status(self) -> None:
        self.get_logger().info(
            f"[arm_pose_commander] active={self._active_goal}, "
            f"current_mode={self._current_mode}"
        )

    def _publish_done(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.done_pub.publish(msg)

        self.get_logger().info(f"[arm_pose_commander] done='{text}'")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmPoseCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()