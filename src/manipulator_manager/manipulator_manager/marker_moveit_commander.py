#!/usr/bin/env python3

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


class MarkerMoveItCommander(Node):
    def __init__(self) -> None:
        super().__init__("marker_moveit_commander")
        self.declare_parameter("marker_topic", "/object_3d_marker")
        self.declare_parameter("cmd_topic", "/manipulator_manager/cmd")
        self.declare_parameter("move_action_name", "/move_action")

        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("ee_link", "ee_link")
        self.declare_parameter("target_frame", "link1")

        self.declare_parameter("marker_timeout_sec", 30.0)
        self.declare_parameter("position_tolerance_m", 0.01)
        self.declare_parameter("max_velocity_scaling", 0.2)
        self.declare_parameter("max_acceleration_scaling", 0.2)
        self.declare_parameter("allowed_planning_time_sec", 3.0)
        self.declare_parameter("num_planning_attempts", 5)

        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)

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

        self.marker_topic = self.get_parameter("marker_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.move_action_name = self.get_parameter("move_action_name").value

        self.planning_group = self.get_parameter("planning_group").value
        self.ee_link = self.get_parameter("ee_link").value
        self.target_frame = self.get_parameter("target_frame").value

        self.marker_timeout_sec = float(self.get_parameter("marker_timeout_sec").value)
        self.position_tolerance_m = float(self.get_parameter("position_tolerance_m").value)
        self.max_velocity_scaling = float(self.get_parameter("max_velocity_scaling").value)
        self.max_acceleration_scaling = float(self.get_parameter("max_acceleration_scaling").value)
        self.allowed_planning_time_sec = float(self.get_parameter("allowed_planning_time_sec").value)
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)

        self.offset_x = float(self.get_parameter("offset_x").value)
        self.offset_y = float(self.get_parameter("offset_y").value)
        self.offset_z = float(self.get_parameter("offset_z").value)

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
        self.wait_for_server_sec = float(self.get_parameter("wait_for_server_sec").value)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.move_action_client = ActionClient(self, MoveGroup, self.move_action_name)

        self.marker_sub = self.create_subscription(
            Marker, self.marker_topic, self._marker_cb, 10
        )
        self.cmd_sub = self.create_subscription(
            String, self.cmd_topic, self._cmd_cb, 10
        )

        self._last_marker_point: Optional[PointStamped] = None
        self._last_marker_rx_time = None
        self._active_goal = False
        self._goal_handle = None

        self.get_logger().info(f"[manager] marker_topic={self.marker_topic}")
        self.get_logger().info(f"[manager] cmd_topic={self.cmd_topic}")
        self.get_logger().info(f"[manager] move_action={self.move_action_name}")
        self.get_logger().info(
            "[manager] commands: go|move|exec, status, clear, cancel"
        )

    def _marker_cb(self, msg: Marker) -> None:
        if msg.action != Marker.ADD:
            return

        p = PointStamped()
        p.header = msg.header
        p.point.x = msg.pose.position.x
        p.point.y = msg.pose.position.y
        p.point.z = msg.pose.position.z

        self._last_marker_point = p
        self._last_marker_rx_time = self.get_clock().now()

    def _cmd_cb(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if not cmd:
            return

        if cmd in ("go", "move", "exec", "execute"):
            self._execute_last_marker()
            return
        if cmd == "status":
            self._print_status()
            return
        if cmd == "clear":
            self._last_marker_point = None
            self._last_marker_rx_time = None
            self.get_logger().info("[manager] cleared last marker")
            return
        if cmd == "cancel":
            self._cancel_active_goal()
            return

        self.get_logger().warn(f"[manager] unknown cmd='{cmd}'")

    def _print_status(self) -> None:
        if self._last_marker_point is None or self._last_marker_rx_time is None:
            self.get_logger().info("[manager] no marker cached")
            return

        age_sec = (
            (self.get_clock().now() - self._last_marker_rx_time).nanoseconds / 1e9
        )
        p = self._last_marker_point.point
        self.get_logger().info(
            f"[manager] last marker: frame={self._last_marker_point.header.frame_id}, "
            f"xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), age={age_sec:.2f}s"
        )

    def _cancel_active_goal(self) -> None:
        if self._goal_handle is None:
            self.get_logger().info("[manager] no active goal to cancel")
            return
        self._goal_handle.cancel_goal_async()
        self.get_logger().info("[manager] cancel requested")

    def _execute_last_marker(self) -> None:
        if self._active_goal:
            self.get_logger().warn("[manager] move is already running")
            return

        if self._last_marker_point is None or self._last_marker_rx_time is None:
            self.get_logger().warn("[manager] no cached marker yet")
            return

        age_sec = (
            (self.get_clock().now() - self._last_marker_rx_time).nanoseconds / 1e9
        )
        if age_sec > self.marker_timeout_sec:
            self.get_logger().warn(
                f"[manager] cached marker too old ({age_sec:.2f}s > {self.marker_timeout_sec:.2f}s)"
            )
            return

        target = self._transform_to_target_frame(self._last_marker_point)
        if target is None:
            return

        if not self.move_action_client.wait_for_server(timeout_sec=self.wait_for_server_sec):
            self.get_logger().error(
                f"[manager] move action server not ready: {self.move_action_name}"
            )
            return

        goal = self._build_goal(target)
        self._active_goal = True
        send_future = self.move_action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _transform_to_target_frame(self, point: PointStamped) -> Optional[PointStamped]:
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
                f"[manager] TF failed: src={point.header.frame_id}, dst={self.target_frame}, err={exc}"
            )
            return None

    def _build_goal(self, target_point: PointStamped) -> MoveGroup.Goal:
        x = target_point.point.x + self.offset_x
        y = target_point.point.y + self.offset_y
        z = target_point.point.z + self.offset_z

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
        constraints.orientation_constraints.append(orientation_constraint)

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

        self.get_logger().info(
            f"[manager] send goal: frame={self.target_frame}, xyz=({x:.3f}, {y:.3f}, {z:.3f})"
        )
        return goal

    def _goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._active_goal = False
            self.get_logger().error(f"[manager] send goal failed: {exc}")
            return

        if not goal_handle.accepted:
            self._active_goal = False
            self.get_logger().warn("[manager] goal rejected")
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)
        self.get_logger().info("[manager] goal accepted")

    def _result_cb(self, future) -> None:
        self._active_goal = False
        self._goal_handle = None

        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().error(f"[manager] get result failed: {exc}")
            return

        result = wrapped.result
        status = wrapped.status
        code = result.error_code.val

        ok = status == GoalStatus.STATUS_SUCCEEDED and code == MoveItErrorCodes.SUCCESS
        if ok:
            self.get_logger().info("[manager] execution succeeded")
            return

        self.get_logger().warn(
            f"[manager] execution failed: status={status}, moveit_error={code}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarkerMoveItCommander()
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