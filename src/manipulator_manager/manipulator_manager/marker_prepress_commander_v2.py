#!/usr/bin/env python3
"""
Marker-based pre-press commander v2.

The default behavior stops at the pre-contact target. When enable_press is
true, the final press reuses the same approach vector computed from the marker
sample instead of collecting a new marker near the button.
"""

from __future__ import annotations

from collections import deque
from math import sqrt
from statistics import median
from typing import Deque, Dict, List, Optional, Tuple

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


class MarkerPrepressCommanderV2(Node):
    def __init__(self) -> None:
        super().__init__("marker_prepress_commander_v2")

        self.declare_parameter("marker_topic", "/object_3d_marker")
        self.declare_parameter("cmd_topic", "/marker_prepress_commander_v2/cmd")
        self.declare_parameter("result_topic", "/marker_prepress_commander_v2/result")
        self.declare_parameter("state_topic", "/marker_prepress_commander_v2/state")
        self.declare_parameter("move_action_name", "/move_action")

        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("ee_link", "ee_link")
        self.declare_parameter("goal_frame", "link1")

        self.declare_parameter("marker_timeout_sec", 3.0)
        self.declare_parameter("marker_collect_sec", 0.7)
        self.declare_parameter("preferred_marker_samples", 5)
        self.declare_parameter("min_marker_samples", 1)
        self.declare_parameter("marker_buffer_size", 20)
        self.declare_parameter("allow_recent_marker", False)
        self.declare_parameter("collect_tick_sec", 0.05)

        self.declare_parameter("position_tolerance_m", 0.008)
        self.declare_parameter("max_velocity_scaling", 0.15)
        self.declare_parameter("max_acceleration_scaling", 0.15)
        self.declare_parameter("allowed_planning_time_sec", 3.0)
        self.declare_parameter("num_planning_attempts", 5)

        self.declare_parameter("use_orientation_constraint", True)
        self.declare_parameter("goal_qx", 0.0)
        self.declare_parameter("goal_qy", 0.0)
        self.declare_parameter("goal_qz", 0.0)
        self.declare_parameter("goal_qw", 1.0)
        self.declare_parameter("ori_tol_x", 3.14)
        self.declare_parameter("ori_tol_y", 3.14)
        self.declare_parameter("ori_tol_z", 3.14)

        self.declare_parameter("plan_only", False)
        self.declare_parameter("replan", True)
        self.declare_parameter("replan_attempts", 1)
        self.declare_parameter("replan_delay_sec", 1.0)
        self.declare_parameter("wait_for_server_sec", 2.0)

        self.declare_parameter("enable_press", False)
        self.declare_parameter("press_hold_sec", 0.2)
        self.declare_parameter("release_after_press", True)
        self.declare_parameter("press_max_velocity_scaling", 0.08)
        self.declare_parameter("press_max_acceleration_scaling", 0.08)

        self.declare_parameter(
            "profile_names",
            ["outside_front", "inside_front", "inside_right"],
        )

        profile_defaults = {
            "outside_front": {
                "approach_frame": "camera_link",
                "approach_axis": "x",
                "approach_sign": -1.0,
                "standoff_m": 0.035,
                "offset": (0.0, 0.0, 0.0),
                "approach_mode": "radial_from_frame",
                "radial_frame": "link1",
                "radial_plane": "xy",
                "radial_min_norm_m": 0.05,
                "press_travel_m": 0.045,
            },
            "inside_front": {
                "approach_frame": "camera_link",
                "approach_axis": "x",
                "approach_sign": -1.0,
                "standoff_m": 0.035,
                "offset": (0.0, 0.0, 0.0),
                "approach_mode": "radial_from_frame",
                "radial_frame": "link1",
                "radial_plane": "xy",
                "radial_min_norm_m": 0.05,
                "press_travel_m": 0.045,
            },
            "inside_right": {
                "approach_frame": "camera_link",
                "approach_axis": "y",
                "approach_sign": -1.0,
                "standoff_m": 0.035,
                "offset": (0.0, 0.0, 0.0),
                "approach_mode": "radial_from_frame",
                "radial_frame": "link1",
                "radial_plane": "xy",
                "radial_min_norm_m": 0.05,
                "press_travel_m": 0.045,
            },
        }
        self.profile_names = [
            str(name) for name in self.get_parameter("profile_names").value
        ]
        for name in self.profile_names:
            defaults = profile_defaults.get(
                name,
                profile_defaults["inside_front"],
            )
            off_x, off_y, off_z = defaults["offset"]
            self.declare_parameter(
                f"{name}_approach_frame",
                defaults["approach_frame"],
            )
            self.declare_parameter(f"{name}_approach_axis", defaults["approach_axis"])
            self.declare_parameter(f"{name}_approach_sign", defaults["approach_sign"])
            self.declare_parameter(f"{name}_standoff_m", defaults["standoff_m"])
            self.declare_parameter(f"{name}_offset_x", off_x)
            self.declare_parameter(f"{name}_offset_y", off_y)
            self.declare_parameter(f"{name}_offset_z", off_z)
            self.declare_parameter(f"{name}_approach_mode", defaults["approach_mode"])
            self.declare_parameter(f"{name}_radial_frame", defaults["radial_frame"])
            self.declare_parameter(f"{name}_radial_plane", defaults["radial_plane"])
            self.declare_parameter(
                f"{name}_radial_min_norm_m",
                defaults["radial_min_norm_m"],
            )
            self.declare_parameter(f"{name}_press_travel_m", defaults["press_travel_m"])

        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.move_action_name = str(self.get_parameter("move_action_name").value)

        self.planning_group = str(self.get_parameter("planning_group").value)
        self.ee_link = str(self.get_parameter("ee_link").value)
        self.goal_frame = str(self.get_parameter("goal_frame").value)

        self.marker_timeout_sec = float(self.get_parameter("marker_timeout_sec").value)
        self.marker_collect_sec = float(self.get_parameter("marker_collect_sec").value)
        self.preferred_marker_samples = int(
            self.get_parameter("preferred_marker_samples").value
        )
        self.min_marker_samples = int(self.get_parameter("min_marker_samples").value)
        self.marker_buffer_size = int(self.get_parameter("marker_buffer_size").value)
        self.allow_recent_marker = bool(self.get_parameter("allow_recent_marker").value)
        self.collect_tick_sec = float(self.get_parameter("collect_tick_sec").value)

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

        self.use_orientation_constraint = bool(
            self.get_parameter("use_orientation_constraint").value
        )
        self.goal_qx = float(self.get_parameter("goal_qx").value)
        self.goal_qy = float(self.get_parameter("goal_qy").value)
        self.goal_qz = float(self.get_parameter("goal_qz").value)
        self.goal_qw = float(self.get_parameter("goal_qw").value)
        self.ori_tol_x = float(self.get_parameter("ori_tol_x").value)
        self.ori_tol_y = float(self.get_parameter("ori_tol_y").value)
        self.ori_tol_z = float(self.get_parameter("ori_tol_z").value)

        self.plan_only = bool(self.get_parameter("plan_only").value)
        self.replan = bool(self.get_parameter("replan").value)
        self.replan_attempts = int(self.get_parameter("replan_attempts").value)
        self.replan_delay_sec = float(self.get_parameter("replan_delay_sec").value)
        self.wait_for_server_sec = float(
            self.get_parameter("wait_for_server_sec").value
        )
        self.enable_press = bool(self.get_parameter("enable_press").value)
        self.press_hold_sec = float(self.get_parameter("press_hold_sec").value)
        self.release_after_press = bool(
            self.get_parameter("release_after_press").value
        )
        self.press_max_velocity_scaling = float(
            self.get_parameter("press_max_velocity_scaling").value
        )
        self.press_max_acceleration_scaling = float(
            self.get_parameter("press_max_acceleration_scaling").value
        )

        self.profiles = self._load_profiles()

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.move_action_client = ActionClient(self, MoveGroup, self.move_action_name)

        self.marker_sub = self.create_subscription(
            Marker,
            self.marker_topic,
            self._marker_cb,
            10,
        )
        self.cmd_sub = self.create_subscription(String, self.cmd_topic, self._cmd_cb, 10)
        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self._marker_buffer: Deque[Tuple[object, PointStamped]] = deque(
            maxlen=max(1, self.marker_buffer_size)
        )
        self._state = "IDLE"
        self._active_profile: Optional[str] = None
        self._collection_started = None
        self._active_goal = False
        self._goal_handle = None
        self._motion_stage: Optional[str] = None
        self._hold_until = None
        self._press_frame: Optional[str] = None
        self._press_precontact_point: Optional[PointStamped] = None
        self._press_unit: Optional[Tuple[float, float, float]] = None

        self.collect_timer = self.create_timer(self.collect_tick_sec, self._timer_cb)

        self.get_logger().info("[prepress_v2] ready")
        self.get_logger().info(f"[prepress_v2] profiles={self.profile_names}")
        self.get_logger().info(f"[prepress_v2] marker_topic={self.marker_topic}")
        self.get_logger().info(f"[prepress_v2] cmd_topic={self.cmd_topic}")
        self.get_logger().info(f"[prepress_v2] result_topic={self.result_topic}")
        self.get_logger().info(
            f"[prepress_v2] goal_frame={self.goal_frame}, ee_link={self.ee_link}"
        )
        self._publish_state("IDLE")

    def _load_profiles(self) -> Dict[str, Dict[str, object]]:
        profiles: Dict[str, Dict[str, object]] = {}
        for name in self.profile_names:
            axis = str(self.get_parameter(f"{name}_approach_axis").value).lower()
            if axis not in ("x", "y", "z"):
                raise ValueError(f"{name}_approach_axis must be x, y, or z")
            mode = str(self.get_parameter(f"{name}_approach_mode").value).lower()
            if mode not in ("axis", "radial_from_frame"):
                raise ValueError(f"{name}_approach_mode must be axis or radial_from_frame")
            radial_plane = str(self.get_parameter(f"{name}_radial_plane").value).lower()
            if radial_plane not in ("xy", "xz", "yz", "xyz"):
                raise ValueError(f"{name}_radial_plane must be xy, xz, yz, or xyz")
            profiles[name] = {
                "approach_frame": str(
                    self.get_parameter(f"{name}_approach_frame").value
                ),
                "approach_axis": axis,
                "approach_sign": float(
                    self.get_parameter(f"{name}_approach_sign").value
                ),
                "standoff_m": float(self.get_parameter(f"{name}_standoff_m").value),
                "offset": (
                    float(self.get_parameter(f"{name}_offset_x").value),
                    float(self.get_parameter(f"{name}_offset_y").value),
                    float(self.get_parameter(f"{name}_offset_z").value),
                ),
                "approach_mode": mode,
                "radial_frame": str(self.get_parameter(f"{name}_radial_frame").value),
                "radial_plane": radial_plane,
                "radial_min_norm_m": float(
                    self.get_parameter(f"{name}_radial_min_norm_m").value
                ),
                "press_travel_m": float(
                    self.get_parameter(f"{name}_press_travel_m").value
                ),
            }
        return profiles

    def _marker_cb(self, msg: Marker) -> None:
        if msg.action != Marker.ADD or not msg.header.frame_id:
            return

        p = PointStamped()
        p.header = msg.header
        p.point.x = msg.pose.position.x
        p.point.y = msg.pose.position.y
        p.point.z = msg.pose.position.z
        self._marker_buffer.append((self.get_clock().now(), p))

    def _cmd_cb(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if not cmd:
            return

        if cmd == "status":
            self._print_status()
            return

        if cmd == "clear":
            self._marker_buffer.clear()
            self._clear_press_plan()
            self.get_logger().info("[prepress_v2] cleared marker buffer")
            return

        if cmd in ("cancel", "stop"):
            self._cancel_active_goal()
            self._reset_runtime("IDLE")
            self._publish_result("cancelled")
            return

        profile = self._profile_from_cmd(cmd)
        if profile is None:
            self._publish_result(f"prepress_failed:unknown_profile:{cmd}")
            return

        if self._state != "IDLE" or self._active_goal:
            self._publish_result(f"prepress_failed:busy:{self._state}")
            return

        self._marker_buffer.clear()
        self._active_profile = profile
        self._collection_started = self.get_clock().now()
        self._set_state("COLLECTING")
        self.get_logger().info(f"[prepress_v2] collecting markers for profile={profile}")

    def _timer_cb(self) -> None:
        if self._state == "HOLDING":
            if self._hold_until is not None and self.get_clock().now() >= self._hold_until:
                self._start_release_motion()
            return

        if self._state != "COLLECTING" or self._active_profile is None:
            return

        samples = self._recent_samples()
        elapsed = (
            self.get_clock().now() - self._collection_started
        ).nanoseconds / 1e9

        if len(samples) >= self.preferred_marker_samples or elapsed >= self.marker_collect_sec:
            if len(samples) < self.min_marker_samples:
                self._publish_failed("no_recent_marker")
                return
            self._execute_prepress(self._active_profile, samples)

    def _recent_samples(self) -> List[PointStamped]:
        now = self.get_clock().now()
        out: List[PointStamped] = []
        for rx_time, point in list(self._marker_buffer):
            age = (now - rx_time).nanoseconds / 1e9
            if age > self.marker_timeout_sec:
                continue
            if not self.allow_recent_marker and self._collection_started is not None:
                if rx_time < self._collection_started:
                    continue
            out.append(point)
        return out

    def _execute_prepress(self, profile_name: str, samples: List[PointStamped]) -> None:
        if not self.move_action_client.wait_for_server(
            timeout_sec=self.wait_for_server_sec
        ):
            self._publish_failed(f"move_action_server_not_ready:{self.move_action_name}")
            return

        target = self._prepress_target(profile_name, samples)
        if target is None:
            self._publish_failed("target_transform_failed")
            return

        goal = self._build_goal(target)
        self._active_goal = True
        self._motion_stage = "PREPRESS"
        self._set_state("MOVING")
        send_future = self.move_action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _prepress_target(
        self,
        profile_name: str,
        samples: List[PointStamped],
    ) -> Optional[PointStamped]:
        profile = self.profiles[profile_name]
        approach_frame = str(profile["approach_frame"])
        transformed: List[PointStamped] = []

        for sample in samples:
            p = self._transform_point(sample, approach_frame)
            if p is not None:
                transformed.append(p)

        if not transformed:
            return None

        xs = [p.point.x for p in transformed]
        ys = [p.point.y for p in transformed]
        zs = [p.point.z for p in transformed]

        target = PointStamped()
        target.header.frame_id = approach_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.point.x = float(median(xs))
        target.point.y = float(median(ys))
        target.point.z = float(median(zs))

        off_x, off_y, off_z = profile["offset"]
        target.point.x += float(off_x)
        target.point.y += float(off_y)
        target.point.z += float(off_z)

        self._clear_press_plan()
        if profile["approach_mode"] == "radial_from_frame":
            goal_target = self._apply_radial_standoff(target, profile)
        else:
            self._apply_axis_standoff(target, profile)
            self._store_axis_press_plan(target, profile)
            goal_target = self._transform_point(target, self.goal_frame)

        if goal_target is not None:
            self.get_logger().info(
                f"[prepress_v2] profile={profile_name}, samples={len(transformed)}, "
                f"target[{self.goal_frame}]=({goal_target.point.x:.3f}, "
                f"{goal_target.point.y:.3f}, {goal_target.point.z:.3f})"
            )
        return goal_target

    def _apply_axis_standoff(
        self,
        target: PointStamped,
        profile: Dict[str, object],
    ) -> None:
        axis = str(profile["approach_axis"])
        delta = float(profile["approach_sign"]) * float(profile["standoff_m"])
        if axis == "x":
            target.point.x += delta
        elif axis == "y":
            target.point.y += delta
        else:
            target.point.z += delta

    def _apply_radial_standoff(
        self,
        marker_target: PointStamped,
        profile: Dict[str, object],
    ) -> Optional[PointStamped]:
        radial_frame = str(profile["radial_frame"])
        radial_target = self._transform_point(marker_target, radial_frame)
        if radial_target is None:
            return None

        x = radial_target.point.x
        y = radial_target.point.y
        z = radial_target.point.z
        plane = str(profile["radial_plane"])

        vx = x if "x" in plane else 0.0
        vy = y if "y" in plane else 0.0
        vz = z if "z" in plane else 0.0
        norm = sqrt(vx * vx + vy * vy + vz * vz)
        min_norm = float(profile["radial_min_norm_m"])
        if norm < min_norm:
            self.get_logger().warn(
                f"[prepress_v2] radial vector too small: norm={norm:.4f}, "
                f"min={min_norm:.4f}, frame={radial_frame}, plane={plane}"
            )
            return None

        unit_x = vx / norm
        unit_y = vy / norm
        unit_z = vz / norm
        delta = float(profile["approach_sign"]) * float(profile["standoff_m"])

        radial_target.point.x += unit_x * delta
        radial_target.point.y += unit_y * delta
        radial_target.point.z += unit_z * delta

        self.get_logger().info(
            f"[prepress_v2] radial standoff: frame={radial_frame}, plane={plane}, "
            f"unit=({unit_x:.3f}, {unit_y:.3f}, {unit_z:.3f}), delta={delta:.3f}"
        )
        self._store_press_plan(
            frame_id=radial_frame,
            precontact_point=radial_target,
            unit=(
                -float(profile["approach_sign"]) * unit_x,
                -float(profile["approach_sign"]) * unit_y,
                -float(profile["approach_sign"]) * unit_z,
            ),
        )
        return self._transform_point(radial_target, self.goal_frame)

    def _store_axis_press_plan(
        self,
        precontact_point: PointStamped,
        profile: Dict[str, object],
    ) -> None:
        axis = str(profile["approach_axis"])
        sign = -float(profile["approach_sign"])
        unit = {
            "x": (sign, 0.0, 0.0),
            "y": (0.0, sign, 0.0),
            "z": (0.0, 0.0, sign),
        }[axis]
        self._store_press_plan(
            frame_id=precontact_point.header.frame_id,
            precontact_point=precontact_point,
            unit=unit,
        )

    def _store_press_plan(
        self,
        frame_id: str,
        precontact_point: PointStamped,
        unit: Tuple[float, float, float],
    ) -> None:
        self._press_frame = frame_id
        self._press_precontact_point = self._copy_point(precontact_point)
        self._press_unit = unit
        self.get_logger().info(
            f"[prepress_v2] press vector stored: frame={frame_id}, "
            f"unit=({unit[0]:.3f}, {unit[1]:.3f}, {unit[2]:.3f})"
        )

    def _clear_press_plan(self) -> None:
        self._press_frame = None
        self._press_precontact_point = None
        self._press_unit = None

    def _press_target_from_stored_vector(self, travel_m: float) -> Optional[PointStamped]:
        if (
            self._press_precontact_point is None
            or self._press_unit is None
            or self._press_frame is None
        ):
            return None

        target = self._copy_point(self._press_precontact_point)
        target.header.stamp = self.get_clock().now().to_msg()
        target.point.x += self._press_unit[0] * travel_m
        target.point.y += self._press_unit[1] * travel_m
        target.point.z += self._press_unit[2] * travel_m
        return self._transform_point(target, self.goal_frame)

    def _release_target_from_stored_vector(self) -> Optional[PointStamped]:
        if self._press_precontact_point is None:
            return None
        target = self._copy_point(self._press_precontact_point)
        target.header.stamp = self.get_clock().now().to_msg()
        return self._transform_point(target, self.goal_frame)

    @staticmethod
    def _copy_point(point: PointStamped) -> PointStamped:
        copied = PointStamped()
        copied.header.frame_id = point.header.frame_id
        copied.header.stamp = point.header.stamp
        copied.point.x = point.point.x
        copied.point.y = point.point.y
        copied.point.z = point.point.z
        return copied

    def _transform_point(
        self,
        point: PointStamped,
        target_frame: str,
    ) -> Optional[PointStamped]:
        if point.header.frame_id == target_frame:
            return point
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            return do_transform_point(point, tf)
        except TransformException as exc:
            self.get_logger().warn(
                f"[prepress_v2] TF failed: src={point.header.frame_id}, "
                f"dst={target_frame}, err={exc}"
            )
            return None

    def _build_goal(
        self,
        target_point: PointStamped,
        velocity_scaling: Optional[float] = None,
        acceleration_scaling: Optional[float] = None,
    ) -> MoveGroup.Goal:
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.goal_frame
        position_constraint.link_name = self.ee_link
        position_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance_m]

        region_pose = Pose()
        region_pose.position.x = target_point.point.x
        region_pose.position.y = target_point.point.y
        region_pose.position.z = target_point.point.z
        region_pose.orientation.w = 1.0

        position_constraint.constraint_region.primitives.append(sphere)
        position_constraint.constraint_region.primitive_poses.append(region_pose)

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)

        if self.use_orientation_constraint:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.goal_frame
            orientation_constraint.link_name = self.ee_link
            orientation_constraint.orientation.x = self.goal_qx
            orientation_constraint.orientation.y = self.goal_qy
            orientation_constraint.orientation.z = self.goal_qz
            orientation_constraint.orientation.w = self.goal_qw
            orientation_constraint.absolute_x_axis_tolerance = self.ori_tol_x
            orientation_constraint.absolute_y_axis_tolerance = self.ori_tol_y
            orientation_constraint.absolute_z_axis_tolerance = self.ori_tol_z
            orientation_constraint.weight = 1.0
            constraints.orientation_constraints.append(orientation_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = self.num_planning_attempts
        goal.request.allowed_planning_time = self.allowed_planning_time_sec
        goal.request.max_velocity_scaling_factor = (
            self.max_velocity_scaling
            if velocity_scaling is None
            else float(velocity_scaling)
        )
        goal.request.max_acceleration_scaling_factor = (
            self.max_acceleration_scaling
            if acceleration_scaling is None
            else float(acceleration_scaling)
        )
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = self.plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = self.replan
        goal.planning_options.replan_attempts = self.replan_attempts
        goal.planning_options.replan_delay = self.replan_delay_sec
        return goal

    def _goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._active_goal = False
            self._publish_failed(f"send_goal_failed:{exc}")
            return

        if not goal_handle.accepted:
            self._active_goal = False
            self._publish_failed("goal_rejected")
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)
        self.get_logger().info(f"[prepress_v2] goal accepted: stage={self._motion_stage}")

    def _result_cb(self, future) -> None:
        self._active_goal = False
        self._goal_handle = None

        try:
            wrapped = future.result()
        except Exception as exc:
            self._publish_failed(f"get_result_failed:{exc}")
            return

        result = wrapped.result
        status = wrapped.status
        code = result.error_code.val
        ok = status == GoalStatus.STATUS_SUCCEEDED and code == MoveItErrorCodes.SUCCESS

        if ok:
            self._handle_motion_success()
            return

        self._publish_failed(
            f"execution_failed:stage={self._motion_stage},"
            f"status={self._goal_status_name(status)},moveit_error={code}"
        )

    def _handle_motion_success(self) -> None:
        stage = self._motion_stage
        profile = self._active_profile or "unknown"

        if stage == "PREPRESS":
            if self.enable_press:
                self._start_press_motion()
                return
            self._publish_result(f"prepress_done:{profile}")
            self._reset_runtime("IDLE")
            return

        if stage == "PRESS":
            if self.release_after_press:
                self._motion_stage = "HOLD"
                self._hold_until = self.get_clock().now() + Duration(
                    seconds=self.press_hold_sec
                )
                self._set_state("HOLDING")
                return
            self._publish_result(f"press_done:{profile}")
            self._reset_runtime("IDLE")
            return

        if stage == "RELEASE":
            self._publish_result(f"press_done:{profile}")
            self._reset_runtime("IDLE")
            return

        self._publish_failed(f"unknown_motion_stage:{stage}")

    def _start_press_motion(self) -> None:
        if self._active_profile is None:
            self._publish_failed("press_no_active_profile")
            return

        profile = self.profiles[self._active_profile]
        press_travel_m = float(profile["press_travel_m"])
        if press_travel_m <= 0.0:
            self._publish_failed(f"invalid_press_travel_m:{press_travel_m:.4f}")
            return

        target = self._press_target_from_stored_vector(press_travel_m)
        if target is None:
            self._publish_failed("press_target_from_stored_vector_failed")
            return

        self.get_logger().info(
            f"[prepress_v2] press target[{self.goal_frame}]="
            f"({target.point.x:.3f}, {target.point.y:.3f}, {target.point.z:.3f}), "
            f"travel={press_travel_m:.3f}"
        )
        goal = self._build_goal(
            target,
            velocity_scaling=self.press_max_velocity_scaling,
            acceleration_scaling=self.press_max_acceleration_scaling,
        )
        self._active_goal = True
        self._motion_stage = "PRESS"
        self._set_state("PRESSING")
        send_future = self.move_action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _start_release_motion(self) -> None:
        target = self._release_target_from_stored_vector()
        if target is None:
            self._publish_failed("release_target_from_stored_vector_failed")
            return

        self.get_logger().info(
            f"[prepress_v2] release target[{self.goal_frame}]="
            f"({target.point.x:.3f}, {target.point.y:.3f}, {target.point.z:.3f})"
        )
        self._hold_until = None
        goal = self._build_goal(
            target,
            velocity_scaling=self.press_max_velocity_scaling,
            acceleration_scaling=self.press_max_acceleration_scaling,
        )
        self._active_goal = True
        self._motion_stage = "RELEASE"
        self._set_state("RELEASING")
        send_future = self.move_action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _profile_from_cmd(self, cmd: str) -> Optional[str]:
        aliases = {
            "press_outside": "outside_front",
            "outside": "outside_front",
            "outside_front": "outside_front",
            "press_inside": "inside_front",
            "inside": "inside_front",
            "inside_front": "inside_front",
            "inside_right": "inside_right",
            "right": "inside_right",
        }
        profile = aliases.get(cmd, cmd)
        return profile if profile in self.profiles else None

    def _cancel_active_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self.get_logger().info("[prepress_v2] cancel requested")

    def _print_status(self) -> None:
        self.get_logger().info(
            f"[prepress_v2] state={self._state}, active_profile={self._active_profile}, "
            f"active_goal={self._active_goal}, markers={len(self._marker_buffer)}"
        )

    def _publish_failed(self, reason: str) -> None:
        self.get_logger().warn(f"[prepress_v2] failed: {reason}")
        self._publish_result(f"prepress_failed:{reason}")
        self._reset_runtime("IDLE")

    def _reset_runtime(self, state: str) -> None:
        self._active_profile = None
        self._collection_started = None
        self._active_goal = False
        self._goal_handle = None
        self._motion_stage = None
        self._hold_until = None
        self._clear_press_plan()
        self._set_state(state)

    def _set_state(self, state: str) -> None:
        self._state = state
        self._publish_state(state)

    def _publish_result(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)
        self.get_logger().info(f"[prepress_v2] result='{text}'")

    def _publish_state(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    @staticmethod
    def _goal_status_name(status: int) -> str:
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
    node = MarkerPrepressCommanderV2()
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
