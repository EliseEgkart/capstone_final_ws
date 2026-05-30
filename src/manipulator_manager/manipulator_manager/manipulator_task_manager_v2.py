#!/usr/bin/env python3
"""
Top-level manipulator task FSM v2.

This node follows the v1 task-manager structure, but delegates the button stage
to marker_prepress_commander_v2 profiles. A button command is executed once,
the arm returns home, and only then the external *_BTN_DONE result is published.
Repeated copies of the same completed button command only republish DONE and do
not start arm motion again.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Int32, String


CMD_OUTSIDE_BTN_FRONT = "OUTSIDE_BTN_FRONT"
CMD_INSIDE_BTN_FRONT = "INSIDE_BTN_FRONT"
CMD_INSIDE_B1_BTN_FRONT = "INSIDE_B1_BTN_FRONT"
CMD_INSIDE_3F_BTN_FRONT = "INSIDE_3F_BTN_FRONT"
CMD_INSIDE_B1_BTN_RIGHT = "INSIDE_B1_BTN_RIGHT"
CMD_INSIDE_3F_BTN_RIGHT = "INSIDE_3F_BTN_RIGHT"
CMD_DESTINATION_UNLOAD = "DESTINATION_UNLOAD"
CMD_HOME = "HOME"
CMD_CANCEL = "CANCEL"
CMD_STATUS = "STATUS"
CMD_RESET = "RESET"

BUTTON_TASK_COMMANDS = (
    CMD_OUTSIDE_BTN_FRONT,
    CMD_INSIDE_BTN_FRONT,
    CMD_INSIDE_B1_BTN_FRONT,
    CMD_INSIDE_3F_BTN_FRONT,
    CMD_INSIDE_B1_BTN_RIGHT,
    CMD_INSIDE_3F_BTN_RIGHT,
)

RESULT_OUTSIDE_BTN_DONE = "OUTSIDE_BTN_DONE"
RESULT_INSIDE_BTN_DONE = "INSIDE_BTN_DONE"
RESULT_INSIDE_B1_BTN_DONE = "INSIDE_B1_BTN_DONE"
RESULT_INSIDE_3F_BTN_DONE = "INSIDE_3F_BTN_DONE"
RESULT_UNLOAD_DONE = "UNLOAD_DONE"
RESULT_HOME_DONE = "HOME_DONE"
RESULT_CANCELLED = "CANCELLED"

ARM_OUTSIDE_SCAN = "outside_scan"
ARM_INSIDE_SCAN = "inside_scan"
ARM_HOME = "home"
ARM_CANCEL = "cancel"

PREPRESS_CLEAR = "clear"
PREPRESS_CANCEL = "cancel"

PERCEPTION_OUTSIDE_DOWN = "OUTSIDE_DOWN"
PERCEPTION_INSIDE_B1 = "INSIDE_B1"
PERCEPTION_INSIDE_3F = "INSIDE_3F"


class ManipulatorTaskManagerV2(Node):
    def __init__(self) -> None:
        super().__init__("manipulator_task_manager_v2")

        self.declare_parameter("task_cmd_topic", "/manipulator_task_cmd")
        self.declare_parameter("task_result_topic", "/manipulator_task_result")
        self.declare_parameter("task_state_topic", "/manipulator_task_state")
        self.declare_parameter("task_error_topic", "/manipulator_task_error")

        self.declare_parameter("arm_flag_topic", "/arm_pose_commander_v2/flag")
        self.declare_parameter("arm_done_topic", "/arm_pose_commander_v2/done")

        self.declare_parameter(
            "prepress_cmd_topic",
            "/marker_prepress_commander_v2/cmd",
        )
        self.declare_parameter(
            "prepress_result_topic",
            "/marker_prepress_commander_v2/result",
        )

        self.declare_parameter(
            "perception_target_topic",
            "/manipulator_perception/target_button",
        )
        self.declare_parameter("perception_outside_down", PERCEPTION_OUTSIDE_DOWN)
        self.declare_parameter("perception_inside_b1", PERCEPTION_INSIDE_B1)
        self.declare_parameter("perception_inside_3f", PERCEPTION_INSIDE_3F)

        self.declare_parameter("mcu_cmd_topic", "/mcu/cmd")
        self.declare_parameter("mcu_result_topic", "/mcu/result")
        self.declare_parameter("mcu_unload_cmd", "UNLOAD")
        self.declare_parameter("mcu_unload_done", "UNLOAD_DONE")
        self.declare_parameter("mcu_unload_failed", "UNLOAD_FAILED")
        self.declare_parameter("mcu_cancel_cmd", "CANCEL")
        self.declare_parameter(
            "cmd_pos_flag_topic",
            "/manipulator_hardware/cmd_pos_flag",
        )
        self.declare_parameter("unload_prepare_cmd_pos_flag", 3)
        self.declare_parameter("unload_cmd_pos_flag", 2)
        self.declare_parameter("publish_legacy_mcu_unload_cmd", False)

        self.declare_parameter("unload_wait_for_result", True)
        self.declare_parameter("unload_assume_done_delay_sec", 5.0)

        self.declare_parameter("return_home_after_prepress", True)
        self.declare_parameter("return_home_after_unload", True)
        self.declare_parameter("complete_button_on_prepress_failure", True)
        # Kept for config compatibility. The safe v2 path keeps this false.
        self.declare_parameter("publish_button_done_on_prepress_start", False)
        self.declare_parameter("button_done_dedup_sec", -1.0)
        self.declare_parameter("marker_settle_sec", 3.0)

        self.declare_parameter("outside_align_timeout_sec", 12.0)
        self.declare_parameter("inside_align_timeout_sec", 12.0)
        self.declare_parameter("prepress_timeout_sec", 25.0)
        self.declare_parameter("home_timeout_sec", 12.0)
        self.declare_parameter("unload_timeout_sec", 25.0)
        self.declare_parameter("fsm_tick_sec", 0.05)
        self.declare_parameter("auto_reset_after_failure", True)

        self.task_cmd_topic = str(self.get_parameter("task_cmd_topic").value)
        self.task_result_topic = str(self.get_parameter("task_result_topic").value)
        self.task_state_topic = str(self.get_parameter("task_state_topic").value)
        self.task_error_topic = str(self.get_parameter("task_error_topic").value)

        self.arm_flag_topic = str(self.get_parameter("arm_flag_topic").value)
        self.arm_done_topic = str(self.get_parameter("arm_done_topic").value)
        self.prepress_cmd_topic = str(self.get_parameter("prepress_cmd_topic").value)
        self.prepress_result_topic = str(
            self.get_parameter("prepress_result_topic").value
        )

        self.perception_target_topic = str(
            self.get_parameter("perception_target_topic").value
        )
        self.perception_outside_down = str(
            self.get_parameter("perception_outside_down").value
        )
        self.perception_inside_b1 = str(
            self.get_parameter("perception_inside_b1").value
        )
        self.perception_inside_3f = str(
            self.get_parameter("perception_inside_3f").value
        )

        self.mcu_cmd_topic = str(self.get_parameter("mcu_cmd_topic").value)
        self.mcu_result_topic = str(self.get_parameter("mcu_result_topic").value)
        self.mcu_unload_cmd = str(self.get_parameter("mcu_unload_cmd").value)
        self.mcu_unload_done = str(self.get_parameter("mcu_unload_done").value)
        self.mcu_unload_failed = str(self.get_parameter("mcu_unload_failed").value)
        self.mcu_cancel_cmd = str(self.get_parameter("mcu_cancel_cmd").value)
        self.cmd_pos_flag_topic = str(self.get_parameter("cmd_pos_flag_topic").value)
        self.unload_prepare_cmd_pos_flag = int(
            self.get_parameter("unload_prepare_cmd_pos_flag").value
        )
        self.unload_cmd_pos_flag = int(self.get_parameter("unload_cmd_pos_flag").value)
        self.publish_legacy_mcu_unload_cmd = bool(
            self.get_parameter("publish_legacy_mcu_unload_cmd").value
        )

        self.unload_wait_for_result = bool(
            self.get_parameter("unload_wait_for_result").value
        )
        self.unload_assume_done_delay_sec = float(
            self.get_parameter("unload_assume_done_delay_sec").value
        )
        self.return_home_after_prepress = bool(
            self.get_parameter("return_home_after_prepress").value
        )
        self.return_home_after_unload = bool(
            self.get_parameter("return_home_after_unload").value
        )
        self.complete_button_on_prepress_failure = bool(
            self.get_parameter("complete_button_on_prepress_failure").value
        )
        self.publish_button_done_on_prepress_start = bool(
            self.get_parameter("publish_button_done_on_prepress_start").value
        )
        self.button_done_dedup_sec = float(
            self.get_parameter("button_done_dedup_sec").value
        )
        self.marker_settle_sec = float(self.get_parameter("marker_settle_sec").value)
        self.outside_align_timeout_sec = float(
            self.get_parameter("outside_align_timeout_sec").value
        )
        self.inside_align_timeout_sec = float(
            self.get_parameter("inside_align_timeout_sec").value
        )
        self.prepress_timeout_sec = float(
            self.get_parameter("prepress_timeout_sec").value
        )
        self.home_timeout_sec = float(self.get_parameter("home_timeout_sec").value)
        self.unload_timeout_sec = float(self.get_parameter("unload_timeout_sec").value)
        self.fsm_tick_sec = float(self.get_parameter("fsm_tick_sec").value)
        self.auto_reset_after_failure = bool(
            self.get_parameter("auto_reset_after_failure").value
        )

        self._state = "IDLE"
        self._active_task: Optional[str] = None
        self._active_profile: Optional[str] = None
        self._current_button_done_result: Optional[str] = None
        self._deadline = None
        self._delay_until = None
        self._pending_result_after_home: Optional[str] = None
        self._last_completed_button_task: Optional[str] = None
        self._last_completed_button_result: Optional[str] = None
        self._last_completed_button_time = None

        self.task_cmd_sub = self.create_subscription(
            String,
            self.task_cmd_topic,
            self._task_cmd_cb,
            10,
        )
        self.arm_done_sub = self.create_subscription(
            String,
            self.arm_done_topic,
            self._arm_done_cb,
            10,
        )
        self.prepress_result_sub = self.create_subscription(
            String,
            self.prepress_result_topic,
            self._prepress_result_cb,
            10,
        )
        self.mcu_result_sub = self.create_subscription(
            String,
            self.mcu_result_topic,
            self._mcu_result_cb,
            10,
        )

        self.task_result_pub = self.create_publisher(
            String,
            self.task_result_topic,
            10,
        )
        self.task_state_pub = self.create_publisher(String, self.task_state_topic, 10)
        self.task_error_pub = self.create_publisher(String, self.task_error_topic, 10)
        self.arm_flag_pub = self.create_publisher(String, self.arm_flag_topic, 10)
        self.prepress_cmd_pub = self.create_publisher(
            String,
            self.prepress_cmd_topic,
            10,
        )
        self.perception_target_pub = self.create_publisher(
            String,
            self.perception_target_topic,
            10,
        )
        self.mcu_cmd_pub = self.create_publisher(String, self.mcu_cmd_topic, 10)
        self.cmd_pos_flag_pub = self.create_publisher(
            Int32,
            self.cmd_pos_flag_topic,
            10,
        )

        self.fsm_timer = self.create_timer(self.fsm_tick_sec, self._fsm_tick)

        self.get_logger().info("[task_manager_v2] ready")
        self.get_logger().info(f"[task_manager_v2] task_cmd_topic={self.task_cmd_topic}")
        self.get_logger().info(
            f"[task_manager_v2] task_result_topic={self.task_result_topic}"
        )
        self.get_logger().info(f"[task_manager_v2] arm_flag_topic={self.arm_flag_topic}")
        self.get_logger().info(f"[task_manager_v2] arm_done_topic={self.arm_done_topic}")
        self.get_logger().info(
            f"[task_manager_v2] prepress_cmd_topic={self.prepress_cmd_topic}"
        )
        self.get_logger().info(
            f"[task_manager_v2] cmd_pos_flag_topic={self.cmd_pos_flag_topic}"
        )
        self.get_logger().info(
            "[task_manager_v2] button result policy: execute once, home, then DONE"
        )
        self._publish_state()

    def _task_cmd_cb(self, msg: String) -> None:
        cmd = msg.data.strip().upper()
        if not cmd:
            return

        if cmd == CMD_STATUS:
            self._publish_status()
            return
        if cmd == CMD_CANCEL:
            self._cancel_current_task()
            return
        if cmd == CMD_RESET:
            self._clear_completed_button_latch()
            self._reset_to_idle()
            self._publish_result("RESET_DONE")
            return
        if cmd == CMD_HOME:
            self._start_home_only()
            return
        if cmd == CMD_OUTSIDE_BTN_FRONT:
            self._start_button_task(
                task=cmd,
                perception_target=self.perception_outside_down,
                profile="outside_front",
                done_result=RESULT_OUTSIDE_BTN_DONE,
                align_state="OUTSIDE_ALIGNING",
                arm_scan=ARM_OUTSIDE_SCAN,
            )
            return
        if cmd == CMD_INSIDE_BTN_FRONT:
            self._start_inside_button_task(
                task=cmd,
                perception_target=self.perception_inside_b1,
                profile="inside_b1_front",
                done_result=RESULT_INSIDE_BTN_DONE,
            )
            return
        if cmd == CMD_INSIDE_B1_BTN_FRONT:
            self._start_inside_button_task(
                task=cmd,
                perception_target=self.perception_inside_b1,
                profile="inside_b1_front",
                done_result=RESULT_INSIDE_B1_BTN_DONE,
            )
            return
        if cmd == CMD_INSIDE_3F_BTN_FRONT:
            self._start_inside_button_task(
                task=cmd,
                perception_target=self.perception_inside_3f,
                profile="inside_front",
                done_result=RESULT_INSIDE_3F_BTN_DONE,
            )
            return
        if cmd == CMD_INSIDE_B1_BTN_RIGHT:
            self._start_inside_button_task(
                task=cmd,
                perception_target=self.perception_inside_b1,
                profile="inside_b1_right",
                done_result=RESULT_INSIDE_B1_BTN_DONE,
            )
            return
        if cmd == CMD_INSIDE_3F_BTN_RIGHT:
            self._start_inside_button_task(
                task=cmd,
                perception_target=self.perception_inside_3f,
                profile="inside_right",
                done_result=RESULT_INSIDE_3F_BTN_DONE,
            )
            return
        if cmd == CMD_DESTINATION_UNLOAD:
            self._start_destination_unload_task()
            return

        self._publish_error(f"UNKNOWN_COMMAND:{cmd}")

    def _start_inside_button_task(
        self,
        task: str,
        perception_target: str,
        profile: str,
        done_result: str,
    ) -> None:
        self._start_button_task(
            task=task,
            perception_target=perception_target,
            profile=profile,
            done_result=done_result,
            align_state="INSIDE_ALIGNING",
            arm_scan=ARM_INSIDE_SCAN,
        )

    def _start_button_task(
        self,
        task: str,
        perception_target: str,
        profile: str,
        done_result: str,
        align_state: str,
        arm_scan: str,
    ) -> None:
        if not self._accept_new_task(task):
            return

        self._active_task = task
        self._active_profile = profile
        self._current_button_done_result = done_result

        self._publish_perception_target(perception_target)
        self._publish_prepress_cmd(PREPRESS_CLEAR)

        self._set_state(align_state)
        timeout = (
            self.outside_align_timeout_sec
            if align_state == "OUTSIDE_ALIGNING"
            else self.inside_align_timeout_sec
        )
        self._set_deadline(timeout)
        self._publish_arm_cmd(arm_scan)

    def _arm_done_cb(self, msg: String) -> None:
        text = msg.data.strip().lower()
        if not text:
            return
        self.get_logger().info(f"[task_manager_v2] arm_done='{text}'")

        if self._state == "OUTSIDE_ALIGNING":
            if text == "outside_scan_done":
                self._start_marker_settle("OUTSIDE_MARKER_SETTLE")
            elif self._is_failure(text):
                self._fail_task(f"OUTSIDE_ALIGN_FAILED:{text}")
            return

        if self._state == "INSIDE_ALIGNING":
            if text == "inside_scan_done":
                self._start_marker_settle("INSIDE_MARKER_SETTLE")
            elif self._is_failure(text):
                self._fail_task(f"INSIDE_ALIGN_FAILED:{text}")
            return

        if self._state in ("BUTTON_HOMING", "UNLOAD_HOMING", "HOME_ONLY"):
            if text == "home_done":
                if self._state == "HOME_ONLY":
                    self._reset_to_idle()
                    self._publish_result(RESULT_HOME_DONE)
                    return

                if self._pending_result_after_home is not None:
                    completed_task = self._active_task
                    result = self._pending_result_after_home
                    self._reset_to_idle()
                    self._remember_completed_button_task(completed_task, result)
                    self._publish_result(result)
                    return

                self._reset_to_idle()
                return

            if self._is_failure(text):
                self._publish_error(f"HOME_FAILED:{text}")
                self._reset_to_idle()
                return

        self.get_logger().debug(
            f"[task_manager_v2] ignored arm_done='{text}' in state={self._state}"
        )

    def _prepress_result_cb(self, msg: String) -> None:
        text = msg.data.strip().lower()
        if not text:
            return
        self.get_logger().info(f"[task_manager_v2] prepress_result='{text}'")

        if self._state != "BUTTON_PREPRESSING":
            self.get_logger().debug(
                f"[task_manager_v2] ignored prepress_result='{text}' in state={self._state}"
            )
            return

        done_result = self._current_button_done_result or "BUTTON_DONE"
        if text.startswith("prepress_done") or text.startswith("press_done"):
            self._finish_button_attempt(done_result)
            return

        if self._is_failure(text) or text == "cancelled":
            self._publish_error(f"BUTTON_PREPRESS_FAILED:{text}")
            if self.complete_button_on_prepress_failure:
                self._finish_button_attempt(done_result)
                return
            self._fail_task(f"BUTTON_PREPRESS_FAILED:{text}")

    def _mcu_result_cb(self, msg: String) -> None:
        text = msg.data.strip().upper()
        if not text:
            return
        self.get_logger().info(f"[task_manager_v2] mcu_result='{text}'")

        if self._state != "UNLOADING":
            self.get_logger().debug(
                f"[task_manager_v2] ignored mcu_result='{text}' in state={self._state}"
            )
            return

        if text == self.mcu_unload_done.upper():
            self._finish_unload_success()
            return
        if text == self.mcu_unload_failed.upper() or "FAIL" in text:
            self._fail_task(f"UNLOAD_FAILED:{text}")

    def _start_marker_settle(self, settle_state: str) -> None:
        self._set_state(settle_state)
        self._delay_until = self.get_clock().now() + Duration(
            seconds=self.marker_settle_sec
        )
        self._set_deadline(max(2.0, self.marker_settle_sec + 1.0))

    def _start_prepress(self) -> None:
        if self._active_profile is None:
            self._fail_task("NO_ACTIVE_PREPRESS_PROFILE")
            return

        self._set_state("BUTTON_PREPRESSING")
        self._set_deadline(self.prepress_timeout_sec)
        self.get_logger().info(
            f"[task_manager_v2] selected prepress profile='{self._active_profile}' "
            f"for active_task={self._active_task}"
        )
        self._publish_prepress_cmd(self._active_profile)

        if self.publish_button_done_on_prepress_start:
            self.get_logger().warn(
                "[task_manager_v2] publish_button_done_on_prepress_start is enabled; "
                "this bypasses the home-before-DONE safety policy"
            )
            self._publish_result(self._current_button_done_result or "BUTTON_DONE")

    def _finish_button_attempt(self, done_result: str) -> None:
        if self.return_home_after_prepress:
            self._start_home("BUTTON_HOMING", done_result)
            return

        completed_task = self._active_task
        self._reset_to_idle()
        self._remember_completed_button_task(completed_task, done_result)
        self._publish_result(done_result)

    def _start_destination_unload_task(self) -> None:
        if not self._accept_new_task(CMD_DESTINATION_UNLOAD):
            return

        self._active_task = CMD_DESTINATION_UNLOAD
        self._set_state("UNLOADING")
        self._set_deadline(self.unload_timeout_sec)
        self._publish_cmd_pos_flag(self.unload_prepare_cmd_pos_flag)
        self._publish_cmd_pos_flag(self.unload_cmd_pos_flag)
        if self.publish_legacy_mcu_unload_cmd:
            self._publish_mcu_cmd(self.mcu_unload_cmd)
        if not self.unload_wait_for_result:
            self._delay_until = self.get_clock().now() + Duration(
                seconds=self.unload_assume_done_delay_sec
            )

    def _start_home_only(self) -> None:
        if not self._accept_new_task(CMD_HOME):
            return

        self._active_task = CMD_HOME
        self._start_home("HOME_ONLY", RESULT_HOME_DONE)

    def _start_home(self, state: str, result_after_home: Optional[str] = None) -> None:
        self._pending_result_after_home = result_after_home
        self._set_state(state)
        self._set_deadline(self.home_timeout_sec)
        self._publish_arm_cmd(ARM_HOME)

    def _finish_unload_success(self) -> None:
        if self.return_home_after_unload:
            self._start_home("UNLOAD_HOMING", RESULT_UNLOAD_DONE)
            return

        self._reset_to_idle()
        self._publish_result(RESULT_UNLOAD_DONE)

    def _fsm_tick(self) -> None:
        now = self.get_clock().now()
        if self._delay_until is not None and now >= self._delay_until:
            self._delay_until = None

            if self._state in ("OUTSIDE_MARKER_SETTLE", "INSIDE_MARKER_SETTLE"):
                self._start_prepress()
                return

            if self._state == "UNLOADING" and not self.unload_wait_for_result:
                self._finish_unload_success()
                return

        if self._deadline is not None and now >= self._deadline:
            self._handle_timeout()

    def _handle_timeout(self) -> None:
        state = self._state

        if state == "OUTSIDE_ALIGNING":
            self._fail_task("OUTSIDE_ALIGN_TIMEOUT")
            return
        if state == "INSIDE_ALIGNING":
            self._fail_task("INSIDE_ALIGN_TIMEOUT")
            return
        if state == "OUTSIDE_MARKER_SETTLE":
            self._fail_task("OUTSIDE_MARKER_SETTLE_TIMEOUT")
            return
        if state == "INSIDE_MARKER_SETTLE":
            self._fail_task("INSIDE_MARKER_SETTLE_TIMEOUT")
            return
        if state == "BUTTON_PREPRESSING":
            self._publish_error("BUTTON_PREPRESS_TIMEOUT")
            if self.complete_button_on_prepress_failure:
                self._publish_prepress_cmd(PREPRESS_CANCEL)
                self._finish_button_attempt(
                    self._current_button_done_result or "BUTTON_DONE"
                )
                return
            self._fail_task("BUTTON_PREPRESS_TIMEOUT")
            return
        if state == "BUTTON_HOMING":
            self._fail_task("BUTTON_HOMING_TIMEOUT")
            return
        if state == "UNLOAD_HOMING":
            self._fail_task("UNLOAD_HOMING_TIMEOUT")
            return
        if state == "HOME_ONLY":
            self._fail_task("HOME_ONLY_TIMEOUT")
            return
        if state == "UNLOADING":
            self._fail_task("UNLOAD_TIMEOUT")
            return

        self._fail_task(f"UNKNOWN_TIMEOUT_STATE:{state}")

    def _cancel_current_task(self) -> None:
        if self._state == "IDLE":
            self._publish_result(RESULT_CANCELLED)
            return

        self.get_logger().warn(
            f"[task_manager_v2] cancelling task in state={self._state}"
        )
        self._publish_arm_cmd(ARM_CANCEL)
        self._publish_prepress_cmd(PREPRESS_CANCEL)
        self._publish_mcu_cmd(self.mcu_cancel_cmd)
        self._reset_to_idle()
        self._publish_result(RESULT_CANCELLED)

    def _fail_task(self, reason: str) -> None:
        self.get_logger().warn(f"[task_manager_v2] task failed: {reason}")
        self._publish_arm_cmd(ARM_CANCEL)
        self._publish_prepress_cmd(PREPRESS_CANCEL)
        self._set_state("ERROR")
        self._publish_error(reason)
        self._publish_result(f"FAILED:{reason}")
        if self.auto_reset_after_failure:
            self._reset_to_idle()

    def _reset_to_idle(self) -> None:
        self._active_task = None
        self._active_profile = None
        self._current_button_done_result = None
        self._deadline = None
        self._delay_until = None
        self._pending_result_after_home = None
        self._set_state("IDLE")

    def _accept_new_task(self, task_name: str) -> bool:
        if self._state != "IDLE":
            self._publish_result(
                f"BUSY:CURRENT_STATE={self._state},CURRENT_TASK={self._active_task}"
            )
            self.get_logger().warn(
                f"[task_manager_v2] rejected task={task_name}, "
                f"state={self._state}, active_task={self._active_task}"
            )
            return False

        if self._is_completed_button_task_latched(task_name):
            self._publish_result(self._last_completed_button_result or "BUTTON_DONE")
            self.get_logger().info(
                f"[task_manager_v2] repeated completed task={task_name}; "
                "republished DONE without arm motion"
            )
            return False

        if task_name != self._last_completed_button_task:
            self._clear_completed_button_latch()

        return True

    def _remember_completed_button_task(
        self,
        task: Optional[str],
        result: Optional[str],
    ) -> None:
        if task not in BUTTON_TASK_COMMANDS or result is None:
            return
        self._last_completed_button_task = task
        self._last_completed_button_result = result
        self._last_completed_button_time = self.get_clock().now()

    def _clear_completed_button_latch(self) -> None:
        self._last_completed_button_task = None
        self._last_completed_button_result = None
        self._last_completed_button_time = None

    def _is_completed_button_task_latched(self, task_name: str) -> bool:
        if (
            task_name != self._last_completed_button_task
            or self._last_completed_button_result is None
            or self._last_completed_button_time is None
        ):
            return False

        if self.button_done_dedup_sec < 0.0:
            return True

        age = (
            self.get_clock().now() - self._last_completed_button_time
        ).nanoseconds / 1e9
        return age <= self.button_done_dedup_sec

    def _publish_perception_target(self, text: str) -> None:
        self._publish_string(self.perception_target_pub, text)
        self.get_logger().info(f"[task_manager_v2] perception_target='{text}'")

    def _publish_arm_cmd(self, text: str) -> None:
        self._publish_string(self.arm_flag_pub, text)
        self.get_logger().info(f"[task_manager_v2] arm_cmd='{text}'")

    def _publish_prepress_cmd(self, text: str) -> None:
        self._publish_string(self.prepress_cmd_pub, text)
        self.get_logger().info(f"[task_manager_v2] prepress_cmd='{text}'")

    def _publish_mcu_cmd(self, text: str) -> None:
        self._publish_string(self.mcu_cmd_pub, text)
        self.get_logger().info(f"[task_manager_v2] mcu_cmd='{text}'")

    def _publish_cmd_pos_flag(self, flag: int) -> None:
        msg = Int32()
        msg.data = flag
        self.cmd_pos_flag_pub.publish(msg)
        self.get_logger().info(f"[task_manager_v2] cmd_pos_flag={flag}")

    def _publish_result(self, text: str) -> None:
        self._publish_string(self.task_result_pub, text)
        self.get_logger().info(f"[task_manager_v2] result='{text}'")

    def _publish_error(self, text: str) -> None:
        self._publish_string(self.task_error_pub, text)
        self.get_logger().warn(f"[task_manager_v2] error='{text}'")

    def _publish_state(self) -> None:
        self._publish_string(self.task_state_pub, self._state)

    def _publish_status(self) -> None:
        status = (
            f"STATUS_V2:STATE={self._state},"
            f"ACTIVE_TASK={self._active_task},"
            f"ACTIVE_PROFILE={self._active_profile},"
            f"CURRENT_BUTTON_DONE_RESULT={self._current_button_done_result},"
            f"PENDING_RESULT_AFTER_HOME={self._pending_result_after_home},"
            f"LAST_COMPLETED_BUTTON_TASK={self._last_completed_button_task},"
            f"LAST_COMPLETED_BUTTON_RESULT={self._last_completed_button_result},"
            f"RETURN_HOME_AFTER_PREPRESS={self.return_home_after_prepress},"
            f"COMPLETE_BUTTON_ON_PREPRESS_FAILURE={self.complete_button_on_prepress_failure}"
        )
        self._publish_result(status)

    def _set_state(self, state: str) -> None:
        if self._state != state:
            self.get_logger().info(f"[task_manager_v2] state: {self._state} -> {state}")
        self._state = state
        self._publish_state()

    def _set_deadline(self, seconds: float) -> None:
        self._deadline = self.get_clock().now() + Duration(seconds=float(seconds))

    @staticmethod
    def _publish_string(pub, text: str) -> None:
        msg = String()
        msg.data = text
        pub.publish(msg)

    @staticmethod
    def _is_failure(text: str) -> bool:
        t = text.lower()
        return (
            "fail" in t
            or "failed" in t
            or "rejected" in t
            or "abort" in t
            or "timeout" in t
            or "error" in t
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManipulatorTaskManagerV2()
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
