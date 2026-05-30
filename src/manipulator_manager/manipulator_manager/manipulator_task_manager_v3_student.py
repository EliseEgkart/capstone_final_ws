#!/usr/bin/env python3
"""
Student-demo manipulator task manager.

This node intentionally manages only the manipulator-side demo sequence:

* INSIDE_BTN_FRONT: run the same inside elevator button motion used by v2,
  then publish INSIDE_BTN_DONE.
* DESTINATION_UNLOAD: send ESP32 CMD_POS flags 3 then 2 through the hardware
  interface, then publish UNLOAD_DONE.

The external topic contract is the same as manipulator_task_manager_v2:
commands are received on /manipulator_task_cmd and done messages are published
on /manipulator_task_result.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Int32, String


CMD_INSIDE_BTN_FRONT = "INSIDE_BTN_FRONT"
CMD_DESTINATION_UNLOAD = "DESTINATION_UNLOAD"
CMD_HOME = "HOME"
CMD_CANCEL = "CANCEL"
CMD_STATUS = "STATUS"
CMD_RESET = "RESET"

RESULT_INSIDE_BTN_DONE = "INSIDE_BTN_DONE"
RESULT_UNLOAD_DONE = "UNLOAD_DONE"
RESULT_HOME_DONE = "HOME_DONE"
RESULT_CANCELLED = "CANCELLED"

ARM_INSIDE_SCAN = "inside_scan"
ARM_HOME = "home"
ARM_CANCEL = "cancel"

PREPRESS_CLEAR = "clear"
PREPRESS_CANCEL = "cancel"
PERCEPTION_INSIDE_B1 = "INSIDE_B1"

IGNORED_SELF_MESSAGES = {
    RESULT_INSIDE_BTN_DONE,
    RESULT_UNLOAD_DONE,
    RESULT_HOME_DONE,
    RESULT_CANCELLED,
}


class ManipulatorTaskManagerV3Student(Node):
    def __init__(self) -> None:
        super().__init__("manipulator_task_manager_v3_student")

        self.declare_parameter("task_cmd_topic", "/manipulator_task_cmd")
        self.declare_parameter("done_topic", "/manipulator_task_result")
        self.declare_parameter("state_topic", "/manipulator_task_state")
        self.declare_parameter("error_topic", "/manipulator_task_error")

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
        self.declare_parameter("perception_inside_b1", PERCEPTION_INSIDE_B1)
        self.declare_parameter("inside_prepress_profile", "inside_b1_front")

        self.declare_parameter("mcu_result_topic", "/mcu/result")
        self.declare_parameter("mcu_unload_done", RESULT_UNLOAD_DONE)
        self.declare_parameter(
            "cmd_pos_flag_topic",
            "/manipulator_hardware/cmd_pos_flag",
        )
        self.declare_parameter("unload_prepare_cmd_pos_flag", 3)
        self.declare_parameter("unload_cmd_pos_flag", 2)
        self.declare_parameter("unload_wait_for_result", True)

        self.declare_parameter("marker_settle_sec", 3.0)
        self.declare_parameter("inside_align_timeout_sec", 12.0)
        self.declare_parameter("prepress_timeout_sec", 25.0)
        self.declare_parameter("home_timeout_sec", 12.0)
        self.declare_parameter("unload_timeout_sec", 10.0)
        self.declare_parameter("fsm_tick_sec", 0.05)
        self.declare_parameter("return_home_after_inside_button", True)
        self.declare_parameter("complete_inside_button_on_failure", True)

        self.task_cmd_topic = str(self.get_parameter("task_cmd_topic").value)
        self.done_topic = str(self.get_parameter("done_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.error_topic = str(self.get_parameter("error_topic").value)
        self.arm_flag_topic = str(self.get_parameter("arm_flag_topic").value)
        self.arm_done_topic = str(self.get_parameter("arm_done_topic").value)
        self.prepress_cmd_topic = str(self.get_parameter("prepress_cmd_topic").value)
        self.prepress_result_topic = str(
            self.get_parameter("prepress_result_topic").value
        )
        self.perception_target_topic = str(
            self.get_parameter("perception_target_topic").value
        )
        self.perception_inside_b1 = str(
            self.get_parameter("perception_inside_b1").value
        )
        self.inside_prepress_profile = str(
            self.get_parameter("inside_prepress_profile").value
        )
        self.mcu_result_topic = str(self.get_parameter("mcu_result_topic").value)
        self.mcu_unload_done = str(self.get_parameter("mcu_unload_done").value)
        self.cmd_pos_flag_topic = str(self.get_parameter("cmd_pos_flag_topic").value)
        self.unload_prepare_cmd_pos_flag = int(
            self.get_parameter("unload_prepare_cmd_pos_flag").value
        )
        self.unload_cmd_pos_flag = int(self.get_parameter("unload_cmd_pos_flag").value)
        self.marker_settle_sec = float(self.get_parameter("marker_settle_sec").value)
        self.inside_align_timeout_sec = float(
            self.get_parameter("inside_align_timeout_sec").value
        )
        self.prepress_timeout_sec = float(
            self.get_parameter("prepress_timeout_sec").value
        )
        self.home_timeout_sec = float(self.get_parameter("home_timeout_sec").value)
        self.unload_timeout_sec = float(self.get_parameter("unload_timeout_sec").value)
        self.fsm_tick_sec = float(self.get_parameter("fsm_tick_sec").value)
        self.return_home_after_inside_button = bool(
            self.get_parameter("return_home_after_inside_button").value
        )
        self.complete_inside_button_on_failure = bool(
            self.get_parameter("complete_inside_button_on_failure").value
        )

        self._state = "IDLE"
        self._active_task: Optional[str] = None
        self._deadline = None
        self._delay_until = None
        self._pending_done_after_home: Optional[str] = None

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

        self.done_pub = self.create_publisher(String, self.done_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.error_pub = self.create_publisher(String, self.error_topic, 10)
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
        self.cmd_pos_flag_pub = self.create_publisher(
            Int32,
            self.cmd_pos_flag_topic,
            10,
        )

        self.fsm_timer = self.create_timer(self.fsm_tick_sec, self._fsm_tick)

        self.get_logger().info("[task_manager_v3_student] ready")
        self.get_logger().info(
            f"[task_manager_v3_student] task_cmd_topic={self.task_cmd_topic}"
        )
        self.get_logger().info(
            f"[task_manager_v3_student] done_topic={self.done_topic}"
        )
        self.get_logger().info(
            "[task_manager_v3_student] supported commands: "
            "INSIDE_BTN_FRONT, DESTINATION_UNLOAD, HOME, CANCEL, STATUS, RESET"
        )
        self._publish_state()

    def _task_cmd_cb(self, msg: String) -> None:
        cmd = msg.data.strip().upper()
        if not cmd:
            return

        if cmd in IGNORED_SELF_MESSAGES:
            self.get_logger().debug(
                f"[task_manager_v3_student] ignored done message='{cmd}'"
            )
            return
        if cmd == "RESET_DONE" or cmd.startswith("STATUS:") or cmd.startswith("FAILED:"):
            self.get_logger().debug(
                f"[task_manager_v3_student] ignored self message='{cmd}'"
            )
            return
        if cmd == CMD_STATUS:
            self._publish_status()
            return
        if cmd == CMD_CANCEL:
            self._cancel_current_task()
            return
        if cmd == CMD_RESET:
            self._reset_to_idle()
            self._publish_done("RESET_DONE")
            return
        if cmd == CMD_HOME:
            self._start_home_only()
            return
        if cmd == CMD_INSIDE_BTN_FRONT:
            self._start_inside_button_task()
            return
        if cmd == CMD_DESTINATION_UNLOAD:
            self._start_destination_unload_task()
            return

        self._publish_error(f"UNKNOWN_COMMAND:{cmd}")

    def _start_inside_button_task(self) -> None:
        if not self._accept_new_task(CMD_INSIDE_BTN_FRONT):
            return

        self._active_task = CMD_INSIDE_BTN_FRONT
        self._publish_perception_target(self.perception_inside_b1)
        self._publish_prepress_cmd(PREPRESS_CLEAR)
        self._set_state("INSIDE_ALIGNING")
        self._set_deadline(self.inside_align_timeout_sec)
        self._publish_arm_cmd(ARM_INSIDE_SCAN)

    def _arm_done_cb(self, msg: String) -> None:
        text = msg.data.strip().lower()
        if not text:
            return
        self.get_logger().info(f"[task_manager_v3_student] arm_done='{text}'")

        if self._state == "INSIDE_ALIGNING":
            if text == "inside_scan_done":
                self._set_state("MARKER_SETTLE")
                self._delay_until = self.get_clock().now() + Duration(
                    seconds=self.marker_settle_sec
                )
                self._set_deadline(max(2.0, self.marker_settle_sec + 1.0))
            elif self._is_failure(text):
                self._complete_inside_button_demo(f"INSIDE_ALIGN_FAILED:{text}")
            return

        if self._state in ("BUTTON_HOMING", "HOME_ONLY"):
            if text == "home_done":
                done = self._pending_done_after_home
                if self._state == "HOME_ONLY":
                    done = RESULT_HOME_DONE
                self._reset_to_idle()
                if done is not None:
                    self._publish_done(done)
                return
            if self._is_failure(text):
                self._publish_error(f"HOME_FAILED:{text}")
                if self._pending_done_after_home == RESULT_INSIDE_BTN_DONE:
                    self._complete_inside_button_demo(
                        f"HOME_FAILED:{text}",
                        try_home=False,
                    )
                    return
                self._reset_to_idle()

    def _prepress_result_cb(self, msg: String) -> None:
        text = msg.data.strip().lower()
        if not text:
            return
        self.get_logger().info(f"[task_manager_v3_student] prepress_result='{text}'")

        if self._state != "BUTTON_PREPRESSING":
            return

        if text.startswith("prepress_done") or text.startswith("press_done"):
            self._finish_inside_button_attempt()
            return

        if self._is_failure(text) or text == "cancelled":
            self._publish_error(f"BUTTON_PREPRESS_FAILED:{text}")
            self._finish_inside_button_attempt()

    def _mcu_result_cb(self, msg: String) -> None:
        text = msg.data.strip().upper()
        if not text:
            return
        self.get_logger().info(f"[task_manager_v3_student] mcu_result='{text}'")

        if self._state != "UNLOADING":
            return

        if text == self.mcu_unload_done.upper():
            self._reset_to_idle()
            self._publish_done(RESULT_UNLOAD_DONE)
            return
        if "FAIL" in text:
            self._complete_unload_demo(f"UNLOAD_FAILED:{text}")

    def _finish_inside_button_attempt(self) -> None:
        if self.return_home_after_inside_button:
            self._start_home("BUTTON_HOMING", RESULT_INSIDE_BTN_DONE)
            return

        self._reset_to_idle()
        self._publish_done(RESULT_INSIDE_BTN_DONE)

    def _complete_inside_button_demo(
        self,
        reason: Optional[str] = None,
        *,
        try_home: bool = True,
    ) -> None:
        if reason is not None:
            self._publish_error(reason)

        if try_home and self.return_home_after_inside_button:
            self._start_home("BUTTON_HOMING", RESULT_INSIDE_BTN_DONE)
            return

        self._reset_to_idle()
        self._publish_done(RESULT_INSIDE_BTN_DONE)

    def _complete_unload_demo(self, reason: Optional[str] = None) -> None:
        if reason is not None:
            self._publish_error(reason)

        self._reset_to_idle()
        self._publish_done(RESULT_UNLOAD_DONE)

    def _start_destination_unload_task(self) -> None:
        if not self._accept_new_task(CMD_DESTINATION_UNLOAD):
            return

        self._active_task = CMD_DESTINATION_UNLOAD
        self._set_state("UNLOADING")
        self._set_deadline(self.unload_timeout_sec)
        self._publish_cmd_pos_flag(self.unload_prepare_cmd_pos_flag)
        self._publish_cmd_pos_flag(self.unload_cmd_pos_flag)

    def _start_home_only(self) -> None:
        if not self._accept_new_task(CMD_HOME):
            return

        self._active_task = CMD_HOME
        self._start_home("HOME_ONLY", RESULT_HOME_DONE)

    def _start_home(self, state: str, done_after_home: Optional[str]) -> None:
        self._pending_done_after_home = done_after_home
        self._set_state(state)
        self._set_deadline(self.home_timeout_sec)
        self._publish_arm_cmd(ARM_HOME)

    def _fsm_tick(self) -> None:
        now = self.get_clock().now()
        if self._delay_until is not None and now >= self._delay_until:
            self._delay_until = None
            if self._state == "MARKER_SETTLE":
                self._set_state("BUTTON_PREPRESSING")
                self._set_deadline(self.prepress_timeout_sec)
                self._publish_prepress_cmd(self.inside_prepress_profile)
                return

        if self._deadline is not None and now >= self._deadline:
            self._handle_timeout()

    def _handle_timeout(self) -> None:
        state = self._state
        if state == "INSIDE_ALIGNING":
            self._complete_inside_button_demo("INSIDE_ALIGN_TIMEOUT")
            return
        if state == "MARKER_SETTLE":
            self._complete_inside_button_demo("MARKER_SETTLE_TIMEOUT")
            return
        if state == "BUTTON_PREPRESSING":
            self._publish_error("BUTTON_PREPRESS_TIMEOUT")
            self._publish_prepress_cmd(PREPRESS_CANCEL)
            self._finish_inside_button_attempt()
            return
        if state == "BUTTON_HOMING":
            self._complete_inside_button_demo("BUTTON_HOMING_TIMEOUT", try_home=False)
            return
        if state == "HOME_ONLY":
            self._fail_task("HOME_ONLY_TIMEOUT")
            return
        if state == "UNLOADING":
            self._complete_unload_demo("UNLOAD_TIMEOUT")
            return
        self._fail_task(f"UNKNOWN_TIMEOUT_STATE:{state}")

    def _cancel_current_task(self) -> None:
        if self._state == "IDLE":
            self._publish_done(RESULT_CANCELLED)
            return

        self.get_logger().warn(
            f"[task_manager_v3_student] cancelling state={self._state}"
        )
        self._publish_arm_cmd(ARM_CANCEL)
        self._publish_prepress_cmd(PREPRESS_CANCEL)
        self._reset_to_idle()
        self._publish_done(RESULT_CANCELLED)

    def _fail_task(self, reason: str) -> None:
        self.get_logger().warn(f"[task_manager_v3_student] task failed: {reason}")
        self._publish_arm_cmd(ARM_CANCEL)
        self._publish_prepress_cmd(PREPRESS_CANCEL)
        self._set_state("ERROR")
        self._publish_error(reason)
        if self._active_task == CMD_INSIDE_BTN_FRONT:
            self._reset_to_idle()
            self._publish_done(RESULT_INSIDE_BTN_DONE)
            return
        if self._active_task == CMD_DESTINATION_UNLOAD:
            self._reset_to_idle()
            self._publish_done(RESULT_UNLOAD_DONE)
            return
        self._publish_done(f"FAILED:{reason}")
        self._reset_to_idle()

    def _accept_new_task(self, task: str) -> bool:
        if self._state == "IDLE":
            return True

        self.get_logger().warn(
            f"[task_manager_v3_student] busy state={self._state}; reject task={task}"
        )
        self._publish_error(f"BUSY:{self._state}")
        return False

    def _reset_to_idle(self) -> None:
        self._active_task = None
        self._deadline = None
        self._delay_until = None
        self._pending_done_after_home = None
        self._set_state("IDLE")

    def _set_state(self, state: str) -> None:
        if state != self._state:
            self.get_logger().info(
                f"[task_manager_v3_student] state: {self._state} -> {state}"
            )
        self._state = state
        self._publish_state()

    def _set_deadline(self, timeout_sec: float) -> None:
        self._deadline = self.get_clock().now() + Duration(seconds=timeout_sec)

    def _publish_perception_target(self, text: str) -> None:
        self._publish_string(self.perception_target_pub, text)
        self.get_logger().info(f"[task_manager_v3_student] perception_target='{text}'")

    def _publish_arm_cmd(self, text: str) -> None:
        self._publish_string(self.arm_flag_pub, text)
        self.get_logger().info(f"[task_manager_v3_student] arm_cmd='{text}'")

    def _publish_prepress_cmd(self, text: str) -> None:
        self._publish_string(self.prepress_cmd_pub, text)
        self.get_logger().info(f"[task_manager_v3_student] prepress_cmd='{text}'")

    def _publish_cmd_pos_flag(self, flag: int) -> None:
        msg = Int32()
        msg.data = flag
        self.cmd_pos_flag_pub.publish(msg)
        self.get_logger().info(f"[task_manager_v3_student] cmd_pos_flag={flag}")

    def _publish_done(self, text: str) -> None:
        self._publish_string(self.done_pub, text)
        self.get_logger().info(f"[task_manager_v3_student] done='{text}'")

    def _publish_error(self, text: str) -> None:
        self._publish_string(self.error_pub, text)
        self.get_logger().warn(f"[task_manager_v3_student] error='{text}'")

    def _publish_state(self) -> None:
        active = self._active_task or "NONE"
        self._publish_string(self.state_pub, f"{self._state}:{active}")

    def _publish_status(self) -> None:
        self._publish_done(f"STATUS:{self._state}:{self._active_task or 'NONE'}")

    def _publish_string(self, publisher, text: str) -> None:
        msg = String()
        msg.data = text
        publisher.publish(msg)

    def _is_failure(self, text: str) -> bool:
        lowered = text.lower()
        return (
            lowered.endswith("_failed")
            or lowered == "failed"
            or lowered.startswith("failed")
            or "error" in lowered
            or "timeout" in lowered
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManipulatorTaskManagerV3Student()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
