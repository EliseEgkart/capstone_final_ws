#!/usr/bin/env python3
"""
Student demo version of the top-level manipulator task FSM.

Supported demo commands:
  - INSIDE_BTN_FRONT
      inside scan -> marker settle -> prepress profile -> home -> INSIDE_BTN_DONE
  - DESTINATION_UNLOAD
      publish unload prepare flag(3) -> publish unload flag(2) -> UNLOAD_DONE

The node intentionally allows the same command to be executed repeatedly.
It only rejects a new command while another task is currently running.
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
RESULT_RESET_DONE = "RESET_DONE"

STATE_IDLE = "IDLE"
STATE_INSIDE_ALIGNING = "INSIDE_ALIGNING"
STATE_INSIDE_MARKER_SETTLE = "INSIDE_MARKER_SETTLE"
STATE_BUTTON_PREPRESSING = "BUTTON_PREPRESSING"
STATE_BUTTON_HOMING = "BUTTON_HOMING"
STATE_UNLOAD_PREPARE = "UNLOAD_PREPARE"
STATE_UNLOAD_EXECUTE = "UNLOAD_EXECUTE"
STATE_HOME_ONLY = "HOME_ONLY"
STATE_ERROR = "ERROR"


class ManipulatorTaskManagerV3Student(Node):
    def __init__(self) -> None:
        super().__init__("manipulator_task_manager_v3_student")

        # =========================================================
        # Task-level topics
        # =========================================================
        self.declare_parameter("task_cmd_topic", "/manipulator_task_cmd")
        self.declare_parameter("task_result_topic", "/manipulator_task_result")
        self.declare_parameter("task_state_topic", "/manipulator_task_state")
        self.declare_parameter("task_error_topic", "/manipulator_task_error")

        # =========================================================
        # Arm pose commander interface
        # =========================================================
        self.declare_parameter("arm_flag_topic", "/arm_pose_commander_v2/flag")
        self.declare_parameter("arm_done_topic", "/arm_pose_commander_v2/done")
        self.declare_parameter("inside_scan_arm_flag", "inside_scan")
        self.declare_parameter("home_arm_flag", "home")
        self.declare_parameter("cancel_arm_flag", "cancel")
        self.declare_parameter("inside_scan_done_text", "inside_scan_done")
        self.declare_parameter("home_done_text", "home_done")

        # =========================================================
        # Marker prepress commander interface
        # =========================================================
        self.declare_parameter(
            "prepress_cmd_topic",
            "/marker_prepress_commander_v2/cmd",
        )
        self.declare_parameter(
            "prepress_result_topic",
            "/marker_prepress_commander_v2/result",
        )
        self.declare_parameter("inside_prepress_profile", "inside_b1_front")
        self.declare_parameter("prepress_clear_cmd", "clear")
        self.declare_parameter("prepress_cancel_cmd", "cancel")

        # =========================================================
        # Perception target interface
        # =========================================================
        self.declare_parameter(
            "perception_target_topic",
            "/manipulator_perception/target_button",
        )
        self.declare_parameter("perception_inside_b1", "INSIDE_B1")

        # =========================================================
        # ESP32 / hardware unload interface
        # =========================================================
        self.declare_parameter(
            "cmd_pos_flag_topic",
            "/manipulator_hardware/cmd_pos_flag",
        )
        self.declare_parameter("unload_prepare_cmd_pos_flag", 3)
        self.declare_parameter("unload_cmd_pos_flag", 2)

        # Optional legacy/string command path.
        # Keep disabled unless the ESP32 command node directly expects String commands.
        self.declare_parameter("publish_esp32_cmd_string", False)
        self.declare_parameter("esp32_cmd_topic", "/mcu/cmd")
        self.declare_parameter("esp32_unload_prepare_cmd", "CMD_POS,0,0,3")
        self.declare_parameter("esp32_unload_cmd", "CMD_POS,0,0,2")
        self.declare_parameter("esp32_cancel_cmd", "CANCEL")

        # =========================================================
        # Behavior / timeout parameters
        # =========================================================
        self.declare_parameter("return_home_after_prepress", True)
        self.declare_parameter("complete_button_on_prepress_failure", True)
        self.declare_parameter("marker_settle_sec", 3.0)
        self.declare_parameter("inside_align_timeout_sec", 12.0)
        self.declare_parameter("prepress_timeout_sec", 25.0)
        self.declare_parameter("home_timeout_sec", 12.0)
        self.declare_parameter("unload_timeout_sec", 25.0)
        self.declare_parameter("unload_step_delay_sec", 0.5)
        self.declare_parameter("unload_done_delay_sec", 0.2)
        self.declare_parameter("fsm_tick_sec", 0.05)
        self.declare_parameter("auto_reset_after_failure", True)

        # =========================================================
        # Parameter values
        # =========================================================
        self.task_cmd_topic = str(self.get_parameter("task_cmd_topic").value)
        self.task_result_topic = str(self.get_parameter("task_result_topic").value)
        self.task_state_topic = str(self.get_parameter("task_state_topic").value)
        self.task_error_topic = str(self.get_parameter("task_error_topic").value)

        self.arm_flag_topic = str(self.get_parameter("arm_flag_topic").value)
        self.arm_done_topic = str(self.get_parameter("arm_done_topic").value)
        self.inside_scan_arm_flag = str(self.get_parameter("inside_scan_arm_flag").value)
        self.home_arm_flag = str(self.get_parameter("home_arm_flag").value)
        self.cancel_arm_flag = str(self.get_parameter("cancel_arm_flag").value)
        self.inside_scan_done_text = str(
            self.get_parameter("inside_scan_done_text").value
        ).lower()
        self.home_done_text = str(self.get_parameter("home_done_text").value).lower()

        self.prepress_cmd_topic = str(self.get_parameter("prepress_cmd_topic").value)
        self.prepress_result_topic = str(
            self.get_parameter("prepress_result_topic").value
        )
        self.inside_prepress_profile = str(
            self.get_parameter("inside_prepress_profile").value
        )
        self.prepress_clear_cmd = str(self.get_parameter("prepress_clear_cmd").value)
        self.prepress_cancel_cmd = str(self.get_parameter("prepress_cancel_cmd").value)

        self.perception_target_topic = str(
            self.get_parameter("perception_target_topic").value
        )
        self.perception_inside_b1 = str(
            self.get_parameter("perception_inside_b1").value
        )

        self.cmd_pos_flag_topic = str(self.get_parameter("cmd_pos_flag_topic").value)
        self.unload_prepare_cmd_pos_flag = int(
            self.get_parameter("unload_prepare_cmd_pos_flag").value
        )
        self.unload_cmd_pos_flag = int(self.get_parameter("unload_cmd_pos_flag").value)

        self.publish_esp32_cmd_string = bool(
            self.get_parameter("publish_esp32_cmd_string").value
        )
        self.esp32_cmd_topic = str(self.get_parameter("esp32_cmd_topic").value)
        self.esp32_unload_prepare_cmd = str(
            self.get_parameter("esp32_unload_prepare_cmd").value
        )
        self.esp32_unload_cmd = str(self.get_parameter("esp32_unload_cmd").value)
        self.esp32_cancel_cmd = str(self.get_parameter("esp32_cancel_cmd").value)

        self.return_home_after_prepress = bool(
            self.get_parameter("return_home_after_prepress").value
        )
        self.complete_button_on_prepress_failure = bool(
            self.get_parameter("complete_button_on_prepress_failure").value
        )
        self.marker_settle_sec = float(self.get_parameter("marker_settle_sec").value)
        self.inside_align_timeout_sec = float(
            self.get_parameter("inside_align_timeout_sec").value
        )
        self.prepress_timeout_sec = float(
            self.get_parameter("prepress_timeout_sec").value
        )
        self.home_timeout_sec = float(self.get_parameter("home_timeout_sec").value)
        self.unload_timeout_sec = float(self.get_parameter("unload_timeout_sec").value)
        self.unload_step_delay_sec = float(
            self.get_parameter("unload_step_delay_sec").value
        )
        self.unload_done_delay_sec = float(
            self.get_parameter("unload_done_delay_sec").value
        )
        self.fsm_tick_sec = float(self.get_parameter("fsm_tick_sec").value)
        self.auto_reset_after_failure = bool(
            self.get_parameter("auto_reset_after_failure").value
        )

        # =========================================================
        # FSM variables
        # =========================================================
        self._state = STATE_IDLE
        self._active_task: Optional[str] = None
        self._deadline = None
        self._delay_until = None
        self._pending_result_after_home: Optional[str] = None

        # =========================================================
        # ROS interfaces
        # =========================================================
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
        self.cmd_pos_flag_pub = self.create_publisher(
            Int32,
            self.cmd_pos_flag_topic,
            10,
        )
        self.esp32_cmd_pub = self.create_publisher(String, self.esp32_cmd_topic, 10)

        self.fsm_timer = self.create_timer(self.fsm_tick_sec, self._fsm_tick)

        self.get_logger().info("[task_manager_v3_student] ready")
        self.get_logger().info(
            "[task_manager_v3_student] supported tasks: "
            f"{CMD_INSIDE_BTN_FRONT}, {CMD_DESTINATION_UNLOAD}"
        )
        self.get_logger().info(
            "[task_manager_v3_student] repeated same command is allowed after IDLE"
        )
        self._publish_state()

    # =============================================================
    # Command callback
    # =============================================================
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
            self._reset_to_idle()
            self._publish_result(RESULT_RESET_DONE)
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

    # =============================================================
    # Inside button task
    # =============================================================
    def _start_inside_button_task(self) -> None:
        if not self._accept_new_task(CMD_INSIDE_BTN_FRONT):
            return

        self._active_task = CMD_INSIDE_BTN_FRONT

        self._publish_perception_target(self.perception_inside_b1)
        self._publish_prepress_cmd(self.prepress_clear_cmd)

        self._set_state(STATE_INSIDE_ALIGNING)
        self._set_deadline(self.inside_align_timeout_sec)
        self._publish_arm_cmd(self.inside_scan_arm_flag)

    def _arm_done_cb(self, msg: String) -> None:
        text = msg.data.strip().lower()
        if not text:
            return

        self.get_logger().info(f"[task_manager_v3_student] arm_done='{text}'")

        if self._state == STATE_INSIDE_ALIGNING:
            if text == self.inside_scan_done_text:
                self._start_marker_settle()
                return
            if self._is_failure(text):
                self._fail_task(f"INSIDE_ALIGN_FAILED:{text}")
                return

        if self._state in (STATE_BUTTON_HOMING, STATE_HOME_ONLY):
            if text == self.home_done_text:
                if self._state == STATE_HOME_ONLY:
                    self._reset_to_idle()
                    self._publish_result(RESULT_HOME_DONE)
                    return

                result = self._pending_result_after_home or RESULT_INSIDE_BTN_DONE
                self._reset_to_idle()
                self._publish_result(result)
                return

            if self._is_failure(text):
                self._fail_task(f"HOME_FAILED:{text}")
                return

        self.get_logger().debug(
            f"[task_manager_v3_student] ignored arm_done='{text}' "
            f"in state={self._state}"
        )

    def _start_marker_settle(self) -> None:
        self._set_state(STATE_INSIDE_MARKER_SETTLE)
        self._delay_until = self.get_clock().now() + Duration(
            seconds=self.marker_settle_sec
        )
        self._set_deadline(max(2.0, self.marker_settle_sec + 1.0))

    def _start_prepress(self) -> None:
        self._set_state(STATE_BUTTON_PREPRESSING)
        self._set_deadline(self.prepress_timeout_sec)
        self.get_logger().info(
            "[task_manager_v3_student] selected prepress profile='"
            f"{self.inside_prepress_profile}'"
        )
        self._publish_prepress_cmd(self.inside_prepress_profile)

    def _prepress_result_cb(self, msg: String) -> None:
        text = msg.data.strip().lower()
        if not text:
            return

        self.get_logger().info(f"[task_manager_v3_student] prepress_result='{text}'")

        if self._state != STATE_BUTTON_PREPRESSING:
            self.get_logger().debug(
                f"[task_manager_v3_student] ignored prepress_result='{text}' "
                f"in state={self._state}"
            )
            return

        if text.startswith("prepress_done") or text.startswith("press_done"):
            self._finish_button_attempt(RESULT_INSIDE_BTN_DONE)
            return

        if self._is_failure(text) or text == "cancelled":
            reason = f"BUTTON_PREPRESS_FAILED:{text}"
            self._publish_error(reason)
            if self.complete_button_on_prepress_failure:
                self._finish_button_attempt(RESULT_INSIDE_BTN_DONE)
                return
            self._fail_task(reason)

    def _finish_button_attempt(self, done_result: str) -> None:
        if self.return_home_after_prepress:
            self._start_home(STATE_BUTTON_HOMING, done_result)
            return

        self._reset_to_idle()
        self._publish_result(done_result)

    # =============================================================
    # Destination unload task
    # =============================================================
    def _start_destination_unload_task(self) -> None:
        if not self._accept_new_task(CMD_DESTINATION_UNLOAD):
            return

        self._active_task = CMD_DESTINATION_UNLOAD
        self._set_state(STATE_UNLOAD_PREPARE)
        self._set_deadline(self.unload_timeout_sec)

        self._publish_unload_prepare_command()
        self._delay_until = self.get_clock().now() + Duration(
            seconds=self.unload_step_delay_sec
        )

    def _publish_unload_prepare_command(self) -> None:
        self._publish_cmd_pos_flag(self.unload_prepare_cmd_pos_flag)
        if self.publish_esp32_cmd_string:
            self._publish_esp32_cmd(self.esp32_unload_prepare_cmd)

    def _publish_unload_execute_command(self) -> None:
        self._publish_cmd_pos_flag(self.unload_cmd_pos_flag)
        if self.publish_esp32_cmd_string:
            self._publish_esp32_cmd(self.esp32_unload_cmd)

    def _finish_unload_success(self) -> None:
        self._reset_to_idle()
        self._publish_result(RESULT_UNLOAD_DONE)

    # =============================================================
    # Home / cancel / failure
    # =============================================================
    def _start_home_only(self) -> None:
        if not self._accept_new_task(CMD_HOME):
            return

        self._active_task = CMD_HOME
        self._start_home(STATE_HOME_ONLY, RESULT_HOME_DONE)

    def _start_home(self, state: str, result_after_home: Optional[str] = None) -> None:
        self._pending_result_after_home = result_after_home
        self._set_state(state)
        self._set_deadline(self.home_timeout_sec)
        self._publish_arm_cmd(self.home_arm_flag)

    def _cancel_current_task(self) -> None:
        if self._state == STATE_IDLE:
            self._publish_result(RESULT_CANCELLED)
            return

        self.get_logger().warn(
            f"[task_manager_v3_student] cancelling task in state={self._state}"
        )
        self._publish_arm_cmd(self.cancel_arm_flag)
        self._publish_prepress_cmd(self.prepress_cancel_cmd)
        if self.publish_esp32_cmd_string:
            self._publish_esp32_cmd(self.esp32_cancel_cmd)
        self._reset_to_idle()
        self._publish_result(RESULT_CANCELLED)

    def _fail_task(self, reason: str) -> None:
        self.get_logger().warn(f"[task_manager_v3_student] task failed: {reason}")
        self._publish_arm_cmd(self.cancel_arm_flag)
        self._publish_prepress_cmd(self.prepress_cancel_cmd)
        if self.publish_esp32_cmd_string:
            self._publish_esp32_cmd(self.esp32_cancel_cmd)
        self._set_state(STATE_ERROR)
        self._publish_error(reason)
        self._publish_result(f"FAILED:{reason}")
        if self.auto_reset_after_failure:
            self._reset_to_idle()

    def _reset_to_idle(self) -> None:
        self._active_task = None
        self._deadline = None
        self._delay_until = None
        self._pending_result_after_home = None
        self._set_state(STATE_IDLE)

    def _accept_new_task(self, task_name: str) -> bool:
        if self._state != STATE_IDLE:
            self._publish_result(
                f"BUSY:CURRENT_STATE={self._state},CURRENT_TASK={self._active_task}"
            )
            self.get_logger().warn(
                f"[task_manager_v3_student] rejected task={task_name}, "
                f"state={self._state}, active_task={self._active_task}"
            )
            return False
        return True

    # =============================================================
    # FSM timer
    # =============================================================
    def _fsm_tick(self) -> None:
        now = self.get_clock().now()

        if self._delay_until is not None and now >= self._delay_until:
            self._delay_until = None

            if self._state == STATE_INSIDE_MARKER_SETTLE:
                self._start_prepress()
                return

            if self._state == STATE_UNLOAD_PREPARE:
                self._publish_unload_execute_command()
                self._set_state(STATE_UNLOAD_EXECUTE)
                self._delay_until = self.get_clock().now() + Duration(
                    seconds=self.unload_done_delay_sec
                )
                return

            if self._state == STATE_UNLOAD_EXECUTE:
                self._finish_unload_success()
                return

        if self._deadline is not None and now >= self._deadline:
            self._handle_timeout()

    def _handle_timeout(self) -> None:
        state = self._state

        if state == STATE_INSIDE_ALIGNING:
            self._fail_task("INSIDE_ALIGN_TIMEOUT")
            return
        if state == STATE_INSIDE_MARKER_SETTLE:
            self._fail_task("INSIDE_MARKER_SETTLE_TIMEOUT")
            return
        if state == STATE_BUTTON_PREPRESSING:
            self._publish_error("BUTTON_PREPRESS_TIMEOUT")
            if self.complete_button_on_prepress_failure:
                self._publish_prepress_cmd(self.prepress_cancel_cmd)
                self._finish_button_attempt(RESULT_INSIDE_BTN_DONE)
                return
            self._fail_task("BUTTON_PREPRESS_TIMEOUT")
            return
        if state == STATE_BUTTON_HOMING:
            self._fail_task("BUTTON_HOMING_TIMEOUT")
            return
        if state == STATE_HOME_ONLY:
            self._fail_task("HOME_ONLY_TIMEOUT")
            return
        if state in (STATE_UNLOAD_PREPARE, STATE_UNLOAD_EXECUTE):
            self._fail_task("UNLOAD_TIMEOUT")
            return

        self._fail_task(f"UNKNOWN_TIMEOUT_STATE:{state}")

    # =============================================================
    # Publishers / status
    # =============================================================
    def _publish_perception_target(self, text: str) -> None:
        self._publish_string(self.perception_target_pub, text)
        self.get_logger().info(f"[task_manager_v3_student] perception_target='{text}'")

    def _publish_arm_cmd(self, text: str) -> None:
        self._publish_string(self.arm_flag_pub, text)
        self.get_logger().info(f"[task_manager_v3_student] arm_cmd='{text}'")

    def _publish_prepress_cmd(self, text: str) -> None:
        self._publish_string(self.prepress_cmd_pub, text)
        self.get_logger().info(f"[task_manager_v3_student] prepress_cmd='{text}'")

    def _publish_esp32_cmd(self, text: str) -> None:
        self._publish_string(self.esp32_cmd_pub, text)
        self.get_logger().info(f"[task_manager_v3_student] esp32_cmd='{text}'")

    def _publish_cmd_pos_flag(self, flag: int) -> None:
        msg = Int32()
        msg.data = int(flag)
        self.cmd_pos_flag_pub.publish(msg)
        self.get_logger().info(f"[task_manager_v3_student] cmd_pos_flag={flag}")

    def _publish_result(self, text: str) -> None:
        self._publish_string(self.task_result_pub, text)
        self.get_logger().info(f"[task_manager_v3_student] result='{text}'")

    def _publish_error(self, text: str) -> None:
        self._publish_string(self.task_error_pub, text)
        self.get_logger().warn(f"[task_manager_v3_student] error='{text}'")

    def _publish_state(self) -> None:
        self._publish_string(self.task_state_pub, self._state)

    def _publish_status(self) -> None:
        status = (
            f"STATUS_V3_STUDENT:STATE={self._state},"
            f"ACTIVE_TASK={self._active_task},"
            f"PENDING_RESULT_AFTER_HOME={self._pending_result_after_home},"
            f"INSIDE_PROFILE={self.inside_prepress_profile},"
            f"RETURN_HOME_AFTER_PREPRESS={self.return_home_after_prepress},"
            f"COMPLETE_BUTTON_ON_PREPRESS_FAILURE="
            f"{self.complete_button_on_prepress_failure},"
            f"UNLOAD_PREPARE_FLAG={self.unload_prepare_cmd_pos_flag},"
            f"UNLOAD_FLAG={self.unload_cmd_pos_flag},"
            f"PUBLISH_ESP32_CMD_STRING={self.publish_esp32_cmd_string}"
        )
        self._publish_result(status)

    def _set_state(self, state: str) -> None:
        if self._state != state:
            self.get_logger().info(
                f"[task_manager_v3_student] state: {self._state} -> {state}"
            )
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
    node = ManipulatorTaskManagerV3Student()
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