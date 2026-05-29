#!/usr/bin/env python3
"""
Manipulator task manager.

This node is the top-level FSM for the manipulator side only.

AMR -> Manipulator:
  /manipulator_task_cmd      std_msgs/msg/String

Manipulator -> AMR:
  /manipulator_task_result   std_msgs/msg/String
  /manipulator_task_state    std_msgs/msg/String

Internal:
  task_manager -> arm_pose_commander:
    /arm_pose_commander/flag std_msgs/msg/String

  arm_pose_commander -> task_manager:
    /arm_pose_commander/done std_msgs/msg/String

  task_manager -> marker_button_press_commander:
    /marker_button_press_commander/cmd std_msgs/msg/String
      press_outside, press_inside, clear, cancel, status

  marker_button_press_commander -> task_manager:
    /marker_button_press_commander/result std_msgs/msg/String

  task_manager -> object_distance_node:
    /manipulator_perception/target_button std_msgs/msg/String
      OUTSIDE_DOWN, INSIDE_B1, INSIDE_3F

  task_manager -> MCU bridge:
    /mcu/cmd std_msgs/msg/String

  MCU bridge -> task_manager:
    /mcu/result std_msgs/msg/String

Button task behavior:
  button_press_done or button_press_failed
  -> publish OUTSIDE_BTN_DONE / INSIDE_BTN_DONE immediately
  -> return manipulator to home internally
  -> on home_done, only return FSM to IDLE
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String


# =========================================================
# External command strings: AMR -> manipulator
# =========================================================
CMD_OUTSIDE_BTN_FRONT = "OUTSIDE_BTN_FRONT"
CMD_INSIDE_BTN_FRONT = "INSIDE_BTN_FRONT"  # legacy alias: B1 button
CMD_INSIDE_B1_BTN_FRONT = "INSIDE_B1_BTN_FRONT"
CMD_INSIDE_3F_BTN_FRONT = "INSIDE_3F_BTN_FRONT"
CMD_DESTINATION_UNLOAD = "DESTINATION_UNLOAD"
CMD_HOME = "HOME"
CMD_CANCEL = "CANCEL"
CMD_STATUS = "STATUS"
CMD_RESET = "RESET"


# =========================================================
# External result strings: manipulator -> AMR
# =========================================================
RESULT_OUTSIDE_BTN_DONE = "OUTSIDE_BTN_DONE"
RESULT_INSIDE_BTN_DONE = "INSIDE_BTN_DONE"  # legacy alias result
RESULT_INSIDE_B1_BTN_DONE = "INSIDE_B1_BTN_DONE"
RESULT_INSIDE_3F_BTN_DONE = "INSIDE_3F_BTN_DONE"
RESULT_UNLOAD_DONE = "UNLOAD_DONE"
RESULT_HOME_DONE = "HOME_DONE"
RESULT_CANCELLED = "CANCELLED"


# =========================================================
# Internal command/result strings
# =========================================================
ARM_OUTSIDE_SCAN = "outside_scan"
ARM_INSIDE_SCAN = "inside_scan"
ARM_HOME = "home"
ARM_CANCEL = "cancel"

BUTTON_PRESS = "press"
BUTTON_PRESS_OUTSIDE = "press_outside"
BUTTON_PRESS_INSIDE = "press_inside"
BUTTON_CLEAR = "clear"
BUTTON_CANCEL = "cancel"

PERCEPTION_OUTSIDE_DOWN = "OUTSIDE_DOWN"
PERCEPTION_INSIDE_B1 = "INSIDE_B1"
PERCEPTION_INSIDE_3F = "INSIDE_3F"


class ManipulatorTaskManager(Node):
    """
    Top-level manipulator-side task FSM.

    Main sequences:
      1. OUTSIDE_BTN_FRONT:
         clear marker -> outside_scan -> press_outside -> home -> OUTSIDE_BTN_DONE

      2. INSIDE_BTN_FRONT:
         clear marker -> inside_scan -> press_inside -> home -> INSIDE_BTN_DONE

      3. DESTINATION_UNLOAD:
         unload command to MCU -> wait or delay -> optional home -> UNLOAD_DONE
    """

    def __init__(self) -> None:
        super().__init__("manipulator_task_manager")

        # =====================================================
        # External topics
        # =====================================================
        self.declare_parameter("task_cmd_topic", "/manipulator_task_cmd")
        self.declare_parameter("task_result_topic", "/manipulator_task_result")
        self.declare_parameter("task_state_topic", "/manipulator_task_state")
        self.declare_parameter("task_error_topic", "/manipulator_task_error")

        # =====================================================
        # Internal commander topics
        # =====================================================
        self.declare_parameter("arm_flag_topic", "/arm_pose_commander/flag")
        self.declare_parameter("arm_done_topic", "/arm_pose_commander/done")

        self.declare_parameter(
            "button_cmd_topic",
            "/marker_button_press_commander/cmd",
        )
        self.declare_parameter(
            "button_result_topic",
            "/marker_button_press_commander/result",
        )

        # =====================================================
        # Perception target selection
        # =====================================================
        self.declare_parameter(
            "perception_target_topic",
            "/manipulator_perception/target_button",
        )
        self.declare_parameter("perception_outside_down", PERCEPTION_OUTSIDE_DOWN)
        self.declare_parameter("perception_inside_b1", PERCEPTION_INSIDE_B1)
        self.declare_parameter("perception_inside_3f", PERCEPTION_INSIDE_3F)

        # =====================================================
        # MCU unload interface
        # =====================================================
        self.declare_parameter("mcu_cmd_topic", "/mcu/cmd")
        self.declare_parameter("mcu_result_topic", "/mcu/result")

        self.declare_parameter("mcu_unload_cmd", "UNLOAD")
        self.declare_parameter("mcu_unload_done", "UNLOAD_DONE")
        self.declare_parameter("mcu_unload_failed", "UNLOAD_FAILED")
        self.declare_parameter("mcu_cancel_cmd", "CANCEL")

        # If False, the manager publishes UNLOAD_DONE after unload_assume_done_delay_sec.
        self.declare_parameter("unload_wait_for_result", True)
        self.declare_parameter("unload_assume_done_delay_sec", 5.0)

        # =====================================================
        # Sequence options
        # =====================================================
        self.declare_parameter("return_home_after_button", True)
        self.declare_parameter("return_home_after_unload", True)

        # Time to wait after the scan pose is reached before pressing.
        # This gives perception time to refresh marker data at the new arm pose.
        self.declare_parameter("marker_settle_sec", 0.5)

        # =====================================================
        # Timeouts
        # =====================================================
        self.declare_parameter("outside_align_timeout_sec", 12.0)
        self.declare_parameter("inside_align_timeout_sec", 12.0)
        self.declare_parameter("button_press_timeout_sec", 20.0)
        self.declare_parameter("home_timeout_sec", 12.0)
        self.declare_parameter("unload_timeout_sec", 25.0)

        self.declare_parameter("fsm_tick_sec", 0.05)
        self.declare_parameter("auto_reset_after_failure", True)

        # =====================================================
        # Load parameters
        # =====================================================
        self.task_cmd_topic = str(self.get_parameter("task_cmd_topic").value)
        self.task_result_topic = str(self.get_parameter("task_result_topic").value)
        self.task_state_topic = str(self.get_parameter("task_state_topic").value)
        self.task_error_topic = str(self.get_parameter("task_error_topic").value)

        self.arm_flag_topic = str(self.get_parameter("arm_flag_topic").value)
        self.arm_done_topic = str(self.get_parameter("arm_done_topic").value)

        self.button_cmd_topic = str(self.get_parameter("button_cmd_topic").value)
        self.button_result_topic = str(
            self.get_parameter("button_result_topic").value
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

        self.unload_wait_for_result = bool(
            self.get_parameter("unload_wait_for_result").value
        )
        self.unload_assume_done_delay_sec = float(
            self.get_parameter("unload_assume_done_delay_sec").value
        )

        self.return_home_after_button = bool(
            self.get_parameter("return_home_after_button").value
        )
        self.return_home_after_unload = bool(
            self.get_parameter("return_home_after_unload").value
        )

        self.marker_settle_sec = float(self.get_parameter("marker_settle_sec").value)

        self.outside_align_timeout_sec = float(
            self.get_parameter("outside_align_timeout_sec").value
        )
        self.inside_align_timeout_sec = float(
            self.get_parameter("inside_align_timeout_sec").value
        )
        self.button_press_timeout_sec = float(
            self.get_parameter("button_press_timeout_sec").value
        )
        self.home_timeout_sec = float(self.get_parameter("home_timeout_sec").value)
        self.unload_timeout_sec = float(self.get_parameter("unload_timeout_sec").value)

        self.fsm_tick_sec = float(self.get_parameter("fsm_tick_sec").value)
        self.auto_reset_after_failure = bool(
            self.get_parameter("auto_reset_after_failure").value
        )

        # =====================================================
        # Runtime FSM state
        # =====================================================
        self._state = "IDLE"
        self._active_task: Optional[str] = None
        self._deadline = None
        self._delay_until = None
        self._pending_result_after_home: Optional[str] = None
        self._current_button_done_result: Optional[str] = None

        # =====================================================
        # ROS interfaces
        # =====================================================
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

        self.button_result_sub = self.create_subscription(
            String,
            self.button_result_topic,
            self._button_result_cb,
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

        self.task_state_pub = self.create_publisher(
            String,
            self.task_state_topic,
            10,
        )

        self.task_error_pub = self.create_publisher(
            String,
            self.task_error_topic,
            10,
        )

        self.arm_flag_pub = self.create_publisher(
            String,
            self.arm_flag_topic,
            10,
        )

        self.button_cmd_pub = self.create_publisher(
            String,
            self.button_cmd_topic,
            10,
        )

        self.perception_target_pub = self.create_publisher(
            String,
            self.perception_target_topic,
            10,
        )

        self.mcu_cmd_pub = self.create_publisher(
            String,
            self.mcu_cmd_topic,
            10,
        )

        self.fsm_timer = self.create_timer(self.fsm_tick_sec, self._fsm_tick)

        self.get_logger().info("[task_manager] ready")
        self.get_logger().info(f"[task_manager] task_cmd_topic={self.task_cmd_topic}")
        self.get_logger().info(
            f"[task_manager] task_result_topic={self.task_result_topic}"
        )
        self.get_logger().info(
            f"[task_manager] task_state_topic={self.task_state_topic}"
        )
        self.get_logger().info(f"[task_manager] arm_flag_topic={self.arm_flag_topic}")
        self.get_logger().info(f"[task_manager] arm_done_topic={self.arm_done_topic}")
        self.get_logger().info(
            f"[task_manager] button_cmd_topic={self.button_cmd_topic}"
        )
        self.get_logger().info(
            f"[task_manager] button_result_topic={self.button_result_topic}"
        )
        self.get_logger().info(
            f"[task_manager] perception_target_topic={self.perception_target_topic}"
        )
        self.get_logger().info(f"[task_manager] mcu_cmd_topic={self.mcu_cmd_topic}")
        self.get_logger().info(
            "[task_manager] button press command mapping: "
            f"outside={BUTTON_PRESS_OUTSIDE}, inside={BUTTON_PRESS_INSIDE}"
        )
        self.get_logger().info(
            "[task_manager] commands: "
            "OUTSIDE_BTN_FRONT, INSIDE_BTN_FRONT, INSIDE_B1_BTN_FRONT, "
            "INSIDE_3F_BTN_FRONT, DESTINATION_UNLOAD, HOME, CANCEL, STATUS, RESET"
        )

        self._publish_state()

    # =========================================================
    # External command callback
    # =========================================================
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
            self._publish_result("RESET_DONE")
            return

        if cmd == CMD_HOME:
            self._start_home_only()
            return

        if cmd == CMD_OUTSIDE_BTN_FRONT:
            self._start_outside_button_task()
            return

        if cmd == CMD_INSIDE_BTN_FRONT:
            # Legacy command: use B1 as the default inside target.
            self._start_inside_button_task(
                active_task=CMD_INSIDE_BTN_FRONT,
                perception_target=self.perception_inside_b1,
                done_result=RESULT_INSIDE_BTN_DONE,
            )
            return

        if cmd == CMD_INSIDE_B1_BTN_FRONT:
            self._start_inside_button_task(
                active_task=CMD_INSIDE_B1_BTN_FRONT,
                perception_target=self.perception_inside_b1,
                done_result=RESULT_INSIDE_B1_BTN_DONE,
            )
            return

        if cmd == CMD_INSIDE_3F_BTN_FRONT:
            self._start_inside_button_task(
                active_task=CMD_INSIDE_3F_BTN_FRONT,
                perception_target=self.perception_inside_3f,
                done_result=RESULT_INSIDE_3F_BTN_DONE,
            )
            return

        if cmd == CMD_DESTINATION_UNLOAD:
            self._start_destination_unload_task()
            return

        self._publish_error(f"UNKNOWN_COMMAND:{cmd}")

    # =========================================================
    # Internal result callbacks
    # =========================================================
    def _arm_done_cb(self, msg: String) -> None:
        text = msg.data.strip().lower()

        if not text:
            return

        self.get_logger().info(f"[task_manager] arm_done='{text}'")

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

        if self._state in ("OUTSIDE_HOMING", "INSIDE_HOMING", "UNLOAD_HOMING", "HOME_ONLY"):
            if text == "home_done":
                # HOME_ONLY는 home 명령 자체가 외부 요청이므로 HOME_DONE을 발행한다.
                if self._state == "HOME_ONLY":
                    self._reset_to_idle()
                    self._publish_result(RESULT_HOME_DONE)
                    return

                # UNLOAD_HOMING처럼 home 이후 결과 발행이 필요한 경우.
                if self._pending_result_after_home is not None:
                    result = self._pending_result_after_home
                    self._reset_to_idle()
                    self._publish_result(result)
                    return

                # OUTSIDE_HOMING / INSIDE_HOMING은 이미 AMR 완료 플래그를 보냈다.
                # 따라서 home_done 시점에는 내부 상태만 IDLE로 복귀한다.
                self._reset_to_idle()
                return

            elif self._is_failure(text):
                self._publish_error(f"HOME_FAILED:{text}")
                self._reset_to_idle()
                return

        # Ignore old or unrelated arm messages.
        self.get_logger().debug(
            f"[task_manager] ignored arm_done='{text}' in state={self._state}"
        )

    def _button_result_cb(self, msg: String) -> None:
        text = msg.data.strip().lower()

        if not text:
            return

        self.get_logger().info(f"[task_manager] button_result='{text}'")

        if self._state == "OUTSIDE_PRESSING":
            done_result = self._current_button_done_result or RESULT_OUTSIDE_BTN_DONE

            if text == "button_press_done":
                self._finish_button_attempt(
                    homing_state="OUTSIDE_HOMING",
                    done_result=done_result,
                )
                return

            if self._is_failure(text) or text == "cancelled":
                self._publish_error(f"OUTSIDE_PRESS_FAILED:{text}")
                self._finish_button_attempt(
                    homing_state="OUTSIDE_HOMING",
                    done_result=done_result,
                )
                return

            return

        if self._state == "INSIDE_PRESSING":
            done_result = self._current_button_done_result or RESULT_INSIDE_BTN_DONE

            if text == "button_press_done":
                self._finish_button_attempt(
                    homing_state="INSIDE_HOMING",
                    done_result=done_result,
                )
                return

            if self._is_failure(text) or text == "cancelled":
                self._publish_error(f"INSIDE_PRESS_FAILED:{text}")
                self._finish_button_attempt(
                    homing_state="INSIDE_HOMING",
                    done_result=done_result,
                )
                return

            return

        # Ignore old or unrelated button messages.
        self.get_logger().debug(
            f"[task_manager] ignored button_result='{text}' in state={self._state}"
        )

    def _mcu_result_cb(self, msg: String) -> None:
        text = msg.data.strip()

        if not text:
            return

        text_upper = text.upper()
        self.get_logger().info(f"[task_manager] mcu_result='{text_upper}'")

        if self._state != "UNLOADING":
            self.get_logger().debug(
                f"[task_manager] ignored mcu_result='{text_upper}' in state={self._state}"
            )
            return

        if text_upper == self.mcu_unload_done.upper():
            self._finish_unload_success()
            return

        if text_upper == self.mcu_unload_failed.upper() or "FAIL" in text_upper:
            self._fail_task(f"UNLOAD_FAILED:{text_upper}")
            return

    # =========================================================
    # Task starters
    # =========================================================
    def _start_outside_button_task(self) -> None:
        if not self._accept_new_task(CMD_OUTSIDE_BTN_FRONT):
            return

        self._active_task = CMD_OUTSIDE_BTN_FRONT
        self._current_button_done_result = RESULT_OUTSIDE_BTN_DONE

        # Select external down button before moving to scan pose.
        self._publish_perception_target(self.perception_outside_down)

        # Clear old marker that may have been created for another target.
        self._publish_button_cmd(BUTTON_CLEAR)

        self._set_state("OUTSIDE_ALIGNING")
        self._set_deadline(self.outside_align_timeout_sec)
        self._publish_arm_cmd(ARM_OUTSIDE_SCAN)

    def _start_inside_button_task(
        self,
        active_task: str,
        perception_target: str,
        done_result: str,
    ) -> None:
        if not self._accept_new_task(active_task):
            return

        self._active_task = active_task
        self._current_button_done_result = done_result

        # Select target button before moving to scan pose.
        self._publish_perception_target(perception_target)

        # Clear old marker that may have been created for another target.
        self._publish_button_cmd(BUTTON_CLEAR)

        self._set_state("INSIDE_ALIGNING")
        self._set_deadline(self.inside_align_timeout_sec)
        self._publish_arm_cmd(ARM_INSIDE_SCAN)

    def _start_destination_unload_task(self) -> None:
        if not self._accept_new_task(CMD_DESTINATION_UNLOAD):
            return

        self._active_task = CMD_DESTINATION_UNLOAD
        self._set_state("UNLOADING")
        self._set_deadline(self.unload_timeout_sec)
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

    # =========================================================
    # Sequence helpers
    # =========================================================
    def _start_marker_settle(self, settle_state: str) -> None:
        self._set_state(settle_state)
        self._delay_until = self.get_clock().now() + Duration(
            seconds=self.marker_settle_sec
        )

        # Marker settle is short, but keep a safe timeout.
        self._set_deadline(max(2.0, self.marker_settle_sec + 1.0))

    def _button_press_cmd_for_state(self, state: str) -> str:
        """Return marker_button_press_commander command for the current button task."""
        if state == "OUTSIDE_PRESSING" or self._active_task == CMD_OUTSIDE_BTN_FRONT:
            return BUTTON_PRESS_OUTSIDE

        if state == "INSIDE_PRESSING" or self._active_task in (
            CMD_INSIDE_BTN_FRONT,
            CMD_INSIDE_B1_BTN_FRONT,
            CMD_INSIDE_3F_BTN_FRONT,
        ):
            return BUTTON_PRESS_INSIDE

        return BUTTON_PRESS

    def _start_button_press(self, state: str) -> None:
        self._set_state(state)
        self._set_deadline(self.button_press_timeout_sec)

        button_cmd = self._button_press_cmd_for_state(state)
        self.get_logger().info(
            f"[task_manager] selected button command='{button_cmd}' "
            f"for state={state}, active_task={self._active_task}"
        )
        self._publish_button_cmd(button_cmd)

    def _finish_button_attempt(self, homing_state: str, done_result: str) -> None:
        """
        버튼 누르기 동작이 성공이든 실패든,
        AMR에는 즉시 완료 플래그를 보내고 로봇팔은 home으로 복귀한다.

        이때 OUTSIDE_BTN_DONE / INSIDE_BTN_DONE의 의미는
        '버튼 성공'이 아니라 '버튼 누르기 시도 완료, AMR 진행 가능'이다.
        """

        # AMR은 이 신호를 받고 다음 동작을 진행할 수 있다.
        self._publish_result(done_result)

        # 로봇팔은 내부적으로 home 복귀를 계속 수행한다.
        if self.return_home_after_button:
            self._start_home(homing_state, result_after_home=None)
            return

        self._reset_to_idle()

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

    # =========================================================
    # FSM tick
    # =========================================================
    def _fsm_tick(self) -> None:
        now = self.get_clock().now()

        # Delayed transition handling.
        if self._delay_until is not None and now >= self._delay_until:
            self._delay_until = None

            if self._state == "OUTSIDE_MARKER_SETTLE":
                self._start_button_press("OUTSIDE_PRESSING")
                return

            if self._state == "INSIDE_MARKER_SETTLE":
                self._start_button_press("INSIDE_PRESSING")
                return

            if self._state == "UNLOADING" and not self.unload_wait_for_result:
                self._finish_unload_success()
                return

        # Timeout handling.
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

        if state == "OUTSIDE_PRESSING":
            self._publish_error("OUTSIDE_PRESS_TIMEOUT")
            self._finish_button_attempt(
                homing_state="OUTSIDE_HOMING",
                done_result=self._current_button_done_result or RESULT_OUTSIDE_BTN_DONE,
            )
            return

        if state == "INSIDE_PRESSING":
            self._publish_error("INSIDE_PRESS_TIMEOUT")
            self._finish_button_attempt(
                homing_state="INSIDE_HOMING",
                done_result=self._current_button_done_result or RESULT_INSIDE_BTN_DONE,
            )
            return

        if state in ("OUTSIDE_HOMING", "INSIDE_HOMING", "UNLOAD_HOMING", "HOME_ONLY"):
            self._fail_task(f"{state}_TIMEOUT")
            return

        if state == "UNLOADING":
            self._fail_task("UNLOAD_TIMEOUT")
            return

        self._fail_task(f"UNKNOWN_TIMEOUT_STATE:{state}")

    # =========================================================
    # Cancel / failure / reset
    # =========================================================
    def _cancel_current_task(self) -> None:
        if self._state == "IDLE":
            self._publish_result(RESULT_CANCELLED)
            return

        self.get_logger().warn(f"[task_manager] cancelling task in state={self._state}")

        self._publish_arm_cmd(ARM_CANCEL)
        self._publish_button_cmd(BUTTON_CANCEL)
        self._publish_mcu_cmd(self.mcu_cancel_cmd)

        self._reset_to_idle()
        self._publish_result(RESULT_CANCELLED)

    def _fail_task(self, reason: str) -> None:
        self.get_logger().warn(f"[task_manager] task failed: {reason}")

        self._publish_arm_cmd(ARM_CANCEL)
        self._publish_button_cmd(BUTTON_CANCEL)

        self._set_state("ERROR")
        self._publish_error(reason)
        self._publish_result(f"FAILED:{reason}")

        if self.auto_reset_after_failure:
            self._reset_to_idle()

    def _reset_to_idle(self) -> None:
        self._active_task = None
        self._deadline = None
        self._delay_until = None
        self._pending_result_after_home = None
        self._current_button_done_result = None
        self._set_state("IDLE")

    def _accept_new_task(self, task_name: str) -> bool:
        if self._state != "IDLE":
            self._publish_result(
                f"BUSY:CURRENT_STATE={self._state},CURRENT_TASK={self._active_task}"
            )
            self.get_logger().warn(
                f"[task_manager] rejected task={task_name}, "
                f"state={self._state}, active_task={self._active_task}"
            )
            return False

        return True

    # =========================================================
    # Publishing helpers
    # =========================================================
    def _publish_perception_target(self, text: str) -> None:
        self._publish_string(self.perception_target_pub, text)
        self.get_logger().info(f"[task_manager] perception_target='{text}'")

    def _publish_arm_cmd(self, text: str) -> None:
        self._publish_string(self.arm_flag_pub, text)
        self.get_logger().info(f"[task_manager] arm_cmd='{text}'")

    def _publish_button_cmd(self, text: str) -> None:
        self._publish_string(self.button_cmd_pub, text)
        self.get_logger().info(f"[task_manager] button_cmd='{text}'")

    def _publish_mcu_cmd(self, text: str) -> None:
        self._publish_string(self.mcu_cmd_pub, text)
        self.get_logger().info(f"[task_manager] mcu_cmd='{text}'")

    def _publish_result(self, text: str) -> None:
        self._publish_string(self.task_result_pub, text)
        self.get_logger().info(f"[task_manager] result='{text}'")

    def _publish_error(self, text: str) -> None:
        self._publish_string(self.task_error_pub, text)
        self.get_logger().warn(f"[task_manager] error='{text}'")

    def _publish_state(self) -> None:
        self._publish_string(self.task_state_pub, self._state)

    def _publish_status(self) -> None:
        status = (
            f"STATUS:STATE={self._state},"
            f"ACTIVE_TASK={self._active_task},"
            f"RETURN_HOME_AFTER_BUTTON={self.return_home_after_button},"
            f"RETURN_HOME_AFTER_UNLOAD={self.return_home_after_unload},"
            f"UNLOAD_WAIT_FOR_RESULT={self.unload_wait_for_result},"
            f"PENDING_RESULT_AFTER_HOME={self._pending_result_after_home},"
            f"CURRENT_BUTTON_DONE_RESULT={self._current_button_done_result},"
            f"BUTTON_CMD_FOR_ACTIVE_STATE={self._button_press_cmd_for_state(self._state)},"
            f"PERCEPTION_TARGET_TOPIC={self.perception_target_topic}"
        )
        self._publish_result(status)

    def _set_state(self, state: str) -> None:
        if self._state != state:
            self.get_logger().info(f"[task_manager] state: {self._state} -> {state}")

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
    node = ManipulatorTaskManager()

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
