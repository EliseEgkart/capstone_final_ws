#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Int32


# 내부 인덱스:
# 0 = B1
# 1 = 1F
# 2 = 2F
# 3 = 3F
# 4 = 4F
# 5 = 5F
VALID_FLOOR_INDEX = set(range(0, 6))


def index_to_label(i: int) -> str:
    if i == 0:
        return 'B1'
    return f'{i}'


class ElevatorFloorNode(Node):
    """
    엘리베이터 층수 추정 노드.

    기존 Moonshot elevator_floor_node.py와 달리
    터미널에서 시작층/목적층/s 입력을 받지 않습니다.

    대신 미션 매니저가 다음 토픽으로 목적 층을 알려줍니다.

        /elevator/start   std_msgs/Int32

    예:
        data: 3

    그러면 이 노드는
        시작층 = 1층
        목적층 = 3층
    으로 세션을 시작하고, 추정된 현재층을 다음 토픽으로 publish합니다.

        /current_floor    std_msgs/Int32
    """

    def __init__(self):
        super().__init__('elevator_floor_node')

        # -------------------------------
        # Parameters
        # -------------------------------
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('start_floor', 1)

        self.declare_parameter('acc_z_threshold', 0.02)
        self.declare_parameter('thresh_count', 30)
        self.declare_parameter('window', 0.5)

        # 현재 EBIMU /imu/data의 linear_acceleration.z에 중력 또는 고정 offset이 포함될 수 있으므로
        # 세션 시작 직후 일정 시간 z축 평균을 baseline으로 잡고 빼줍니다.
        self.declare_parameter('use_baseline_compensation', True)
        self.declare_parameter('baseline_duration', 0.8)

        imu_topic = str(self.get_parameter('imu_topic').value)
        self.start_floor_idx = int(self.get_parameter('start_floor').value)

        if self.start_floor_idx not in VALID_FLOOR_INDEX:
            self.get_logger().warning(
                f"Invalid start_floor={self.start_floor_idx}. Force start_floor=1."
            )
            self.start_floor_idx = 1

        self.ACC_Z_THRESHOLD = float(self.get_parameter('acc_z_threshold').value)
        self.THRESH_COUNT = int(self.get_parameter('thresh_count').value)
        self.WINDOW = float(self.get_parameter('window').value)

        self.use_baseline_compensation = bool(
            self.get_parameter('use_baseline_compensation').value
        )
        self.baseline_duration = float(self.get_parameter('baseline_duration').value)

        # -------------------------------
        # State
        # -------------------------------
        self.current_floor_idx = self.start_floor_idx
        self.dest_floor_idx: Optional[int] = None

        self.active = False
        self.arrival_logged = False

        self.consecutive_up = 0
        self.consecutive_down = 0

        self.timer_running = False
        self.timer_start_time = 0.0
        self.elevator_mode: Optional[str] = None  # "UP" or "DOWN"

        self.window_start: Optional[float] = None
        self.count_up = 0
        self.count_down = 0
        self.count_still = 0

        # baseline
        self.baseline_ready = not self.use_baseline_compensation
        self.baseline_start_time: Optional[float] = None
        self.baseline_samples: List[float] = []
        self.z_baseline = 0.0

        # -------------------------------
        # QoS
        # -------------------------------
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        # -------------------------------
        # ROS I/O
        # -------------------------------
        self.sub_imu = self.create_subscription(
            Imu,
            imu_topic,
            self.on_imu,
            imu_qos
        )

        self.sub_start = self.create_subscription(
            Int32,
            '/elevator/start',
            self.on_start_command,
            10
        )

        self.pub_floor = self.create_publisher(
            Int32,
            '/current_floor',
            10
        )

        # 미션 시작 전에도 현재 기본층 1층을 주기적으로 publish
        self.create_timer(1.0, self.publish_current_floor)

        self.get_logger().info(
            "[ElevatorFloorNode] ready. "
            f"imu_topic='{imu_topic}', "
            f"start_floor={index_to_label(self.start_floor_idx)}, "
            f"ACC_Z_THRESHOLD={self.ACC_Z_THRESHOLD}, "
            f"THRESH_COUNT={self.THRESH_COUNT}, "
            f"WINDOW={self.WINDOW}, "
            f"use_baseline_compensation={self.use_baseline_compensation}, "
            f"baseline_duration={self.baseline_duration}"
        )

    # -------------------------------
    # Start command
    # -------------------------------
    def on_start_command(self, msg: Int32):
        dest_floor_idx = int(msg.data)

        if dest_floor_idx not in VALID_FLOOR_INDEX:
            self.get_logger().error(
                f"Invalid destination floor index: {dest_floor_idx}. "
                "Allowed: 0(B1), 1, 2, 3, 4, 5"
            )
            return

        self.start_session(self.start_floor_idx, dest_floor_idx)

    def start_session(self, start_floor_idx: int, dest_floor_idx: int):
        self.current_floor_idx = int(start_floor_idx)
        self.dest_floor_idx = int(dest_floor_idx)

        self.active = True
        self.arrival_logged = False

        self.consecutive_up = 0
        self.consecutive_down = 0

        self.timer_running = False
        self.timer_start_time = 0.0
        self.elevator_mode = None

        self.window_start = None
        self.count_up = 0
        self.count_down = 0
        self.count_still = 0

        self.baseline_ready = not self.use_baseline_compensation
        self.baseline_start_time = None
        self.baseline_samples = []
        self.z_baseline = 0.0

        start_label = index_to_label(self.current_floor_idx)
        dest_label = index_to_label(self.dest_floor_idx)

        msg_txt = (
            f"[START] 엘리베이터 층수 추정 시작. "
            f"시작 층={start_label}, 목적 층={dest_label}"
        )
        print(msg_txt)
        self.get_logger().info(msg_txt)

        self.publish_current_floor()

    # -------------------------------
    # Main IMU callback
    # -------------------------------
    def on_imu(self, msg: Imu):
        if not self.active:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        raw_z = float(msg.linear_acceleration.z)

        # -------------------------------
        # Baseline compensation
        # -------------------------------
        if not self.baseline_ready:
            if self.baseline_start_time is None:
                self.baseline_start_time = now
                self.baseline_samples = []
                self.get_logger().info(
                    f"Collecting z acceleration baseline for {self.baseline_duration:.2f}s..."
                )

            self.baseline_samples.append(raw_z)

            if (now - self.baseline_start_time) < self.baseline_duration:
                return

            if len(self.baseline_samples) > 0:
                self.z_baseline = sum(self.baseline_samples) / len(self.baseline_samples)
            else:
                self.z_baseline = 0.0

            self.baseline_ready = True
            self.window_start = now
            self.count_up = 0
            self.count_down = 0
            self.count_still = 0

            self.get_logger().info(
                f"Baseline ready. z_baseline={self.z_baseline:.5f}"
            )
            return

        if self.use_baseline_compensation:
            z_acc = raw_z - self.z_baseline
        else:
            z_acc = raw_z

        # -------------------------------
        # Window init
        # -------------------------------
        if self.window_start is None:
            self.window_start = now

        # -------------------------------
        # Count z direction
        # -------------------------------
        if z_acc >= self.ACC_Z_THRESHOLD:
            self.count_up += 1
        elif z_acc <= -self.ACC_Z_THRESHOLD:
            self.count_down += 1
        else:
            self.count_still += 1

        # -------------------------------
        # Every WINDOW seconds, decide state
        # -------------------------------
        if (now - self.window_start) < self.WINDOW:
            return

        if self.count_down >= self.THRESH_COUNT:
            state = "내려갑니다"
        elif self.count_up >= self.THRESH_COUNT:
            state = "올라갑니다"
        else:
            state = "z축 등속운동 진행"

        # 연속 상태 카운트
        if state == "올라갑니다":
            self.consecutive_up += 1
            self.consecutive_down = 0
        elif state == "내려갑니다":
            self.consecutive_down += 1
            self.consecutive_up = 0
        else:
            self.consecutive_up = 0
            self.consecutive_down = 0

        # -------------------------------
        # Timer finish
        # -------------------------------
        if self.timer_running:
            if self.elevator_mode == "UP" and self.consecutive_down >= 3:
                self.finish_timer(now, "UP")
            elif self.elevator_mode == "DOWN" and self.consecutive_up >= 3:
                self.finish_timer(now, "DOWN")

        # -------------------------------
        # Timer start
        # -------------------------------
        if not self.timer_running:
            if state == "올라갑니다" and self.consecutive_up >= 3:
                self.start_timer(now, "UP")
            elif state == "내려갑니다" and self.consecutive_down >= 3:
                self.start_timer(now, "DOWN")

        label = index_to_label(self.current_floor_idx)
        msg_txt = (
            f"[현재 층: {label}], "
            f"[상태: {state}], "
            f"[raw_z={raw_z:.4f}, z_acc={z_acc:.4f}]"
        )
        print(msg_txt)
        self.get_logger().info(msg_txt)

        self.publish_current_floor()

        self.window_start = now
        self.count_up = 0
        self.count_down = 0
        self.count_still = 0

    # -------------------------------
    # Timer logic
    # -------------------------------
    def start_timer(self, now: float, mode: str):
        self.elevator_mode = mode
        self.timer_running = True
        self.timer_start_time = now

        msg_txt = (
            f"타이머 시작: 모드={mode}, "
            f"시작시간={time.strftime('%H:%M:%S', time.localtime(self.timer_start_time))}"
        )
        print(msg_txt)
        self.get_logger().info(msg_txt)

    def finish_timer(self, now: float, mode: str):
        self.timer_running = False
        elapsed = now - self.timer_start_time

        diff = self.floor_diff_from_elapsed(elapsed)

        if mode == "UP":
            self.current_floor_idx += diff
            sign = "+"
        else:
            self.current_floor_idx -= diff
            sign = "-"

        self.current_floor_idx = max(0, min(5, self.current_floor_idx))

        self.consecutive_up = 0
        self.consecutive_down = 0

        label = index_to_label(self.current_floor_idx)
        msg_txt = (
            f"타이머 종료: 모드={mode}, 경과={elapsed:.2f}s, "
            f"층변경={sign}{diff}, 현재층={label}"
        )
        print(msg_txt)
        self.get_logger().info(msg_txt)

        self.publish_current_floor()
        self.check_arrival()

    @staticmethod
    def floor_diff_from_elapsed(elapsed: float) -> int:
        # 기존 Moonshot 로직의 시간 기준 유지
        if elapsed >= 15.3:
            return 5
        if elapsed >= 12.5:
            return 4
        if elapsed >= 8.5:
            return 3
        if elapsed >= 5.8:
            return 2
        if elapsed >= 2.6:
            return 1
        return 0

    # -------------------------------
    # Publish / arrival
    # -------------------------------
    def publish_current_floor(self):
        msg = Int32()
        msg.data = int(self.current_floor_idx)
        self.pub_floor.publish(msg)

    def check_arrival(self):
        if self.dest_floor_idx is None:
            return

        if self.current_floor_idx == self.dest_floor_idx and not self.arrival_logged:
            label = index_to_label(self.dest_floor_idx)
            msg_txt = f"목적층인 {label}층에 도달했습니다."
            print(msg_txt)
            self.get_logger().info(msg_txt)
            self.arrival_logged = True


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorFloorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: elevator_floor_node 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()