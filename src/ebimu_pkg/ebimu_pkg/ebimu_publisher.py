#!/usr/bin/env python3
import copy
import math
import re
import threading
import time
from typing import Any, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from std_msgs.msg import String

import serial
from serial import SerialException


FLOAT_RE = re.compile(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?')


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def diag_covariance(diag: float) -> List[float]:
    return [
        float(diag), 0.0, 0.0,
        0.0, float(diag), 0.0,
        0.0, 0.0, float(diag),
    ]


def unavailable_covariance() -> List[float]:
    return [
        -1.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
    ]


def parse_index_parameter(value: Any) -> List[int]:
    """Accept either [0,1,2] or '0,1,2' so launch files stay easy to edit."""
    if isinstance(value, (list, tuple)):
        return [int(x) for x in value]

    if isinstance(value, str):
        text = value.strip()
        if text.startswith('[') and text.endswith(']'):
            text = text[1:-1]
        if not text:
            return []
        return [int(part.strip()) for part in text.split(',') if part.strip() != '']

    return [int(value)]


class EbimuPublisher(Node):
    def __init__(self) -> None:
        super().__init__('ebimu_publisher')

        self.declare_parameter('port', '/dev/ttyUSB_IMU')
        self.declare_parameter('baudrate', 115200)

        # 기존 층수 계산 노드 호환용 raw topic
        self.declare_parameter('legacy_topic_name', 'ebimu_data')
        # EKF용 IMU topic
        self.declare_parameter('imu_topic_name', '/imu/data')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('invert_gyro_z_with_yaw', True)

        # 기본값은 quick-start 문서의 *roll,pitch,yaw
        self.declare_parameter('rpy_field_indices', '0,1,2')

        # raw gyro / accel 형식을 정확히 알면 인덱스를 지정해서 전부 퍼블리시 가능
        self.declare_parameter('gyro_field_indices', '4, 5, 6')
        self.declare_parameter('accel_field_indices', '7, 8, 9')

        self.declare_parameter('gyro_in_deg_s', True)
        self.declare_parameter('accel_in_g', True)

        self.declare_parameter('orientation_cov_roll_pitch', 0.03)
        self.declare_parameter('orientation_cov_yaw', 0.15)
        self.declare_parameter('angular_velocity_covariance_diag', 0.02)
        self.declare_parameter('linear_acceleration_covariance_diag', 0.20)

        # None / 파싱 실패 시 마지막 정상 IMU를 재발행할 최대 횟수
        # 예: 100Hz면 20회 = 약 0.2초, 50Hz면 약 0.4초
        self.declare_parameter('max_republish_count', 20)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.legacy_topic_name = str(self.get_parameter('legacy_topic_name').value)
        self.imu_topic_name = str(self.get_parameter('imu_topic_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.use_degrees = bool(self.get_parameter('use_degrees').value)
        self.invert_yaw = bool(self.get_parameter('invert_yaw').value)
        self.invert_gyro_z_with_yaw = bool(self.get_parameter('invert_gyro_z_with_yaw').value)

        self.rpy_field_indices = parse_index_parameter(self.get_parameter('rpy_field_indices').value)
        self.gyro_field_indices = parse_index_parameter(self.get_parameter('gyro_field_indices').value)
        self.accel_field_indices = parse_index_parameter(self.get_parameter('accel_field_indices').value)

        self.gyro_in_deg_s = bool(self.get_parameter('gyro_in_deg_s').value)
        self.accel_in_g = bool(self.get_parameter('accel_in_g').value)

        self.orientation_cov_roll_pitch = float(self.get_parameter('orientation_cov_roll_pitch').value)
        self.orientation_cov_yaw = float(self.get_parameter('orientation_cov_yaw').value)
        self.angular_velocity_covariance_diag = float(
            self.get_parameter('angular_velocity_covariance_diag').value
        )
        self.linear_acceleration_covariance_diag = float(
            self.get_parameter('linear_acceleration_covariance_diag').value
        )

        self.max_republish_count = int(self.get_parameter('max_republish_count').value)

        qos = QoSProfile(depth=50)
        self.raw_pub = self.create_publisher(String, self.legacy_topic_name, qos)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic_name, qos)

        self.serial_handle: Optional[serial.Serial] = None
        self.running = True
        self.last_parse_warn_time = 0.0

        # 마지막 정상 IMU 저장용
        self.last_imu_msg: Optional[Imu] = None

        # 연속 파싱 실패 / 재발행 횟수
        self.bad_frame_count = 0
        self.republish_count = 0
        self.last_republish_warn_time = 0.0
        self.last_no_valid_imu_warn_time = 0.0
        self.last_republish_stop_warn_time = 0.0

        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def open_serial(self) -> bool:
        try:
            if self.serial_handle is not None and self.serial_handle.is_open:
                self.serial_handle.close()
        except Exception:
            pass

        try:
            self.serial_handle = serial.Serial(
                port=self.port,
                baudrate=int(self.baudrate),
                timeout=0.1,
            )
            self.serial_handle.reset_input_buffer()
            self.get_logger().info(f'EBIMU connected: {self.port} @ {self.baudrate}')
            return True
        except SerialException as exc:
            self.serial_handle = None
            self.get_logger().warning(f'Failed to open EBIMU serial port {self.port}: {exc}')
            return False

    def normalize_tokens(self, line: str) -> List[str]:
        tokens = [token.strip() for token in line.split(',') if token.strip() != '']
        if not tokens:
            return []

        first = tokens[0]

        # quick-start 예시: *roll,pitch,yaw
        if first and first[0] in ('*', '#'):
            first = first[1:]

        # 예: 2-13.31,33.57,87.70 같이 prefix가 붙는 경우 대응
        if len(first) > 1 and '-' in first[1:]:
            prefix, rest = first.split('-', 1)
            if prefix.isdigit():
                first = rest

        tokens[0] = first
        return tokens

    def token_to_float(self, token: str) -> Optional[float]:
        try:
            return float(token)
        except ValueError:
            match = FLOAT_RE.search(token)
            if match is None:
                return None
            try:
                return float(match.group(0))
            except ValueError:
                return None

    def vector_from_indices(self, tokens: List[str], indices: List[int]) -> Optional[Tuple[float, float, float]]:
        if len(indices) != 3:
            return None

        values = []
        for idx in indices:
            if idx < 0 or idx >= len(tokens):
                return None
            value = self.token_to_float(tokens[idx])
            if value is None:
                return None
            values.append(value)

        return values[0], values[1], values[2]

    def build_imu_msg(self, raw_line: str) -> Optional[Imu]:
        line = raw_line.strip()
        if not line:
            return None

        tokens = self.normalize_tokens(line)
        if not tokens:
            return None

        rpy = self.vector_from_indices(tokens, self.rpy_field_indices)
        if rpy is None:
            return None

        roll, pitch, yaw = rpy

        if self.use_degrees:
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)

        if self.invert_yaw:
            yaw = -yaw

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance = [
            float(self.orientation_cov_roll_pitch), 0.0, 0.0,
            0.0, float(self.orientation_cov_roll_pitch), 0.0,
            0.0, 0.0, float(self.orientation_cov_yaw),
        ]

        gyro = self.vector_from_indices(tokens, self.gyro_field_indices)
        if gyro is None:
            msg.angular_velocity_covariance = unavailable_covariance()
        else:
            gx, gy, gz = gyro
            if self.gyro_in_deg_s:
                gx = math.radians(gx)
                gy = math.radians(gy)
                gz = math.radians(gz)

            # yaw 방향을 뒤집었다면 z축 각속도도 같이 뒤집어야 orientation과 일관성이 맞음
            if self.invert_yaw and self.invert_gyro_z_with_yaw:
                gz = -gz

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            msg.angular_velocity_covariance = diag_covariance(self.angular_velocity_covariance_diag)

        accel = self.vector_from_indices(tokens, self.accel_field_indices)
        if accel is None:
            msg.linear_acceleration_covariance = unavailable_covariance()
        else:
            ax, ay, az = accel
            if self.accel_in_g:
                ax *= 9.80665
                ay *= 9.80665
                az *= 9.80665

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.linear_acceleration_covariance = diag_covariance(
                self.linear_acceleration_covariance_diag
            )

        return msg

    def publish_valid_imu(self, imu_msg: Imu) -> None:
        """
        정상적으로 파싱된 IMU를 publish하고 마지막 정상 IMU로 저장한다.
        """
        self.imu_pub.publish(imu_msg)

        # 이후 None / 파싱 실패가 들어왔을 때 재사용하기 위해 깊은 복사로 저장
        self.last_imu_msg = copy.deepcopy(imu_msg)

        # 정상 프레임이 들어왔으므로 실패/재발행 카운터 초기화
        self.bad_frame_count = 0
        self.republish_count = 0

    def republish_last_imu(self, reason: str) -> None:
        """
        None, empty line, parsing failure 등이 발생했을 때
        마지막 정상 IMU 값을 현재 timestamp로 갱신해서 다시 publish한다.
        단, 무한히 재발행하지 않도록 max_republish_count까지만 허용한다.
        """
        self.bad_frame_count += 1
        now = time.time()

        if self.last_imu_msg is None:
            if now - self.last_no_valid_imu_warn_time > 5.0:
                self.get_logger().warning(
                    f'No valid IMU message has been received yet. '
                    f'Cannot republish last IMU. reason={reason}'
                )
                self.last_no_valid_imu_warn_time = now
            return

        if self.republish_count >= self.max_republish_count:
            if now - self.last_republish_stop_warn_time > 5.0:
                self.get_logger().warning(
                    f'IMU input has been invalid for too long. '
                    f'Stop republishing last IMU. '
                    f'bad_frame_count={self.bad_frame_count}, '
                    f'republish_count={self.republish_count}, '
                    f'max_republish_count={self.max_republish_count}, '
                    f'reason={reason}'
                )
                self.last_republish_stop_warn_time = now
            return

        try:
            msg = copy.deepcopy(self.last_imu_msg)

            # EKF가 오래된 timestamp로 판단하지 않도록 현재 시간으로 갱신
            msg.header.stamp = self.get_clock().now().to_msg()

            self.imu_pub.publish(msg)
            self.republish_count += 1

            if now - self.last_republish_warn_time > 5.0:
                self.get_logger().warning(
                    f'Republishing last valid IMU because current EBIMU frame is invalid. '
                    f'republish_count={self.republish_count}/'
                    f'{self.max_republish_count}, reason={reason}'
                )
                self.last_republish_warn_time = now

        except Exception as exc:
            self.get_logger().warning(f'Failed to republish last valid IMU: {exc}')

    def publish_raw_line(self, raw_line: str) -> None:
        """
        기존 elevator_floor_node 호환을 위해 raw ebimu_data는 계속 publish한다.
        """
        raw_msg = String()
        raw_msg.data = raw_line
        self.raw_pub.publish(raw_msg)

    def read_loop(self) -> None:
        while self.running:
            if self.serial_handle is None or not self.serial_handle.is_open:
                if not self.open_serial():
                    time.sleep(1.0)
                    continue

            try:
                raw = self.serial_handle.readline()

                if raw is None or len(raw) == 0:
                    # serial timeout 또는 순간적인 empty read
                    # 기존 코드는 그냥 continue였지만, 여기서는 마지막 정상 IMU를 재발행
                    self.republish_last_imu('empty serial read')
                    time.sleep(0.005)
                    continue

                raw_line = raw.decode('utf-8', errors='ignore')

                if raw_line is None or raw_line == '':
                    self.republish_last_imu('empty decoded line')
                    continue

                # 기존 층수 계산 노드 호환용 raw topic은 계속 publish
                self.publish_raw_line(raw_line)

                imu_msg = self.build_imu_msg(raw_line)

                if imu_msg is not None:
                    self.publish_valid_imu(imu_msg)
                else:
                    # roll/pitch/yaw 파싱 실패 시 마지막 정상 IMU 재발행
                    self.republish_last_imu('failed to parse roll/pitch/yaw from EBIMU line')

                    now = time.time()
                    if now - self.last_parse_warn_time > 5.0:
                        self.get_logger().warning(
                            'Could not parse roll/pitch/yaw for /imu/data from the current EBIMU line. '
                            'Republishing the last valid IMU message if available. '
                            'Raw ebimu_data is still being published. '
                            'If your EBIMU output format is not *roll,pitch,yaw, change rpy_field_indices.'
                        )
                        self.last_parse_warn_time = now

            except SerialException as exc:
                self.get_logger().error(f'EBIMU serial read error: {exc}')

                # serial exception 순간에도 마지막 정상 IMU를 짧게 재발행
                self.republish_last_imu(f'serial exception: {exc}')

                try:
                    if self.serial_handle is not None:
                        self.serial_handle.close()
                except Exception:
                    pass

                self.serial_handle = None
                time.sleep(1.0)

            except Exception as exc:
                # 예상치 못한 예외가 발생해도 노드를 죽이지 않고 마지막 정상 IMU를 재발행
                self.get_logger().warning(f'Unexpected EBIMU handling error: {exc}')
                self.republish_last_imu(f'unexpected exception: {exc}')
                time.sleep(0.01)

    def destroy_node(self):
        self.running = False

        try:
            if self.serial_handle is not None and self.serial_handle.is_open:
                self.serial_handle.close()
        except Exception:
            pass

        try:
            if self.read_thread.is_alive():
                self.read_thread.join(timeout=1.0)
        except Exception:
            pass

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EbimuPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        # Ctrl+C / launch 종료 과정에서 이미 shutdown된 경우 중복 shutdown 방지
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()