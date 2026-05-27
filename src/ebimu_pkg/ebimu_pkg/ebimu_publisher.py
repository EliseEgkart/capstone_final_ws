#!/usr/bin/env python3
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

        qos = QoSProfile(depth=50)
        self.raw_pub = self.create_publisher(String, self.legacy_topic_name, qos)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic_name, qos)

        self.serial_handle: Optional[serial.Serial] = None
        self.running = True
        self.last_parse_warn_time = 0.0

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

    def read_loop(self) -> None:
        while self.running:
            if self.serial_handle is None or not self.serial_handle.is_open:
                if not self.open_serial():
                    time.sleep(1.0)
                    continue

            try:
                raw = self.serial_handle.readline()
                if not raw:
                    time.sleep(0.005)
                    continue

                raw_line = raw.decode('utf-8', errors='ignore')
                if not raw_line:
                    continue

                raw_msg = String()
                raw_msg.data = raw_line
                self.raw_pub.publish(raw_msg)

                imu_msg = self.build_imu_msg(raw_line)
                if imu_msg is not None:
                    self.imu_pub.publish(imu_msg)
                else:
                    now = time.time()
                    if now - self.last_parse_warn_time > 5.0:
                        self.get_logger().warning(
                            'Could not parse roll/pitch/yaw for /imu/data from the current EBIMU line. '
                            'Raw ebimu_data is still being published. '
                            'If your EBIMU output format is not *roll,pitch,yaw, change rpy_field_indices.'
                        )
                        self.last_parse_warn_time = now

            except SerialException as exc:
                self.get_logger().error(f'EBIMU serial read error: {exc}')
                try:
                    if self.serial_handle is not None:
                        self.serial_handle.close()
                except Exception:
                    pass
                self.serial_handle = None
                time.sleep(1.0)

            except Exception as exc:
                self.get_logger().warning(f'Unexpected EBIMU handling error: {exc}')

    def destroy_node(self):
        self.running = False

        try:
            if self.serial_handle is not None and self.serial_handle.is_open:
                self.serial_handle.close()
        except Exception:
            pass

        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()