import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SerialFlagBridge(Node):
    def __init__(self):
        super().__init__('serial_flag_bridge')
        self.declare_parameter('port', '/dev/ttyUSB4')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('line_ending', '\n')
        self.declare_parameter('tx_topic', 'serial_tx')
        self.declare_parameter('rx_topic', 'serial_rx')
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('reconnect_sec', 1.0)
        self.declare_parameter('log_rx', True)
        self.declare_parameter('log_tx', True)

        self._port = self.get_parameter('port').value
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._timeout = float(self.get_parameter('timeout').value)
        self._line_ending = self.get_parameter('line_ending').value
        self._auto_reconnect = bool(self.get_parameter('auto_reconnect').value)
        self._reconnect_sec = float(self.get_parameter('reconnect_sec').value)
        self._log_rx = bool(self.get_parameter('log_rx').value)
        self._log_tx = bool(self.get_parameter('log_tx').value)

        self._tx_topic = self.get_parameter('tx_topic').value
        self._rx_topic = self.get_parameter('rx_topic').value

        self._ser = None
        self._lock = threading.Lock()
        self._last_reconnect = 0.0

        self._rx_pub = self.create_publisher(String, self._rx_topic, 10)
        self._tx_sub = self.create_subscription(String, self._tx_topic, self._tx_callback, 10)

        self._read_timer = self.create_timer(0.02, self._read_serial)
        self._ensure_serial()

    def _ensure_serial(self):
        if self._ser is not None and self._ser.is_open:
            return

        now = time.time()
        if not self._auto_reconnect and self._ser is not None:
            return
        if now - self._last_reconnect < self._reconnect_sec:
            return

        self._last_reconnect = now
        try:
            self._ser = serial.Serial(self._port, self._baudrate, timeout=self._timeout)
            self.get_logger().info(f'Serial connected: {self._port} @ {self._baudrate}')
        except Exception as exc:
            self._ser = None
            self.get_logger().warn(f'Failed to open serial {self._port}: {exc}')

    def _tx_callback(self, msg: String):
        self._ensure_serial()
        if self._ser is None:
            self.get_logger().warn('Serial not connected; drop TX message')
            return

        data = msg.data
        if self._line_ending and not data.endswith(self._line_ending):
            data = f'{data}{self._line_ending}'

        try:
            with self._lock:
                self._ser.write(data.encode('utf-8'))
            if self._log_tx:
                self.get_logger().info(f'TX: {msg.data}')
        except Exception as exc:
            self.get_logger().warn(f'Failed to write to serial: {exc}')
            self._close_serial()

    def _read_serial(self):
        self._ensure_serial()
        if self._ser is None:
            return

        try:
            with self._lock:
                line = self._ser.readline()
            if not line:
                return
            text = line.decode('utf-8', errors='ignore').strip('\r\n')
            if text == '':
                return
            msg = String()
            msg.data = text
            self._rx_pub.publish(msg)
            if self._log_rx:
                self.get_logger().info(f'RX: {text}')
        except Exception as exc:
            self.get_logger().warn(f'Failed to read from serial: {exc}')
            self._close_serial()

    def _close_serial(self):
        if self._ser is None:
            return
        try:
            if self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self._ser = None

    def destroy_node(self):
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialFlagBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
