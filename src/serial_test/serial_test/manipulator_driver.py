import time
import serial
import numpy as np


class MotorDriver:
    def __init__(self, port='/dev/ttyUSB_ARM', baudrate=115200, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

        self.connect()

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2.0)  # ESP32 reset stabilization
            print(f"[INFO] Serial connected: {self.port} @ {self.baudrate}")
        except serial.SerialException as e:
            print(f"[ERROR] Failed to open serial port: {e}")
            self.ser = None

    def disconnect(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            print("[INFO] Serial disconnected")

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def send_raw(self, message: str):
        if not self.is_connected():
            print("[WARN] Serial is not connected")
            return False

        try:
            if not message.endswith('\n'):
                message += '\n'
            self.ser.write(message.encode('utf-8'))
            self.ser.flush()
            print(f"[TX] {message.strip()}")
            return True
        except serial.SerialException as e:
            print(f"[ERROR] Serial write failed: {e}")
            return False

    def read_line(self):
        if not self.is_connected():
            print("[WARN] Serial is not connected")
            return None

        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[RX] {line}")
                return line
            return None
        except serial.SerialException as e:
            print(f"[ERROR] Serial read failed: {e}")
            return None

    def send_joint_angles(self, joint_angles):
        """
        joint_angles: list/tuple/np.ndarray with length 4
        example: [90, 45, 120, 30]
        """
        if len(joint_angles) != 4:
            raise ValueError("joint_angles must contain exactly 4 values")

        joint_angles = np.array(joint_angles, dtype=float)

        # 안전 범위 예시 (필요 시 수정)
        joint_angles = np.clip(joint_angles, 0, 180)

        cmd = "ARM,{:.2f},{:.2f},{:.2f},{:.2f}".format(
            joint_angles[0],
            joint_angles[1],
            joint_angles[2],
            joint_angles[3]
        )

        return self.send_raw(cmd)

    def request_state(self):
        """
        현재 로봇팔 상태 요청
        ESP32는 예를 들어:
        STATE,90.0,45.0,120.0,30.0
        형태로 응답한다고 가정
        """
        success = self.send_raw("GET_STATE")
        if not success:
            return None

        response = self.read_line()
        if response is None:
            return None

        return self.parse_state(response)

    def parse_state(self, response: str):
        """
        response examples:
        STATE,90.0,45.0,120.0,30.0
        OK,90.0,45.0,120.0,30.0
        """
        try:
            tokens = response.split(',')

            if len(tokens) != 5:
                print(f"[WARN] Invalid response format: {response}")
                return None

            header = tokens[0].strip()
            if header not in ["STATE", "OK"]:
                print(f"[WARN] Unknown header: {header}")
                return None

            angles = [float(tokens[i]) for i in range(1, 5)]

            state = {
                "header": header,
                "joint1": angles[0],
                "joint2": angles[1],
                "joint3": angles[2],
                "joint4": angles[3]
            }
            return state

        except Exception as e:
            print(f"[ERROR] Failed to parse state: {e}")
            return None

    def move_and_wait_ack(self, joint_angles, max_wait=2.0):
        """
        명령 전송 후 ACK/OK 응답 대기
        """
        success = self.send_joint_angles(joint_angles)
        if not success:
            return False, None

        start_time = time.time()

        while time.time() - start_time < max_wait:
            response = self.read_line()
            if response is None:
                continue

            parsed = self.parse_state(response)
            if parsed is not None:
                return True, parsed

        print("[WARN] ACK timeout")
        return False, None


if __name__ == "__main__":
    driver = MotorDriver(port='/dev/ttyUSB_ARM', baudrate=115200, timeout=0.5)

    if driver.is_connected():
        # 예시 1: 원하는 각도 전송
        driver.send_joint_angles([90, 45, 120, 30])

        time.sleep(1.0)

        # 예시 2: 상태 요청
        state = driver.request_state()
        print("[STATE]", state)

        # 예시 3: 명령 후 ACK 대기
        ok, ack = driver.move_and_wait_ack([100, 60, 110, 40], max_wait=3.0)
        print("[ACK RESULT]", ok, ack)

    driver.disconnect()