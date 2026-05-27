import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import os

class ScanToTxt(Node):
    def __init__(self):
        super().__init__('scan_to_txt')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.file = open("scan_data.txt", "w")

    def listener_callback(self, msg):
        # 메시지의 시간 스탬프 사용
        timestamp = msg.header.stamp
        sec = timestamp.sec
        nanosec = timestamp.nanosec

        # 데이터 포맷에 맞게 문자열로 변환하여 파일에 저장
        data_str = f"time: {sec}.{nanosec}, "
        data_str += f"ranges: {list(msg.ranges)}, intensities: {list(msg.intensities)}\n"
        self.file.write(data_str)
    
    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    scan_to_txt = ScanToTxt()
    try:
        rclpy.spin(scan_to_txt)
    except KeyboardInterrupt:
        pass
    finally:
        scan_to_txt.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()