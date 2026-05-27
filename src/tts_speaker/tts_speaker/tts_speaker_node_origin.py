#!/usr/bin/env python3

#=====================================================#
# 기능: ROS2 TTS Speaker
# - 포트를 설정하고 출력 장치에 저장해놓은 음성 및 사운드 데이터 출력
# - 다른 노드로 부터 String 트리거를 받아서 해당 함수에서 검증 및 출력
#
# TODO : speaker class 추가 구성
# 최종 수정일: 2025.11.09
# 편집자: 김형진
#=====================================================#

import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pygame import mixer

class Speaker(Node):
    def __init__(self):
        super().__init__('speaker_node')
        mixer.init()

        # 파라미터: 사운드 폴더/파일명
        self.declare_parameter('sound_path', '')
        self.declare_parameter('alarm_effect', 'alarm_test.mp3')  # 알람 기본 파일
        self.declare_parameter('voice_file', 'voice_test.mp3')    # 테스트 음성 기본 파일

        self.sound_path = self.get_parameter('sound_path').get_parameter_value().string_value
        if not self.sound_path:
            self.sound_path = os.path.join(os.getcwd(), 'sounds')

        self.alarm_effect = self.get_parameter('alarm_effect').get_parameter_value().string_value
        self.voice_file   = self.get_parameter('voice_file').get_parameter_value().string_value

        self.get_logger().info(f"[Speaker] Using sound path: {self.sound_path}")
        self.get_logger().info(f"[Speaker] alarm_effect: {self.alarm_effect}, voice_file: {self.voice_file}")

        self.sub_speaker = self.create_subscription(String, '/speaker', self.play_sound_callback, 5)

    def _play_file(self, name: str, repeat: int):
        path = os.path.join(self.sound_path, name)
        if not os.path.isfile(path):
            self.get_logger().warn(f"Sound file not found: {path}")
            return
        mixer.music.load(path)
        mixer.music.play(repeat)

    def play_sound_callback(self, msg: String):
        data = msg.data.strip()
        self.get_logger().info(f"[Speaker] Received: {data}")

        if data == "test_effect":
            self._play_file(self.alarm_effect, -1)

        elif data == "test_voice":
            self._play_file(self.voice_file, 1)
            while mixer.music.get_busy():
                time.sleep(0.1)

        elif data == "alarm_on":
            self._play_file(self.voice_file, -1)  # 무한 반복

        elif data == "alarm_off":
            mixer.music.stop()

        else:
            self.get_logger().warn(f"Undefined message: {data}")

def main(args=None):
    rclpy.init(args=args)
    node = Speaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Speaker Node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


