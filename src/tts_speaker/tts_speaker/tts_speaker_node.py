#!/usr/bin/env python3
#=====================================================#
# 기능: ROS2 TTS Speaker
# - /speaker 토픽(String)을 구독하여 효과음/음성 출력
# - alarm_on 동안 문구를 '끝까지' 재생하고, 끝난 뒤에도
#   가림 상태가 지속되면 다음 회차를 이어서 재생
# - alarm_off 시 즉시 정지
#
# 최종 수정일: 2025.11.11
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

        # 오디오 초기화
        try:
            mixer.init()
        except Exception as e:
            self.get_logger().error(f"[Speaker] mixer.init() failed: {e}")

        # 파라미터: 사운드 폴더/파일명
        self.declare_parameter('sound_path', '')
        # alarm_on에 사용할 파일을 기본적으로 voice_test.mp3로 매핑
        self.declare_parameter('alarm_effect', 'voice_test.mp3')
        self.declare_parameter('voice_file', 'voice_test.mp3')  # test_voice용

        self.sound_path = self.get_parameter('sound_path').get_parameter_value().string_value
        if not self.sound_path:
            self.sound_path = os.path.join(os.getcwd(), 'sounds')

        self.alarm_effect = self.get_parameter('alarm_effect').get_parameter_value().string_value
        self.voice_file   = self.get_parameter('voice_file').get_parameter_value().string_value

        self.get_logger().info(f"[Speaker] Using sound path: {self.sound_path}")
        self.get_logger().info(f"[Speaker] alarm_effect: {self.alarm_effect}, voice_file: {self.voice_file}")

        # 알람 반복 요청 상태 플래그
        self.alarm_requested = False

        # 구독자
        self.sub_speaker = self.create_subscription(String, '/speaker', self.play_sound_callback, 5)

        # 0.1초 주기 모니터 타이머:
        # - 재생 중이면 아무것도 하지 않음(중복 로드 방지)
        # - 재생이 끝났고 alarm_requested=True면 한 번만 재생
        self.monitor_timer = self.create_timer(0.1, self._monitor_alarm_loop)

    # -------- 내부 도우미 --------
    def _play_file(self, name: str, repeat: int):
        """repeat: 0=한 번, -1=무한 반복"""
        path = os.path.join(self.sound_path, name)
        if not os.path.isfile(path):
            self.get_logger().warn(f"[Speaker] Sound file not found: {path}")
            return
        try:
            mixer.music.load(path)
            mixer.music.play(repeat)
        except Exception as e:
            self.get_logger().error(f"[Speaker] play failed: {e}")

    def _monitor_alarm_loop(self):
        # 알람 반복이 요청되어 있고 현재 재생이 끝났을 때만 다음 회차 시작
        try:
            busy = mixer.music.get_busy()
        except Exception:
            busy = False

        if self.alarm_requested and not busy:
            # 한 회차만 재생 -> 끝나면 다시 이 타이머가 조건 확인 후 다음 회차 재생
            self._play_file(self.alarm_effect, 0)

    # -------- 콜백 --------
    def play_sound_callback(self, msg: String):
        data = (msg.data or "").strip()
        self.get_logger().info(f"[Speaker] Received: {data}")

        if data == "test_effect":
            # 필요 시 테스트용 무한 반복
            self._play_file(self.alarm_effect, -1)

        elif data == "test_voice":
            # 단발로 한 번만 재생(끝까지)
            self._play_file(self.voice_file, 0)
            # 다른 콜백 블로킹 방지를 원하면 아래 대기 루프는 제거 가능
            while True:
                try:
                    if not mixer.music.get_busy():
                        break
                except Exception:
                    break
                time.sleep(0.1)

        elif data == "alarm_on":
            # 반복 요청만 켜고, 현재 재생 중이면 건드리지 않음(중간 끊김 방지)
            self.alarm_requested = True
            try:
                if not mixer.music.get_busy():
                    self._play_file(self.alarm_effect, 0)
            except Exception:
                # mixer 상태 조회 실패 시 안전하게 1회 재생 시도
                self._play_file(self.alarm_effect, 0)

        elif data == "alarm_off":
            # 반복 요청 해제 및 즉시 정지
            self.alarm_requested = False
            try:
                mixer.music.stop()
            except Exception as e:
                self.get_logger().warn(f"[Speaker] stop failed: {e}")

        else:
            self.get_logger().warn(f"[Speaker] Undefined message: {data}")

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
