#!/usr/bin/env python3
import time
import roslibpy
import json

# ROS 서버의 호스트와 포트를 설정합니다.
ros_host = 'localhost'
ros_port = 9090

# ROS 클라이언트를 초기화하고 실행합니다.
client = roslibpy.Ros(host=ros_host, port=ros_port)
client.run()

# 토픽 경로를 정의합니다.
from_ros2 = '/from_ros2'
from_web = '/from_web2'

# AMR(Autonomous Mobile Robot)의 데이터를 정의합니다.
amr_data = {
    "AMR_number": 3,
    "AMR_position": {"x": 300, "y": 400},
    "battery_status": {"battery": 90, "remaining_distance": 13.8, "remaining_time": 10.4},
    "current_status": "Loading",
    "AMR_destination_info": {"cell": 1, "row": 1, "column": 1},
    "loaded_item_info": {"cell": 7, "row": 2, "column": 3}
}
amr_data_json = json.dumps(amr_data)

# Publisher 및 Subscriber를 초기화하고 토픽 형식을 설정합니다.
publisher = roslibpy.Topic(client, from_ros2, 'std_msgs/String')
subscriber = roslibpy.Topic(client, from_web, 'std_msgs/String')

def publisher_callback():
    """AMR 데이터를 관제 시스템에 보내는 콜백 함수입니다."""
    publisher.publish(roslibpy.Message({'data': amr_data_json}))
    print('Sending AMR data...')

def subscriber_callback(message):
    """관제 시스템에서 보낸 데이터를 받는 콜백 함수입니다."""
    print(f'Received message: {message}')

# Subscriber 콜백 함수를 등록합니다.
subscriber.subscribe(subscriber_callback)

try:
    # 클라이언트가 연결되어 있는 동안 데이터를 주기적으로 보냅니다.
    while client.is_connected:
        publisher_callback()
        time.sleep(1)
except KeyboardInterrupt:
    # 사용자가 Ctrl+C를 누를 경우, 종료 메시지를 출력하고 종료합니다.
    print("Interrupted by user, shutting down.")
finally:
    # 프로그램 종료 시 모든 리소스를 정리합니다.
    publisher.unadvertise()
    subscriber.unsubscribe()
    client.terminate()
