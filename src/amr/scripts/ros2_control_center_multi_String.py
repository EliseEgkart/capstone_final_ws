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
from_web = '/from_web'

# 많은 자율주행 노드들에서부터 관제 시스템에 보낼 데이터 수집(subscribe)
amr_position_topic='/possarray'
amr_battery_topic='/STM32_battery'

# AMR 데이터 수집및 정의(여러 amr_date->string 배열로 가공)
amr_data_list = [
    {
        "AMR_number": 1,
        "AMR_position": {"x": 100, "y": 200},
        "battery_status": {"battery": 85, "remaining_distance": 15.5, "remaining_time": 12.3},
        "current_status": "Moving",
        "AMR_destination_info": {"cell": 9, "row": 1, "column": 4},
        "loaded_item_info": {"cell": 5, "row": 6, "column": 7}
    },
    {
        "AMR_number": 2,
        "AMR_position": {"x": 300, "y": 400},
        "battery_status": {"battery": 90, "remaining_distance": 13.8, "remaining_time": 10.4},
        "current_status": "Loading",
        "AMR_destination_info": {"cell": 1, "row": 1, "column": 1},
        "loaded_item_info": {"cell": 7, "row": 2, "column": 3}
    },
    {
        "AMR_number": 3,
        "AMR_position": {"x": 350, "y": 480},
        "battery_status": {"battery": 90, "remaining_distance": 13.8, "remaining_time": 10.4},
        "current_status": "Charging",
        "AMR_destination_info": "none",
        "loaded_item_info": "none"
    }
]


# Publisher 및 Subscriber에 대한 콜백 함수 정의
def publisher_callback():
    """AMR 데이터를 관제 시스템에 보내는 콜백 함수입니다."""
    for amr_data in amr_data_list:
        # 각 AMR 데이터 항목을 JSON 문자열로 변환
        json_data = json.dumps(amr_data)
        # JSON 데이터를 std_msgs/String 메시지로 발행
        message = roslibpy.Message({'data': json_data})
        publisher.publish(message)
        # print('Sending AMR data:', json_data)


def subscriber_callback(message):
    """수신한 메시지를 JSON 객체로 파싱하고 적절하게 출력합니다."""
    try:
        data = json.loads(message['data'])
        print("Received AMR data:")
        for amr in data:
            print(f"AMR Number: {amr['AMR_number']}, Position: ({amr['AMR_position']['x']}, {amr['AMR_position']['y']})")
            print(f"Battery Status: {amr['battery_status']['battery']}%, Remaining Distance: {amr['battery_status']['remaining_distance']} km")
            print(f"Current Status: {amr['current_status']}")
            print("---")
    except json.JSONDecodeError:
        print("Error decoding JSON from message")

# Publisher 및 Subscriber 생성
publisher = roslibpy.Topic(client, from_ros2, 'std_msgs/String')
subscriber = roslibpy.Topic(client, from_web, 'std_msgs/String')

# Subscriber에 콜백 함수 등록
subscriber.subscribe(subscriber_callback)

try:
    # 클라이언트 연결 유지
    while client.is_connected:
        # 각 AMR 데이터 메시지 보내기
        publisher_callback()
        # 1초 대기
        time.sleep(5)
except KeyboardInterrupt:
    # Ctrl+C로 인터럽트 받으면 종료
    print("Interrupted by user, shutting down.")
finally:
    # 노드 종료
    publisher.unadvertise()
    subscriber.unsubscribe()
    client.terminate()
