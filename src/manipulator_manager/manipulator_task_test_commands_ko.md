# Manipulator + Perception 실행 및 테스트 명령어 정리

## 1. 전체 실행 구조

현재 시스템은 크게 두 개의 launch로 나누어 실행한다.

```text
1번 터미널: perception system 실행
2번 터미널: manipulator manager system 실행
3번 터미널: 상태 모니터링
4번 터미널: 명령 발행
```

전체 흐름은 다음과 같다.

```text
AMR 또는 사용자 명령
→ /manipulator_task_cmd
→ manipulator_task_manager
→ perception target 전환
→ arm_pose_commander 자세 이동
→ object_distance_node marker 생성
→ marker_button_press_commander 버튼 누르기
→ AMR 완료 플래그 발행
→ 로봇팔 home 복귀
```

---

## 2. 빌드

워크스페이스 루트에서 실행한다.

```bash
cd ~/capstone_final_ws

colcon build --packages-select camera_perception_pkg manipulator_manager --symlink-install

source install/setup.bash
```

빌드 후 config 파일 설치 여부를 확인한다.

```bash
ls ~/capstone_final_ws/install/manipulator_manager/share/manipulator_manager/config
```

아래 파일들이 보여야 한다.

```text
arm_pose_commander.yaml
manipulator_task_manager.yaml
marker_button_press_commander.yaml
```

perception config도 확인한다.

```bash
ls ~/capstone_final_ws/install/camera_perception_pkg/share/camera_perception_pkg/config
```

---

## 3. 1번 터미널: Perception 실행

```bash
cd ~/capstone_final_ws
source install/setup.bash

ros2 launch camera_perception_pkg manipulator_perception.launch.py
```

이 launch는 다음 노드들을 순서대로 실행한다.

```text
RealSense D435
→ YOLOv8 detection node
→ object_distance_node
→ YOLOv8 debug node
```

---

## 4. 2번 터미널: Manipulator System 실행

처음 테스트는 실제 실행 없이 plan only로 시작한다.

```bash
cd ~/capstone_final_ws
source install/setup.bash

ros2 launch manipulator_manager manipulator_task_system.launch.py button_plan_only:=true unload_wait_for_result:=false
```

실제 로봇팔을 움직일 때는 다음 명령을 사용한다.

```bash
ros2 launch manipulator_manager manipulator_task_system.launch.py button_plan_only:=false unload_wait_for_result:=false
```

MCU 하역 응답까지 실제로 받을 경우에는 다음처럼 실행한다.

```bash
ros2 launch manipulator_manager manipulator_task_system.launch.py button_plan_only:=false unload_wait_for_result:=true
```

---

## 5. 3번 터미널: 상태 모니터링

```bash
cd ~/capstone_final_ws
source install/setup.bash
```

### Task Manager 상태 확인

```bash
ros2 topic echo /manipulator_task_state
```

### AMR로 나가는 완료 플래그 확인

```bash
ros2 topic echo /manipulator_task_result
```

### 실패 원인 확인

```bash
ros2 topic echo /manipulator_task_error
```

### Button Press Commander 결과 확인

```bash
ros2 topic echo /marker_button_press_commander/result
```

### Arm Pose Commander 완료 결과 확인

```bash
ros2 topic echo /arm_pose_commander/done
```

### 3D Marker 확인

```bash
ros2 topic echo /object_3d_marker
```

---

## 6. 4번 터미널: 전체 작업 명령

```bash
cd ~/capstone_final_ws
source install/setup.bash
```

---

## 7. 외부 엘리베이터 아래 버튼 누르기

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'OUTSIDE_BTN_FRONT'}"
```

예상 동작 흐름:

```text
OUTSIDE_BTN_FRONT
→ perception target: OUTSIDE_DOWN
→ object_distance_node target: btn_down
→ outside_scan 자세 이동
→ marker settle 대기
→ 버튼 누르기 시도
→ OUTSIDE_BTN_DONE 즉시 발행
→ 로봇팔 home 복귀
```

---

## 8. 내부 B1 버튼 누르기

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_B1_BTN_FRONT'}"
```

예상 동작 흐름:

```text
INSIDE_B1_BTN_FRONT
→ perception target: INSIDE_B1
→ object_distance_node target: elevator_btn_under1
→ inside_scan 자세 이동
→ marker settle 대기
→ 버튼 누르기 시도
→ INSIDE_B1_BTN_DONE 즉시 발행
→ 로봇팔 home 복귀
```

기존 호환 명령으로 `INSIDE_BTN_FRONT`를 B1 버튼으로 처리하도록 구성한 경우 다음 명령도 사용 가능하다.

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_BTN_FRONT'}"
```

---

## 9. 내부 3층 버튼 누르기

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_3F_BTN_FRONT'}"
```

예상 동작 흐름:

```text
INSIDE_3F_BTN_FRONT
→ perception target: INSIDE_3F
→ object_distance_node target: elevator_btn_3
→ inside_scan 자세 이동
→ marker settle 대기
→ 버튼 누르기 시도
→ INSIDE_3F_BTN_DONE 즉시 발행
→ 로봇팔 home 복귀
```

---

## 10. 목적지 하역

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'DESTINATION_UNLOAD'}"
```

예상 동작 흐름:

```text
DESTINATION_UNLOAD
→ /mcu/cmd 로 UNLOAD 발행
→ /mcu/result 대기 또는 시간 기반 완료 처리
→ 필요 시 home 복귀
→ UNLOAD_DONE 발행
```

MCU bridge가 아직 없으면 launch에서 다음 옵션을 사용한다.

```bash
unload_wait_for_result:=false
```

---

## 11. 상태 확인 및 제어 명령

### 현재 상태 확인

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'STATUS'}"
```

### 현재 작업 취소

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'CANCEL'}"
```

### FSM reset

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'RESET'}"
```

### home 복귀

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'HOME'}"
```

---

## 12. Perception Target 단독 테스트

`object_distance_node`는 `/manipulator_perception/target_button`으로 target을 바꾼다.

### 외부 아래 버튼 target

```bash
ros2 topic pub --once /manipulator_perception/target_button std_msgs/msg/String "{data: 'OUTSIDE_DOWN'}"
```

예상 target prefix:

```text
btn_down
```

---

### 내부 B1 버튼 target

```bash
ros2 topic pub --once /manipulator_perception/target_button std_msgs/msg/String "{data: 'INSIDE_B1'}"
```

예상 target prefix:

```text
elevator_btn_under1
```

---

### 내부 3층 버튼 target

```bash
ros2 topic pub --once /manipulator_perception/target_button std_msgs/msg/String "{data: 'INSIDE_3F'}"
```

예상 target prefix:

```text
elevator_btn_3
```

---

## 13. YOLO class 직접 지정 테스트

preset이 아니라 class name을 직접 넣어서 테스트할 수도 있다.

### 아래 버튼

```bash
ros2 topic pub --once /manipulator_perception/target_button std_msgs/msg/String "{data: 'btn_down'}"
```

### B1 버튼

```bash
ros2 topic pub --once /manipulator_perception/target_button std_msgs/msg/String "{data: 'elevator_btn_under1'}"
```

### 3층 버튼

```bash
ros2 topic pub --once /manipulator_perception/target_button std_msgs/msg/String "{data: 'elevator_btn_3'}"
```

---

## 14. Arm Pose Commander 단독 테스트

### 외부 버튼 인식 자세

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'outside_scan'}"
```

### 내부 버튼 인식 자세

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'inside_scan'}"
```

### home 자세

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'home'}"
```

### 상태 확인

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'status'}"
```

### 취소

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'cancel'}"
```

---

## 15. Marker Button Press Commander 단독 테스트

### 상태 확인

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'status'}"
```

### marker buffer 초기화

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'clear'}"
```

### marker 위치로 단순 이동

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'go'}"
```

### 버튼 누르기 실행

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'press'}"
```

### 취소

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'cancel'}"
```

---

## 16. 실제 실행 전 파라미터 확인

### Marker Button Press Commander 파라미터 확인

```bash
ros2 param get /marker_button_press_commander marker_timeout_sec
ros2 param get /marker_button_press_commander position_tolerance_m
ros2 param get /marker_button_press_commander outside_offset_x
ros2 param get /marker_button_press_commander inside_offset_x
ros2 param get /marker_button_press_commander plan_only
```

처음 테스트 권장값:

```text
marker_timeout_sec: 3.0
position_tolerance_m: 0.005
plan_only: true
```

현재 버튼 누르기는 approach/press/retreat 분할 동작이 아니라 marker 위치에 outside/inside offset을 적용한 단일 MoveIt goal로 실행한다.

---

### Task Manager 파라미터 확인

```bash
ros2 param get /manipulator_task_manager marker_settle_sec
ros2 param get /manipulator_task_manager return_home_after_button
```

권장값:

```text
marker_settle_sec: 3.0
return_home_after_button: true
```

---

## 17. 추천 테스트 순서

처음 테스트할 때는 아래 순서로 진행한다.

### 1단계: Perception 실행

```bash
ros2 launch camera_perception_pkg manipulator_perception.launch.py
```

### 2단계: Manipulator 실행

처음에는 plan only로 실행한다.

```bash
ros2 launch manipulator_manager manipulator_task_system.launch.py button_plan_only:=true unload_wait_for_result:=false
```

### 3단계: 상태 모니터링

```bash
ros2 topic echo /manipulator_task_state
```

다른 터미널에서 결과도 확인한다.

```bash
ros2 topic echo /manipulator_task_result
```

에러도 확인한다.

```bash
ros2 topic echo /manipulator_task_error
```

### 4단계: 외부 아래 버튼 target만 먼저 테스트

```bash
ros2 topic pub --once /manipulator_perception/target_button std_msgs/msg/String "{data: 'OUTSIDE_DOWN'}"
```

### 5단계: marker가 나오는지 확인

```bash
ros2 topic echo /object_3d_marker
```

### 6단계: 외부 버튼 전체 시퀀스

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'OUTSIDE_BTN_FRONT'}"
```

### 7단계: 내부 B1 버튼 전체 시퀀스

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_B1_BTN_FRONT'}"
```

### 8단계: 내부 3층 버튼 전체 시퀀스

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_3F_BTN_FRONT'}"
```

---

## 18. 실제 로봇팔 동작 테스트

plan only에서 경로가 정상적으로 생성되는 것을 확인한 뒤 실제 실행한다.

```bash
ros2 launch manipulator_manager manipulator_task_system.launch.py button_plan_only:=false unload_wait_for_result:=false
```

실제 버튼을 누르기 전에는 plan only에서 목표점과 경로를 먼저 확인한다.

```yaml
plan_only: true
```

실제 누르는 위치는 outside/inside offset으로 조정한다.

```yaml
outside_offset_x: 0.050
inside_offset_x: 0.025
```

```yaml
button_offset_x: 0.000
button_offset_y: 0.000
button_offset_z: 0.000
```

---

## 19. 결과 해석

### 버튼 작업 완료 플래그

```text
OUTSIDE_BTN_DONE
INSIDE_B1_BTN_DONE
INSIDE_3F_BTN_DONE
```

이 플래그들은 실제 버튼 눌림 성공만을 의미하지 않는다.

정확한 의미는 다음과 같다.

```text
로봇팔이 버튼 누르기 동작을 시도했고,
AMR은 다음 동작을 진행해도 된다.
로봇팔은 내부적으로 home 복귀 중이거나 이미 복귀 완료 상태일 수 있다.
```

---

### 실패 원인

실패 원인은 다음 토픽에서 확인한다.

```bash
ros2 topic echo /manipulator_task_error
```

예시:

```text
OUTSIDE_PRESS_FAILED:button_press_failed:no_valid_marker_target
INSIDE_PRESS_FAILED:button_press_failed:stage_failed:APPROACH,...
```

---

## 20. 전체 토픽 구조 요약

```text
AMR 또는 사용자
  → /manipulator_task_cmd

manipulator_task_manager
  → /manipulator_perception/target_button
  → /arm_pose_commander/flag
  ← /arm_pose_commander/done
  → /marker_button_press_commander/cmd
  ← /marker_button_press_commander/result
  → /mcu/cmd
  ← /mcu/result
  → /manipulator_task_result
  → /manipulator_task_state
  → /manipulator_task_error

object_distance_node
  ← /manipulator_perception/target_button
  ← /detections
  ← depth image
  ← camera info
  → /object_3d_marker
  → /object_3d_point
```
