# Manipulator Task System 테스트 명령어 정리

## 1. 전체 시스템 실행

### 기본 실행

```bash
ros2 launch manipulator_manager manipulator_task_system.launch.py
```

### 버튼 누르기 노드를 실제 실행하지 않고 계획만 테스트

```bash
ros2 launch manipulator_manager manipulator_task_system.launch.py button_plan_only:=true
```

### MCU 없이 하역 완료를 자동으로 가정하고 테스트

```bash
ros2 launch manipulator_manager manipulator_task_system.launch.py unload_wait_for_result:=false
```

---

## 2. 모니터링 명령어

### Task Manager 상태 확인

```bash
ros2 topic echo /manipulator_task_state
```

### Task Manager 최종 결과 확인

```bash
ros2 topic echo /manipulator_task_result
```

### Task Manager 에러 확인

```bash
ros2 topic echo /manipulator_task_error
```

### Arm Pose Commander 완료 결과 확인

```bash
ros2 topic echo /arm_pose_commander/done
```

### Button Press Commander 결과 확인

```bash
ros2 topic echo /marker_button_press_commander/result
```

### Button Press Commander 상태 확인

```bash
ros2 topic echo /marker_button_press_commander/state
```

---

## 3. 상위 Task Manager 테스트 명령어

Task Manager는 `/manipulator_task_cmd` 토픽으로 명령을 받는다.

### 외부 엘리베이터 승차 버튼 누르기 시퀀스

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'OUTSIDE_BTN_FRONT'}"
```

동작 흐름:

```text
OUTSIDE_BTN_FRONT
→ outside_scan 자세 이동
→ 버튼 Marker 대기
→ 버튼 누르기
→ home 복귀
→ OUTSIDE_BTN_DONE 발행
```

### 내부 엘리베이터 층수 버튼 누르기 시퀀스

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_BTN_FRONT'}"
```

동작 흐름:

```text
INSIDE_BTN_FRONT
→ inside_scan 자세 이동
→ 버튼 Marker 대기
→ 버튼 누르기
→ home 복귀
→ INSIDE_BTN_DONE 발행
```

### 목적지 도착 후 하역 시퀀스

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'DESTINATION_UNLOAD'}"
```

동작 흐름:

```text
DESTINATION_UNLOAD
→ MCU에 UNLOAD 명령 전송
→ MCU 완료 응답 대기 또는 시간 기반 완료 처리
→ home 복귀
→ UNLOAD_DONE 발행
```

### 현재 상태 확인

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'STATUS'}"
```

### 현재 작업 취소

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'CANCEL'}"
```

---

## 4. Arm Pose Commander 단독 테스트

Arm Pose Commander는 `/arm_pose_commander/flag` 토픽으로 명령을 받는다.

### 외부 버튼 인식 자세로 이동

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'outside_scan'}"
```

### 내부 버튼 인식 자세로 이동

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'inside_scan'}"
```

### 기본 자세로 이동

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'home'}"
```

### 상태 확인

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'status'}"
```

### 동작 취소

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'cancel'}"
```

---

## 5. Button Press Commander 단독 테스트

Button Press Commander는 `/marker_button_press_commander/cmd` 토픽으로 명령을 받는다.

### 상태 확인

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'status'}"
```

### 마커 캐시 초기화

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'clear'}"
```

### 마커 위치로 단순 이동 테스트

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'go'}"
```

### 버튼 누르기 실행

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'press'}"
```

### 버튼 누르기 취소

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'cancel'}"
```

---

## 6. 가짜 Marker 발행 테스트

인지 노드 없이 Button Press Commander를 테스트할 때 사용한다.

### link1 기준 가짜 버튼 Marker 발행

```bash
ros2 topic pub --once /object_3d_marker visualization_msgs/msg/Marker "{header: {frame_id: 'link1'}, ns: 'test_button', id: 1, type: 2, action: 0, pose: {position: {x: 0.20, y: 0.00, z: 0.20}, orientation: {w: 1.0}}, scale: {x: 0.02, y: 0.02, z: 0.02}, color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}}"
```

### 가짜 Marker 수신 상태 확인

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'status'}"
```

### 가짜 Marker 기준 버튼 누르기 실행

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'press'}"
```

주의:

```text
x: 0.20, y: 0.00, z: 0.20 값은 예시이다.
실제 로봇팔 작업공간과 충돌 가능성을 확인한 뒤 수정해야 한다.
```

---

## 7. MCU 하역 테스트

Task Manager가 MCU에 명령을 보내는 기본 토픽은 `/mcu/cmd`이다.

### MCU에 하역 명령 직접 발행

```bash
ros2 topic pub --once /mcu/cmd std_msgs/msg/String "{data: 'UNLOAD'}"
```

### MCU 결과 확인

```bash
ros2 topic echo /mcu/result
```

### MCU 완료 응답을 수동으로 흉내내기

MCU bridge가 아직 없을 때 테스트용으로 사용한다.

```bash
ros2 topic pub --once /mcu/result std_msgs/msg/String "{data: 'UNLOAD_DONE'}"
```

---

## 8. 빠른 테스트 순서

### 1단계: 전체 launch 실행

```bash
ros2 launch manipulator_manager manipulator_task_system.launch.py button_plan_only:=true unload_wait_for_result:=false
```

### 2단계: 상태 모니터링

새 터미널에서 실행한다.

```bash
ros2 topic echo /manipulator_task_state
```

새 터미널에서 결과를 확인한다.

```bash
ros2 topic echo /manipulator_task_result
```

### 3단계: Arm Pose 단독 home 테스트

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'home'}"
```

### 4단계: 외부 버튼 인식 자세 테스트

```bash
ros2 topic pub --once /arm_pose_commander/flag std_msgs/msg/String "{data: 'outside_scan'}"
```

### 5단계: 가짜 Marker 발행

```bash
ros2 topic pub --once /object_3d_marker visualization_msgs/msg/Marker "{header: {frame_id: 'link1'}, ns: 'test_button', id: 1, type: 2, action: 0, pose: {position: {x: 0.20, y: 0.00, z: 0.20}, orientation: {w: 1.0}}, scale: {x: 0.02, y: 0.02, z: 0.02}, color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}}"
```

### 6단계: 버튼 누르기 노드 상태 확인

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'status'}"
```

### 7단계: 버튼 누르기 단독 테스트

```bash
ros2 topic pub --once /marker_button_press_commander/cmd std_msgs/msg/String "{data: 'press'}"
```

### 8단계: 외부 버튼 전체 시퀀스 테스트

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'OUTSIDE_BTN_FRONT'}"
```

### 9단계: 내부 버튼 전체 시퀀스 테스트

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_BTN_FRONT'}"
```

### 10단계: 하역 전체 시퀀스 테스트

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'DESTINATION_UNLOAD'}"
```

---

## 9. 실제 실행 전 안전 설정

처음 테스트할 때는 실제 버튼 누르기를 바로 실행하지 않는다.

### launch에서 plan only 사용

```bash
ros2 launch manipulator_manager manipulator_task_system.launch.py button_plan_only:=true
```

### 실제 실행 전 YAML에서 press depth 작게 설정

```yaml
press_depth_m: 0.003
press_velocity_scaling: 0.03
press_acceleration_scaling: 0.03
```

### 안정화 후 점진적으로 증가

```yaml
press_depth_m: 0.005
```

```yaml
press_depth_m: 0.006
```

---

## 10. 토픽 구조 요약

```text
AMR 또는 사용자
  → /manipulator_task_cmd
  → manipulator_task_manager

manipulator_task_manager
  → /arm_pose_commander/flag
  ← /arm_pose_commander/done

manipulator_task_manager
  → /marker_button_press_commander/cmd
  ← /marker_button_press_commander/result

manipulator_task_manager
  → /mcu/cmd
  ← /mcu/result

manipulator_task_manager
  → /manipulator_task_result
  → /manipulator_task_state
  → /manipulator_task_error
```
