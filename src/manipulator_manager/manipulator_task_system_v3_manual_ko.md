# Manipulator Task System v3 Student 매뉴얼

## 1. 목적

이 문서는 `indoor_students_manager.py`와
`manipulator_task_manager_v3_student.py` 사이의 통신 계약을 기준으로
v3 student 시스템을 실행하고 검증하는 절차를 정리한다.

핵심 검증 대상은 다음 네 가지다.

```text
1. AMR -> manipulator 명령 문자열이 task manager에서 정상 인식되는가
2. manipulator state가 AMR의 active-state gate와 맞는가
3. manipulator result가 AMR의 expected_result와 정확히 같은가
4. 하역 완료가 ACK가 아니라 /mcu/result UNLOAD_DONE으로 처리되는가
```

v3 student에서 AMR이 실제로 사용하는 명령은 다음 두 개다.

```text
INSIDE_BTN_FRONT     -> INSIDE_BTN_DONE
DESTINATION_UNLOAD  -> UNLOAD_DONE
```

수동 테스트 호환을 위해 task manager는 다음 alias도 받는다.

```text
INSIDE_B1_BTN_FRONT  -> INSIDE_BTN_DONE
```

`INSIDE_B1_BTN_FRONT`는 B1 버튼 동작을 수행하지만, v3 student에서는 AMR 계약을
단순하게 유지하기 위해 결과를 `INSIDE_BTN_DONE`으로 반환한다.

---

## 2. 대상 파일

주요 파일:

```text
all_in_one_package/launch/manipulator_all_in_one_v3_student.launch.py
amr_navigator/amr_navigator/indoor_students_manager.py
manipulator_manager/launch/manipulator_task_system_v3_student.launch.py
manipulator_manager/config/manipulator_task_manager_v3_student.yaml
manipulator_manager/manipulator_manager/manipulator_task_manager_v3_student.py
manipulator_hardware/src/manipulator_hardware_interface.cpp
```

통신 토픽:

```text
/manipulator_task_cmd     std_msgs/msg/String
/manipulator_task_state   std_msgs/msg/String
/manipulator_task_result  std_msgs/msg/String
/manipulator_task_error   std_msgs/msg/String

/manipulator_hardware/cmd_pos_flag  std_msgs/msg/Int32
/mcu/result                         std_msgs/msg/String
```

---

## 3. 빌드와 재시작 원칙

Python 노드는 실행 중에 코드가 자동 갱신되지 않는다.
코드를 수정한 뒤에는 반드시 기존 launch를 종료하고 다시 실행한다.

```bash
cd ~/capstone_final_ws
colcon build --packages-select manipulator_manager all_in_one_package amr_navigator manipulator_hardware
source install/setup.bash
```

기존 launch를 `Ctrl+C`로 완전히 종료한 뒤 새 터미널에서 다시 실행한다.

정상 설치본 확인:

```bash
grep -n "INSIDE_B1_BTN_FRONT" \
  ~/capstone_final_ws/install/manipulator_manager/lib/python3.10/site-packages/manipulator_manager/manipulator_task_manager_v3_student.py
```

출력이 없으면 아직 수정된 설치본을 실행하고 있지 않다.

---

## 4. 전체 실행

AMR, MoveIt, perception, manipulator task system을 모두 포함해서 실행한다.

```bash
cd ~/capstone_final_ws
source install/setup.bash

ros2 launch all_in_one_package manipulator_all_in_one_v3_student.launch.py
```

기본 launch 계약:

```text
prepress_plan_only=false
unload_wait_for_result=true
amr_manipulator_task_timeout_sec=90.0
amr_manipulator_cmd_publish_count=3
amr_manipulator_cmd_republish_interval_sec=1.0
amr_require_manipulator_active_state_before_result=true
```

plan only로 버튼 접근 경로만 먼저 확인하려면:

```bash
ros2 launch all_in_one_package manipulator_all_in_one_v3_student.launch.py \
  prepress_plan_only:=true \
  unload_wait_for_result:=true
```

주의:

```text
unload_wait_for_result=true이면 DESTINATION_UNLOAD는 /mcu/result의 UNLOAD_DONE을 기다린다.
ESP32나 hardware bridge 없이 이 옵션을 켜면 수동으로 /mcu/result를 publish하지 않는 한 완료되지 않는다.
```

---

## 5. Manipulator v3만 실행

MoveIt과 perception을 별도로 띄운 상태에서 manipulator task system만 실행할 수 있다.

```bash
cd ~/capstone_final_ws
source install/setup.bash

ros2 launch manipulator_manager manipulator_task_system_v3_student.launch.py \
  prepress_plan_only:=true \
  unload_wait_for_result:=true
```

실제 로봇팔 이동:

```bash
ros2 launch manipulator_manager manipulator_task_system_v3_student.launch.py \
  prepress_plan_only:=false \
  unload_wait_for_result:=true
```

하드웨어 없이 시간 기반 fallback 완료 경로만 빠르게 확인할 때만 다음 옵션을 사용한다.

```bash
ros2 launch manipulator_manager manipulator_task_system_v3_student.launch.py \
  prepress_plan_only:=true \
  unload_wait_for_result:=false
```

`unload_wait_for_result=false`는 실제 운용 기본값이 아니다.
실제 ESP32 하역 완료 검증은 반드시 `true`로 한다.
하드웨어 없이도 실제 운용 계약을 검증하려면 12장의 `/mcu/result` 수동 publish 테스트를 사용한다.

---

## 6. 상태 모니터링 터미널

새 터미널에서:

```bash
cd ~/capstone_final_ws
source install/setup.bash
```

Task manager 상태:

```bash
ros2 topic echo /manipulator_task_state
```

AMR로 반환되는 결과:

```bash
ros2 topic echo /manipulator_task_result
```

실패 원인:

```bash
ros2 topic echo /manipulator_task_error
```

Arm pose commander 명령과 완료:

```bash
ros2 topic echo /arm_pose_commander_v2/flag
ros2 topic echo /arm_pose_commander_v2/done
```

Prepress commander 명령, 상태, 결과:

```bash
ros2 topic echo /marker_prepress_commander_v2/cmd
ros2 topic echo /marker_prepress_commander_v2/state
ros2 topic echo /marker_prepress_commander_v2/result
```

Perception target:

```bash
ros2 topic echo /manipulator_perception/target_button
```

하역 flag와 MCU 완료:

```bash
ros2 topic echo /manipulator_hardware/cmd_pos_flag
ros2 topic echo /mcu/result
```

노드 파라미터 확인:

```bash
ros2 param get /manipulator_task_manager_v3_student unload_wait_for_result
ros2 param get /manipulator_task_manager_v3_student mcu_unload_done
ros2 param get /manipulator_task_manager_v3_student unload_timeout_sec
```

`ros2 param get`의 출력 문구는 ROS 2 배포판별로 조금 다를 수 있다.
값 자체는 다음이어야 한다.

```text
unload_wait_for_result: true
mcu_unload_done: UNLOAD_DONE
unload_timeout_sec: 45.0
```

---

## 7. AMR와 Manipulator 사이 계약

`indoor_students_manager.py`는 waypoint에 도착한 뒤 다음 handshake를 수행한다.

### 7.1 내부 버튼 waypoint

```text
AMR publish:
  /manipulator_task_cmd = INSIDE_BTN_FRONT

AMR waits for active state:
  INSIDE_ALIGNING
  INSIDE_MARKER_SETTLE
  BUTTON_PREPRESSING
  BUTTON_HOMING

AMR accepts result only if:
  /manipulator_task_result = INSIDE_BTN_DONE
  result timestamp >= handshake start time
  active state was observed after handshake start
```

### 7.2 하역 waypoint

```text
AMR publish:
  /manipulator_task_cmd = DESTINATION_UNLOAD

AMR waits for active state:
  UNLOAD_PREPARE
  UNLOAD_EXECUTE

AMR accepts result only if:
  /manipulator_task_result = UNLOAD_DONE
  result timestamp >= handshake start time
  active state was observed after handshake start
```

### 7.3 재발행 규칙

AMR은 명령을 한 번 publish한 뒤 active state가 관측되지 않으면
`amr_manipulator_cmd_republish_interval_sec` 간격으로 다시 publish한다.

기본값:

```text
최대 publish 횟수: 3
재발행 간격: 1.0초
```

active state가 한 번이라도 관측되면 같은 명령을 더 이상 재발행하지 않는다.
이는 로봇팔 동작 중 중복 명령으로 FSM이 흔들리는 것을 막기 위한 것이다.

---

## 8. Task Manager FSM 상세 시퀀스

### 8.1 INSIDE_BTN_FRONT

입력:

```text
/manipulator_task_cmd = INSIDE_BTN_FRONT
```

FSM:

```text
IDLE
-> publish /manipulator_perception/target_button = INSIDE_B1
-> publish /marker_prepress_commander_v2/cmd = clear
-> state INSIDE_ALIGNING
-> publish /arm_pose_commander_v2/flag = inside_scan
-> wait /arm_pose_commander_v2/done = inside_scan_done
-> state INSIDE_MARKER_SETTLE
-> wait marker_settle_sec
-> state BUTTON_PREPRESSING
-> publish /marker_prepress_commander_v2/cmd = inside_b1_front
-> wait /marker_prepress_commander_v2/result = prepress_done... or press_done...
-> state BUTTON_HOMING
-> publish /arm_pose_commander_v2/flag = home
-> wait /arm_pose_commander_v2/done = home_done
-> state IDLE
-> publish /manipulator_task_result = INSIDE_BTN_DONE
```

수동 alias:

```text
/manipulator_task_cmd = INSIDE_B1_BTN_FRONT
```

동작은 동일하고 결과도 `INSIDE_BTN_DONE`이다.

### 8.2 DESTINATION_UNLOAD

입력:

```text
/manipulator_task_cmd = DESTINATION_UNLOAD
```

FSM:

```text
IDLE
-> state UNLOAD_PREPARE
-> publish /manipulator_hardware/cmd_pos_flag = 3
-> wait unload_step_delay_sec
-> state UNLOAD_EXECUTE
-> publish /manipulator_hardware/cmd_pos_flag = 2
-> wait /mcu/result = UNLOAD_DONE
-> state IDLE
-> publish /manipulator_task_result = UNLOAD_DONE
```

중요:

```text
ESP32의 ACK,<seq>는 하역 완료가 아니다.
하역 완료는 펌웨어의 UNLOADING COMPLETE가 hardware interface에서
/mcu/result = UNLOAD_DONE으로 변환된 뒤에만 인정한다.
```

---

## 9. 수동 테스트 1: 실행 코드 확인

launch 직후 로그에서 다음 줄이 보여야 한다.

```text
[task_manager_v3_student] supported tasks: INSIDE_BTN_FRONT, INSIDE_B1_BTN_FRONT, DESTINATION_UNLOAD
```

만약 다음처럼 보이면 예전 코드가 실행 중이다.

```text
[task_manager_v3_student] supported tasks: INSIDE_BTN_FRONT, DESTINATION_UNLOAD
```

이 경우:

```bash
cd ~/capstone_final_ws
colcon build --packages-select manipulator_manager
source install/setup.bash
```

그리고 기존 launch를 완전히 종료한 뒤 다시 실행한다.

---

## 10. 수동 테스트 2: STATUS

명령:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'STATUS'}"
```

예상 `/manipulator_task_result`:

```text
STATUS_V3_STUDENT:STATE=IDLE,...
UNLOAD_WAIT_FOR_RESULT=True,...
MCU_RESULT_TOPIC=/mcu/result,...
MCU_UNLOAD_DONE=UNLOAD_DONE,...
```

이 테스트는 task manager가 `/manipulator_task_cmd`를 받고 있는지 확인하는 가장
안전한 테스트다.

---

## 11. 수동 테스트 3: 내부 버튼 명령

AMR과 같은 명령:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_BTN_FRONT'}"
```

예상 state:

```text
INSIDE_ALIGNING
INSIDE_MARKER_SETTLE
BUTTON_PREPRESSING
BUTTON_HOMING
IDLE
```

예상 result:

```text
INSIDE_BTN_DONE
```

수동 호환 alias:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_B1_BTN_FRONT'}"
```

예상 result:

```text
INSIDE_BTN_DONE
```

다음 에러가 나오면 수정 전 코드나 다른 workspace를 실행 중인 것이다.

```text
UNKNOWN_COMMAND:INSIDE_B1_BTN_FRONT
```

---

## 12. 수동 테스트 4: 하드웨어 없이 하역 FSM 확인

이 테스트는 ESP32 없이 task manager와 AMR 계약만 확인한다.
`unload_wait_for_result=true` 상태에서 수행한다.

1번 터미널: task manager 실행

```bash
ros2 launch manipulator_manager manipulator_task_system_v3_student.launch.py \
  prepress_plan_only:=true \
  unload_wait_for_result:=true
```

2번 터미널: 상태 모니터링

아래 echo 명령은 각각 계속 실행되는 명령이다.
동시에 보려면 별도 터미널에서 하나씩 실행한다.

```bash
ros2 topic echo /manipulator_task_state
ros2 topic echo /manipulator_hardware/cmd_pos_flag
ros2 topic echo /manipulator_task_result
```

3번 터미널: 하역 명령

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'DESTINATION_UNLOAD'}"
```

예상 state와 flag:

```text
state: UNLOAD_PREPARE
cmd_pos_flag: 3
state: UNLOAD_EXECUTE
cmd_pos_flag: 2
```

이 시점에서는 아직 완료되면 안 된다.
이제 MCU 완료를 수동 시뮬레이션한다.

```bash
ros2 topic pub --once /mcu/result std_msgs/msg/String "{data: 'UNLOAD_DONE'}"
```

예상 result:

```text
UNLOAD_DONE
```

합격 기준:

```text
/mcu/result를 publish하기 전에는 /manipulator_task_result에 UNLOAD_DONE이 나오지 않는다.
/mcu/result = UNLOAD_DONE 이후에만 /manipulator_task_result = UNLOAD_DONE이 나온다.
```

---

## 13. 수동 테스트 5: 실제 ESP32 하역 완료 확인

실제 하드웨어 포함 실행:

```bash
ros2 launch all_in_one_package manipulator_all_in_one_v3_student.launch.py \
  prepress_plan_only:=false \
  unload_wait_for_result:=true
```

하역 명령:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'DESTINATION_UNLOAD'}"
```

정상 로그 흐름:

```text
task_manager:
  state UNLOAD_PREPARE
  cmd_pos_flag=3
  state UNLOAD_EXECUTE
  cmd_pos_flag=2

ESP32 firmware:
  UNLOADING START
  ...
  UNLOADING COMPLETE

manipulator_hardware_interface:
  /mcu/result = UNLOAD_DONE

task_manager:
  mcu_result='UNLOAD_DONE'
  result='UNLOAD_DONE'
```

주의:

```text
ACK,<seq>는 명령 수신 ACK일 뿐이다.
ACK,<seq>를 받았다는 이유로 task_manager가 UNLOAD_DONE을 publish하면 안 된다.
```

---

## 14. 수동 테스트 6: 중복 명령 억제

로봇팔이 동작 중일 때 같은 명령을 다시 보내도 새 작업이 중복 시작되면 안 된다.

예:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'DESTINATION_UNLOAD'}"
sleep 1
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'DESTINATION_UNLOAD'}"
```

예상:

```text
duplicate active task ignored
```

작업 완료 직후 `duplicate_cmd_ignore_window_sec` 안에 같은 명령이 다시 들어오면
로봇팔을 다시 움직이지 않고 마지막 result만 재발행한다.

예상:

```text
recent duplicate completed task ignored
result='UNLOAD_DONE'
```

---

## 15. AMR 통합 테스트

전체 launch를 실행한다.

```bash
ros2 launch all_in_one_package manipulator_all_in_one_v3_student.launch.py
```

AMR 로그에서 시작 시 다음 정보가 보여야 한다.

```text
inside_button=(INSIDE_BTN_FRONT->INSIDE_BTN_DONE)
destination=(DESTINATION_UNLOAD->UNLOAD_DONE)
cmd_publish_count=3
cmd_republish_interval=1.00s
require_active_state=True
manipulator_task_timeout=90.0s
```

내부 버튼 waypoint 도착 시 AMR 로그:

```text
Manipulator handshake start: publish String(data='INSIDE_BTN_FRONT')
Waiting manipulator result: expected='INSIDE_BTN_DONE', ...
Manipulator task done. received='INSIDE_BTN_DONE'
```

하역 waypoint 도착 시 AMR 로그:

```text
Manipulator handshake start: publish String(data='DESTINATION_UNLOAD')
Waiting manipulator result: expected='UNLOAD_DONE', latest_state='UNLOAD_EXECUTE', observed_active_state=True
Manipulator task done. received='UNLOAD_DONE'
```

합격 기준:

```text
AMR은 old result를 받지 않는다.
AMR은 active state를 본 뒤에만 result를 수락한다.
AMR은 manipulator 동작 중 같은 명령을 계속 publish하지 않는다.
하역은 /mcu/result UNLOAD_DONE 이후에만 완료된다.
```

---

## 16. 문제별 진단

### 16.1 UNKNOWN_COMMAND:INSIDE_B1_BTN_FRONT

원인:

```text
수정 전 manipulator_task_manager_v3_student가 실행 중이다.
또는 다른 workspace의 install/setup.bash를 source했다.
```

확인:

```bash
grep -n "INSIDE_B1_BTN_FRONT" \
  ~/capstone_final_ws/install/manipulator_manager/lib/python3.10/site-packages/manipulator_manager/manipulator_task_manager_v3_student.py
```

해결:

```bash
cd ~/capstone_final_ws
colcon build --packages-select manipulator_manager
source install/setup.bash
```

기존 launch 종료 후 재실행한다.

### 16.2 DESTINATION_UNLOAD가 끝나지 않음

확인:

```bash
ros2 topic echo /mcu/result
```

`UNLOAD_DONE`이 안 나오면 task manager는 정상적으로 기다리는 중이다.
ESP32 firmware의 `UNLOADING COMPLETE`가 hardware interface에서
`/mcu/result = UNLOAD_DONE`으로 변환되는지 확인한다.

임시 확인:

```bash
ros2 topic pub --once /mcu/result std_msgs/msg/String "{data: 'UNLOAD_DONE'}"
```

이때 task manager가 완료되면 task manager 쪽은 정상이고,
문제는 ESP32 serial 또는 hardware interface 입력 쪽이다.

### 16.3 UNLOAD_TIMEOUT

확인:

```bash
ros2 param get /manipulator_task_manager_v3_student unload_timeout_sec
```

정상 기본값은 `45.0`이다.
ESP32 하역 시퀀스가 45초보다 오래 걸리면 YAML에서 시간을 늘린다.

### 16.4 AMR이 manipulator result를 받았는데 계속 기다림

가능한 원인:

```text
1. result 문자열이 expected_result와 정확히 다르다.
2. result가 handshake 시작 전에 발행된 오래된 result다.
3. active state가 관측되지 않았다.
```

확인:

```bash
ros2 topic echo /manipulator_task_state
ros2 topic echo /manipulator_task_result
```

AMR 로그에서 확인할 항목:

```text
latest_result
latest_state
observed_active_state
publish_attempts
elapsed
```

### 16.5 Nav2 TF extrapolation 또는 lidar stale warning

이 문서의 통신 수정은 manipulator task result 계약을 바로잡는 것이다.
로봇팔 동작 중 AMR PC가 50초 이상 멈추거나 lidar topic이 갱신되지 않으면
Nav2 TF와 costmap 문제는 별도로 발생할 수 있다.

확인:

```bash
ros2 topic hz /rplidar1/scan_filtered
ros2 topic hz /tf
ros2 topic hz /amcl_pose
```

로봇팔 동작 중 위 topic들이 멈추면 통신 FSM이 아니라 전원, USB, CPU 부하,
노드 block 문제가 원인이다.

---

## 17. 최종 합격 체크리스트

```text
[ ] launch 로그에 INSIDE_B1_BTN_FRONT가 supported tasks로 나온다.
[ ] STATUS 명령에 STATUS_V3_STUDENT result가 나온다.
[ ] INSIDE_BTN_FRONT 명령이 INSIDE_BTN_DONE으로 끝난다.
[ ] INSIDE_B1_BTN_FRONT 명령도 INSIDE_BTN_DONE으로 끝난다.
[ ] DESTINATION_UNLOAD가 cmd_pos_flag 3 -> 2를 publish한다.
[ ] /mcu/result publish 전에는 UNLOAD_DONE이 나오지 않는다.
[ ] /mcu/result = UNLOAD_DONE 이후에만 /manipulator_task_result = UNLOAD_DONE이 나온다.
[ ] AMR 로그에서 active state 관측 후 result를 수락한다.
[ ] 중복 명령이 로봇팔 동작을 중복 실행하지 않는다.
[ ] 실제 하드웨어에서 ESP32의 UNLOADING COMPLETE가 /mcu/result UNLOAD_DONE으로 변환된다.
```
