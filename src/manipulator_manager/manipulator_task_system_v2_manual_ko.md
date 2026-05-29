# Manipulator Task System v2 매뉴얼

## 1. v2 목적

v2는 기본 설정에서는 버튼 바로 앞 정렬까지만 수행한다.
`enable_press: true`로 켜면 같은 방사 벡터를 재사용해서 짧은 누르기와 release까지 수행한다.

```text
기존 v1:
scan 자세 이동 -> marker 획득 -> EE를 marker+offset으로 바로 이동

v2:
scan 자세 이동 -> 3초 안정화 -> marker 재수집 -> profile 기반 pre-contact 위치로 이동
press enabled:
scan 자세 이동 -> 3초 안정화 -> marker 재수집 -> pre-contact 위치로 이동 -> 저장된 방사 벡터로 press -> release
```

즉 v2의 1차 목표는 EE가 버튼 바로 앞 위치까지 안정적으로 도달하는지 검증하는 것이다. press는 이 검증 이후 `enable_press`를 켜서 추가한다.

---

## 2. v2 구성

v2에서 새로 사용하는 노드는 다음과 같다.

```text
arm_pose_commander_v2
  /arm_pose_commander_v2/flag
  /arm_pose_commander_v2/done

marker_prepress_commander_v2
  /marker_prepress_commander_v2/cmd
  /marker_prepress_commander_v2/result
  /marker_prepress_commander_v2/state

manipulator_task_manager_v2
  /manipulator_task_cmd
  /manipulator_task_result
  /manipulator_task_state
  /manipulator_task_error
```

외부 명령 토픽은 기존과 동일하게 `/manipulator_task_cmd`를 사용한다.

---

## 3. 빌드

워크스페이스 루트에서 실행한다.

```bash
cd ~/capstone_final_ws
colcon build --packages-select manipulator_manager all_in_one_package --symlink-install
source install/setup.bash
```

설치된 v2 파일 확인:

```bash
ls install/manipulator_manager/share/manipulator_manager/config | grep _v2
ls install/manipulator_manager/share/manipulator_manager/launch | grep _v2
```

예상:

```text
arm_pose_commander_v2.yaml
marker_prepress_commander_v2.yaml
manipulator_task_manager_v2.yaml
manipulator_task_system_v2.launch.py
```

---

## 4. 전체 실행

실제 사용 기준 launch:

```bash
cd ~/capstone_final_ws
source install/setup.bash

ros2 launch all_in_one_package manipulator_all_in_one_v2.launch.py
```

처음에는 plan only로 prepress 경로만 확인한다.

```bash
ros2 launch all_in_one_package manipulator_all_in_one_v2.launch.py prepress_plan_only:=true unload_wait_for_result:=false
```

실제 로봇팔 이동:

```bash
ros2 launch all_in_one_package manipulator_all_in_one_v2.launch.py prepress_plan_only:=false unload_wait_for_result:=false
```

`manipulator_all_in_one_v2.launch.py` 시작 순서:

```text
0초  : MoveIt core
3초  : manipulator_task_system_v2
5초  : camera perception
55초 : AMR navigator
```

---

## 5. Manipulator v2만 실행

MoveIt과 perception을 따로 띄운 상태에서 v2 manipulator task system만 실행할 수도 있다.

```bash
ros2 launch manipulator_manager manipulator_task_system_v2.launch.py prepress_plan_only:=true unload_wait_for_result:=false
```

실제 이동:

```bash
ros2 launch manipulator_manager manipulator_task_system_v2.launch.py prepress_plan_only:=false unload_wait_for_result:=false
```

---

## 6. 상태 모니터링

새 터미널에서:

```bash
cd ~/capstone_final_ws
source install/setup.bash
```

Task Manager 상태:

```bash
ros2 topic echo /manipulator_task_state
```

AMR로 나가는 결과:

```bash
ros2 topic echo /manipulator_task_result
```

실패 원인:

```bash
ros2 topic echo /manipulator_task_error
```

Prepress Commander 상태:

```bash
ros2 topic echo /marker_prepress_commander_v2/state
```

Prepress Commander 결과:

```bash
ros2 topic echo /marker_prepress_commander_v2/result
```

Arm Pose Commander 결과:

```bash
ros2 topic echo /arm_pose_commander_v2/done
```

Marker 확인:

```bash
ros2 topic echo /object_3d_marker
```

---

## 7. 전체 작업 명령

### 외부 버튼 전면 접근

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'OUTSIDE_BTN_FRONT'}"
```

흐름:

```text
OUTSIDE_BTN_FRONT
-> perception target OUTSIDE_DOWN
-> outside_scan 자세
-> marker settle
-> prepress profile outside_front
-> OUTSIDE_BTN_DONE
-> home 복귀는 내부적으로 계속 수행
```

### 내부 B1 전면 접근

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_B1_BTN_FRONT'}"
```

결과:

```text
INSIDE_B1_BTN_DONE
```

기존 호환 명령:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_BTN_FRONT'}"
```

결과:

```text
INSIDE_B1_BTN_DONE
```

### 내부 3층 전면 접근

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_3F_BTN_FRONT'}"
```

결과:

```text
INSIDE_3F_BTN_DONE
```

### 내부 B1 오른쪽 접근

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_B1_BTN_RIGHT'}"
```

결과:

```text
INSIDE_B1_BTN_DONE
```

### 내부 3층 오른쪽 접근

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_3F_BTN_RIGHT'}"
```

결과:

```text
INSIDE_3F_BTN_DONE
```

### 목적지 하역

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'DESTINATION_UNLOAD'}"
```

MCU 응답 없이 테스트할 때는 launch에 다음 옵션을 둔다.

```bash
unload_wait_for_result:=false
```

---

## 8. 제어 명령

상태 확인:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'STATUS'}"
```

취소:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'CANCEL'}"
```

FSM reset:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'RESET'}"
```

home 복귀:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'HOME'}"
```

---

## 9. Prepress Commander 단독 테스트

Task Manager를 거치지 않고 prepress commander만 직접 테스트할 수 있다.

상태 확인:

```bash
ros2 topic pub --once /marker_prepress_commander_v2/cmd std_msgs/msg/String "{data: 'status'}"
```

marker buffer 초기화:

```bash
ros2 topic pub --once /marker_prepress_commander_v2/cmd std_msgs/msg/String "{data: 'clear'}"
```

외부 전면 profile:

```bash
ros2 topic pub --once /marker_prepress_commander_v2/cmd std_msgs/msg/String "{data: 'outside_front'}"
```

내부 전면 profile:

```bash
ros2 topic pub --once /marker_prepress_commander_v2/cmd std_msgs/msg/String "{data: 'inside_front'}"
```

내부 오른쪽 profile:

```bash
ros2 topic pub --once /marker_prepress_commander_v2/cmd std_msgs/msg/String "{data: 'inside_right'}"
```

취소:

```bash
ros2 topic pub --once /marker_prepress_commander_v2/cmd std_msgs/msg/String "{data: 'cancel'}"
```

---

## 10. v2 Profile 개념

profile은 버튼 접근 방향과 pre-contact 거리를 정의한다.

기본 방식은 `radial_from_frame`이다. marker에 단순히 camera x/y축 offset을 더하지 않고, marker를 `link1` 기준으로 변환한 뒤 로봇 기준 방사 방향으로 `standoff_m`만큼 앞에 선다. 전면 버튼과 오른쪽 버튼 모두 로봇의 현재 접근 각도를 반영하기 위한 방식이다.

설정 파일:

```text
src/manipulator_manager/config/marker_prepress_commander_v2.yaml
```

기본 profile:

```text
outside_front
inside_b1_front
inside_b1_right
inside_front
inside_right
```

각 profile은 다음 파라미터를 가진다. B1은 실제 EE가 낮게 찍히는 보정을 분리하기 위해 `inside_b1_front`, `inside_b1_right` profile을 사용한다.

```yaml
inside_right_approach_frame: camera_link
inside_right_approach_axis: "y"
inside_right_approach_sign: -1.0
inside_right_standoff_m: 0.035
inside_right_offset_x: 0.0
inside_right_offset_y: 0.0
inside_right_offset_z: 0.0
inside_right_approach_mode: radial_from_frame
inside_right_radial_frame: link1
inside_right_radial_plane: "xy"
inside_right_radial_min_norm_m: 0.05
inside_right_press_travel_m: 0.045
```

현재 데모 보정:

```yaml
outside_front_offset_z: 0.010
inside_b1_front_offset_z: 0.010
inside_b1_right_offset_z: 0.010
```

위 세 값은 실제 타점이 인식 위치보다 아래로 내려가는 오차를 보정하기 위해 목표점을 1cm 위로 올린다.

의미:

```text
approach_frame     : marker와 offset을 먼저 해석할 frame
approach_axis      : approach_mode가 axis일 때만 사용하는 fallback 축
approach_sign      : standoff 방향. -1이면 marker에서 로봇 쪽으로 물러남
standoff_m         : 버튼 marker에서 얼마나 떨어진 pre-contact 위치를 잡을지
offset_*           : marker 중심과 실제 목표점 사이의 미세 보정
approach_mode      : 기본 radial_from_frame
radial_frame       : 방사 방향을 계산할 frame. 기본 link1
radial_plane       : 방사 방향 계산 평면. 기본 xy
radial_min_norm_m  : marker가 기준점에 너무 가까울 때 계산을 막는 최소 거리
press_travel_m     : enable_press가 true일 때 pre-contact에서 버튼 방향으로 더 전진할 거리
```

주의:

```text
YAML에서 x/y/z는 반드시 따옴표를 붙인다.
예: inside_right_approach_axis: "y"
```

따옴표가 없으면 YAML이 `y`를 bool `True`로 해석할 수 있다.

---

## 11. 안정화 및 Marker 수집 방식

v2는 scan 자세에 도달하자마자 받은 marker를 바로 쓰지 않는다.

Task Manager가 먼저 안정화 시간을 둔다.

```yaml
marker_settle_sec: 3.0
```

이 3초 동안 로봇팔 진동과 카메라 시야 흔들림이 줄어들기를 기다린다. 그 다음 `marker_prepress_commander_v2`가 새로 들어온 marker만 모아서 median을 계산한다.

기본 설정:

```yaml
marker_collect_sec: 0.7
preferred_marker_samples: 5
min_marker_samples: 1
marker_timeout_sec: 3.0
allow_recent_marker: false
```

동작:

```text
scan 자세 도달
3초 안정화
prepress 명령 시작 시 기존 marker buffer 비움
0.7초 동안 새로 들어온 marker를 수집
5개 이상이면 최근 marker들의 median 사용
1~4개만 있으면 가진 데이터 안에서 median 사용
0개면 prepress_failed:no_recent_marker
```

YOLO 인식률이 낮아도 최소 1개 marker가 있으면 진행한다.

---

## 12. 튜닝 순서

처음에는 반드시 `prepress_plan_only:=true`로 시작한다.

### 1단계: marker 확인

```bash
ros2 topic echo /object_3d_marker
```

marker frame이 보통 `camera_link`로 나와야 한다.

### 2단계: profile별 plan only

```bash
ros2 launch all_in_one_package manipulator_all_in_one_v2.launch.py prepress_plan_only:=true unload_wait_for_result:=false
```

외부 전면:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'OUTSIDE_BTN_FRONT'}"
```

내부 오른쪽:

```bash
ros2 topic pub --once /manipulator_task_cmd std_msgs/msg/String "{data: 'INSIDE_B1_BTN_RIGHT'}"
```

### 3단계: standoff 튜닝

EE가 버튼에서 너무 멀면 `*_standoff_m`을 줄인다.

```yaml
inside_front_standoff_m: 0.030
```

EE가 버튼에 너무 가까우면 `*_standoff_m`을 늘린다.

```yaml
inside_front_standoff_m: 0.045
```

### 4단계: 접근 방향 튜닝

기본은 `link1` 기준 xy 평면 방사 방향이다.

```yaml
inside_right_approach_mode: radial_from_frame
inside_right_radial_frame: link1
inside_right_radial_plane: "xy"
```

EE가 버튼의 반대 방향으로 멀어지면 `approach_sign`을 반대로 바꾼다.

```yaml
inside_right_approach_sign: 1.0
```

방사 방향 대신 고정축 테스트가 필요할 때만 `approach_mode`를 `axis`로 바꾼다.

```yaml
inside_right_approach_mode: axis
inside_right_approach_axis: "x"
```

### 5단계: offset 튜닝

EE가 버튼 중심보다 위/아래/좌/우로 벗어나면 offset을 조정한다.

```yaml
inside_front_offset_x: 0.000
inside_front_offset_y: 0.005
inside_front_offset_z: -0.003
```

offset은 한 번에 크게 바꾸지 말고 2~5mm 단위로 조정한다.

### 6단계: press 활성화

pre-contact 위치가 안정적으로 맞은 뒤에만 press를 켠다.

```yaml
enable_press: true
press_hold_sec: 0.2
release_after_press: true
inside_right_press_travel_m: 0.045
```

press 단계에서는 marker를 다시 수집하지 않는다. pre-contact 목표를 만들 때 저장한 `link1` 기준 방사 unit vector를 그대로 사용한다.

```text
pre-contact 위치
-> 저장된 press vector 방향으로 press_travel_m 전진
-> press_hold_sec 동안 유지
-> pre-contact 위치로 release
```

버튼까지 닿지 않으면 `*_press_travel_m`을 2~3mm씩 늘리고, 너무 깊게 누르면 줄인다.

---

## 13. 예상 결과 문자열

성공:

```text
OUTSIDE_BTN_DONE
INSIDE_B1_BTN_DONE
INSIDE_3F_BTN_DONE
UNLOAD_DONE
HOME_DONE
```

데모 모드에서는 prepress/press가 실패해도 에러를 기록한 뒤 버튼 task는 위 DONE 문자열로 즉시 완료 처리한다. home 복귀는 내부적으로 계속 수행된다. 이 동작은 `complete_button_on_prepress_failure`로 제어한다.

실패:

```text
FAILED:...
```

Prepress commander 직접 결과:

```text
prepress_done:outside_front
prepress_done:inside_front
prepress_done:inside_right
press_done:outside_front
press_done:inside_front
press_done:inside_right
prepress_failed:no_recent_marker
prepress_failed:target_transform_failed
prepress_failed:goal_rejected
prepress_failed:execution_failed:...
```

---

## 14. 문제 상황별 확인

### marker_prepress_commander_v2가 바로 죽는 경우

`approach_axis` 또는 `radial_plane`이 따옴표로 감싸져 있는지 확인한다.

```yaml
inside_right_approach_axis: "y"
inside_right_radial_plane: "xy"
```

### prepress_failed:no_recent_marker

확인:

```bash
ros2 topic echo /object_3d_marker
ros2 topic echo /manipulator_perception/target_button
```

YOLO target이 맞는지 확인한다.

```bash
ros2 topic pub --once /manipulator_perception/target_button std_msgs/msg/String "{data: 'OUTSIDE_DOWN'}"
ros2 topic pub --once /manipulator_perception/target_button std_msgs/msg/String "{data: 'INSIDE_B1'}"
ros2 topic pub --once /manipulator_perception/target_button std_msgs/msg/String "{data: 'INSIDE_3F'}"
```

### target_transform_failed

TF 확인:

```bash
ros2 run tf2_ros tf2_echo link1 camera_link
```

marker frame도 확인한다.

```bash
ros2 topic echo /object_3d_marker --once
```

### MoveIt goal rejected / execution failed

처음에는 position tolerance를 조금 키운다.

```yaml
position_tolerance_m: 0.012
```

속도도 낮춘다.

```yaml
max_velocity_scaling: 0.10
max_acceleration_scaling: 0.10
```

4자유도 로봇팔이므로 orientation을 강하게 잡으면 안 된다. 기본 tolerance `3.14`는 사실상 orientation을 느슨하게 두기 위한 값이다.

---

## 15. v1과 v2 차이

```text
v1 marker_button_press_commander:
  marker 위치로 EE를 바로 이동
  결과: OUTSIDE_BTN_DONE / INSIDE_*_DONE

v2 marker_prepress_commander_v2:
  3초 안정화 이후 marker를 새로 수집
  marker 위치에서 link1 방사 방향 standoff를 적용한 pre-contact 위치로 이동
  enable_press가 true이면 저장된 같은 방사 벡터로 press/release 수행
  Task Manager 결과: OUTSIDE_BTN_DONE / INSIDE_B1_BTN_DONE / INSIDE_3F_BTN_DONE
```
marker_prepress_commander_v2의 내부 결과는 `prepress_done:*` 또는 `press_done:*`이지만, 외부 `/manipulator_task_result`는 데모 흐름을 위해 `*_BTN_DONE`으로 정규화된다.
