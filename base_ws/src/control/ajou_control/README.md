# ajou_control

`ajou_control`은 MORAI 궤적 추종을 위한 ROS1 Noetic 제어 패키지입니다. 토픽 연결 확인용 가상 기준 경로를 만들 수 있고, `ajou_trajectory_recorder`로 저장한 CSV 궤적을 Pure Pursuit 조향 제어와 종방향 PI 속도 제어로 따라갈 수 있습니다.

이 패키지는 MORAI-ROS의 `morai_msgs` 입출력 규칙을 따릅니다. ROS2나 Autoware Universe 메시지 타입에는 의존하지 않습니다.

## 토픽

| 토픽 | 타입 | 방향 | 설명 |
| --- | --- | --- | --- |
| `/reference_path` | `nav_msgs/Path` | 발행 | `map` 프레임 기준 가상 기준 경로 |
| `/target_speed` | `std_msgs/Float32` | 발행 | 목표 속도, 단위 m/s |
| `/reference_path_marker` | `visualization_msgs/Marker` | 발행 | RViz에서 볼 수 있는 기준 경로 라인 마커 |
| `/recorded_trajectory_path` | `nav_msgs/Path` | 구독 | `ajou_trajectory_recorder`가 CSV에서 읽어 발행한 기록 경로 |
| `/recorded_trajectory/target_speed` | `std_msgs/Float32` | 구독 | 기록 CSV에서 계산한 선택적 목표 속도 |
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` | 발행 | MORAI 차량 제어 명령 토픽 |
| `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | 구독 | MORAI ego 차량 상태 토픽 |

## 빌드

```bash
cd ~/catkin_ws
catkin_make
source ~/.bashrc
```

`.bashrc`에서 workspace를 source하지 않는 환경이면 아래 명령을 직접 실행합니다.

```bash
source ~/catkin_ws/devel/setup.bash
```

## 가상 궤적 검증

```bash
roslaunch ajou_control trajectory_validation.launch
```

원형 경로 예시:

```bash
roslaunch ajou_control trajectory_validation.launch mode:=circle radius:=20.0 target_speed_mps:=5.0
```

## RViz 시각화

```bash
roslaunch ajou_control trajectory_validation.launch enable_rviz:=true
```

RViz 설정:

1. `Fixed Frame`을 `map`으로 설정합니다.
2. `Path`를 추가하고 `/reference_path`를 선택합니다.
3. `Marker`를 추가하고 `/reference_path_marker`를 선택합니다.

## MORAI 명령 토픽 테스트

```bash
rosrun ajou_control simple_morai_cmd.py
rostopic echo /ctrl_cmd
```

기본 명령은 안전 정지 상태입니다.

- `longlCmdType = 1`
- `accel = 0.0`
- `brake = 1.0`
- `front_steer = 0.0`

## 기록한 MORAI 궤적 추종

일반적인 사용 흐름은 다음과 같습니다.

1. `ajou_trajectory_recorder`로 수동 주행 궤적을 기록합니다.
2. 기록한 CSV를 `nav_msgs/Path`로 발행합니다.
3. 이 제어기가 해당 경로를 추종합니다.

이미 CSV가 있으면 아래 launch 하나로 경로 발행, 시각화, 제어기, 선택적 validator를 같이 실행할 수 있습니다.

```bash
roslaunch ajou_control recorded_path_stack.launch target_speed_mps:=3.0
```

추종 검증까지 같이 켜는 경우:

```bash
roslaunch ajou_control recorded_path_stack.launch target_speed_mps:=3.0 enable_validator:=true
```

CSV 경로를 직접 지정하는 경우:

```bash
roslaunch ajou_control recorded_path_stack.launch input_csv:=/absolute/path/to/recorded_trajectory.csv target_speed_mps:=3.0
```

터미널을 나눠 실행하려면 먼저 CSV 궤적을 발행합니다.

```bash
roslaunch ajou_trajectory_recorder publish_trajectory.launch
```

그 다음 Pure Pursuit + PI 제어기를 실행합니다.

```bash
roslaunch ajou_control recorded_path_control.launch
```

튜닝 초기에는 낮은 속도로 시작합니다.

```bash
roslaunch ajou_control recorded_path_control.launch target_speed_mps:=5.0
```

차량이 경로 반대 방향으로 조향하면 조향 부호를 반대로 바꿉니다.

```bash
roslaunch ajou_control recorded_path_control.launch target_speed_mps:=5.0 steering_sign:=-1.0
```

차량은 느리게 보이는데 로그의 raw speed가 크게 나오면 MORAI가 속도를 km/h로 발행하는 상황일 수 있습니다. 이때는 m/s로 변환합니다.

```bash
roslaunch ajou_control recorded_path_control.launch target_speed_mps:=5.0 velocity_scale:=0.27778
```

제어기가 구독하는 토픽:

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/recorded_trajectory_path` | `nav_msgs/Path` | CSV에서 읽은 기록 경로 |
| `/recorded_trajectory/target_speed` | `std_msgs/Float32` | 목표 속도, 단위 m/s |
| `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | MORAI ego 차량 상태 |

제어기는 `/ctrl_cmd`에 `morai_msgs/CtrlCmd`를 발행합니다. 횡방향은 Pure Pursuit로 조향하고, 종방향은 PI 제어로 accel/brake를 계산합니다. 기본 launch는 낮은 고정 목표 속도를 사용합니다. 저속 추종이 안정된 뒤에만 `use_target_speed_topic:=true`로 기록 속도 기반 목표 속도를 사용하세요.

### 주요 런타임 파라미터

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `target_speed_mps` | `3.0` | 튜닝 중 사용할 낮은 고정 목표 속도 |
| `use_target_speed_topic` | `false` | 고정 목표 속도 대신 `/recorded_trajectory/target_speed` 사용 |
| `steering_sign` | `1.0` | MORAI 조향 부호가 반대면 `-1.0`으로 변경 |
| `velocity_scale` | `1.0` | MORAI 속도가 km/h이면 `0.27778` 사용 |
| `min_lookahead` | `7.0` | Pure Pursuit 최소 lookahead 거리 |
| `max_lookahead` | `16.0` | Pure Pursuit 최대 lookahead 거리 |
| `max_steer_rad` | `0.35` | 조향 명령 제한값 |

## 추종 성능 검증

제어기가 주행 중일 때 validator를 실행합니다.

```bash
roslaunch ajou_control tracking_validation.launch
```

validator는 `/Ego_topic`과 `/recorded_trajectory_path`를 비교해 아래 값을 출력합니다.

| 지표 | 설명 |
| --- | --- |
| `lateral` | 현재 signed 횡방향 오차, 단위 m |
| `lateral_rms` | 검증 구간의 횡방향 RMS 오차 |
| `lateral_max` | 최대 절대 횡방향 오차 |
| `heading_rms` | heading RMS 오차, 단위 rad |
| `speed_rms` | 속도 RMS 오차, 단위 m/s |
| `progress` | 경로 진행률 |
| `steer_sat` | 조향 포화 근처에 있었던 샘플 비율 |

validator는 샘플별 CSV도 저장합니다.

```text
~/catkin_ws/src/ajou_control/validation_tracking.csv
```

## 현재 구현 범위

구현된 항목:

- 경로/토픽 검증용 가상 궤적 생성기
- Pure Pursuit 기반 기록 경로 추종
- MORAI `CtrlCmd`를 발행하는 종방향 PI 속도 제어
- ego, nearest point, target point, lookahead, cross-track error RViz debug marker
- 횡방향, heading, 속도, 진행률, 조향 포화율을 계산하는 tracking validator

스켈레톤 스크립트는 참고용 placeholder로만 남겨두었습니다. 실제 테스트에는 `pure_pursuit_pi_controller.py`와 `tracking_validator.py`를 사용합니다.
