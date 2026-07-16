# Trajectory Validator Implementation

`trajectory_validator`는 제어 입력을 막거나 trajectory를 수정하지 않는다. MORAI에서
실제로 주행된 데이터를 기록하고, 저장된 데이터를 기준 trajectory와 비교해 성능을
평가하는 패키지이다.

구조는 크게 세 기능으로 나눈다.

1. `trajectory_recorder_node.py`: MORAI 주행 데이터와 제어기 세부 수치를 실시간 저장
2. `evaluate_trajectory.py`: 저장된 run 데이터를 불러와 기준 trajectory와 비교
3. `trajectory_overlay_node.py`: 저장된 run 데이터를 RViz에 overlay/replay

## 기본 구조

```text
recording phase:
  /Ego_topic
  /ctrl_cmd
  /control/trajectory
  /control/debug/follower/*
  /CollisionData
  /clock
    -> trajectory_recorder_node.py
    -> /control/validator/sample
    -> /control/validator/run_info
    -> artifacts/control_validation/<run_id>/*.csv
    -> RViz debug path / marker

evaluation phase:
  artifacts/control_validation/<run_id>/*.csv
    -> evaluate_trajectory.py
    -> metrics_timeseries.csv
    -> summary.yaml
    -> report.md

overlay phase:
  artifacts/control_validation/<run_id>/*.csv
    -> trajectory_overlay_node.py
    -> /control/validator/overlay/sample
    -> /control/validator/overlay/run_info
    -> RViz reference/actual/error overlay
    -> rqt_multiplot replay plot
```

recorder는 ROS node로 실행한다. evaluator는 저장된 파일을 읽는 standalone Python
script로 둔다.

## 기능 1. 주행 데이터 저장

### 목적

MORAI에서 실제로 주행한 ego 상태, follower가 보낸 command, follower 내부 제어 수치,
기준 trajectory를 같은 run id 아래에 시간순으로 저장한다. 나중에 어떤 파라미터로 어떤
제어 결과가 나왔는지 비교할 수 있어야 한다.

주 분석용 출력은 topic별 raw CSV를 사람이 따로 join해야 하는 구조가 아니라,
한 row 안에 기준값, ego 실제값, 제어 명령, follower 내부 계산값이 같이 들어간
`run_timeseries.csv`로 둔다. 단, 제어기 튜닝 파라미터는 시간에 따라 변하지 않으므로
CSV row마다 반복 저장하지 않고 `run_metadata.yaml`과 run title에 따로 저장한다.

### 기록 대상 topic

| Topic | Type | 저장 파일 | 용도 |
| --- | --- | --- | --- |
| `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | `ego_state.csv` | 실제 위치/속도 |
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` | `ctrl_cmd.csv` | MORAI에 들어간 최종 제어 명령 |
| `/control/trajectory` | `autoware_planning_msgs/Trajectory` | `reference_trajectory.csv` | 기준 trajectory |
| `/control/debug/follower/target_point` | `geometry_msgs/PoseStamped` | `target_point.csv` | follower target point |
| `/control/debug/follower/state` | `ajou_control_msgs/TrajectoryFollowerDebug` | `follower_state.csv` | Pure Pursuit/PI 내부 계산값 |
| `/CollisionData` | `morai_msgs/CollisionData` | `collision.csv` | 충돌 여부 |
| `/VehicleCollisionData` | `morai_msgs/VehicleCollisionData` | `vehicle_collision.csv` | 차량 충돌 여부 |
| `/clock` | `rosgraph_msgs/Clock` | `clock.csv` | simulation time |

`/control/debug/follower/state`는 follower가 publish하는 전용 debug message이다.
validator는 이 값을 받아 `run_timeseries.csv`, `follower_state.csv`,
`TrajectoryValidationSample.msg`에 같은 의미로 반영한다.

topic별 CSV는 디버깅과 재처리를 위한 raw log 성격이다. 실제 비교와 튜닝에는 아래
통합 시계열 CSV를 우선 사용한다.

### 통합 시계열 CSV

`run_timeseries.csv`는 validator의 핵심 산출물이다. 각 row는 recorder sampling time의
스냅샷이며, 같은 시각 기준으로 아래 값을 함께 저장한다.

| Group | Column 예시 | 의미 |
| --- | --- | --- |
| time | `time`, `ros_time`, `dt` | 기록 시각 |
| gt/reference | `gt_index`, `gt_x`, `gt_y`, `gt_z`, `gt_yaw`, `gt_velocity_mps`, `gt_lanelet_id` | 기준 trajectory에서 ego와 대응되는 목표값 |
| ego_topic | `ego_x`, `ego_y`, `ego_z`, `ego_yaw`, `ego_velocity_mps`, `ego_accel_mps2` | MORAI `/Ego_topic` 실제 주행 지표 |
| error | `lateral_error_m`, `heading_error_rad`, `velocity_error_mps`, `station_error_m` | 기준값 대비 실제 주행 오차 |
| ctrl_cmd | `longlCmdType`, `accel_cmd`, `brake_cmd`, `front_steer_rad`, `rear_steer_rad`, `cmd_velocity_mps`, `cmd_acceleration_mps2` | MORAI에 publish된 최종 제어 명령 |
| follower state | `nearest_index`, `target_index`, `lookahead_m`, `pi_integral`, `pi_output`, `steering_rad`, `steering_clamped_rad`, `control_mode` | Pure Pursuit/PI 내부 계산값 |
| target | `target_x`, `target_y`, `target_yaw`, `target_velocity_mps` | follower가 선택한 lookahead target |
| event | `collision_detected`, `vehicle_collision_detected`, `recording_state` | 충돌/상태 이벤트 |

여기서 `gt_*`는 MVP 기준으로 reference trajectory에서 현재 ego 위치에 projection한 목표
상태를 의미한다. 만약 MORAI에서 별도의 absolute ground-truth state topic을 추가로
쓴다면 그 값은 `morai_gt_*`처럼 별도 prefix로 저장해 혼동을 피한다.

예시:

```csv
time,gt_x,gt_y,gt_yaw,gt_velocity_mps,ego_x,ego_y,ego_yaw,ego_velocity_mps,lateral_error_m,heading_error_rad,velocity_error_mps,front_steer_rad,cmd_velocity_mps,target_index,lookahead_m,pi_output
0.000,935412.12,1915820.45,1.570,4.00,935412.00,1915820.40,1.565,3.82,0.13,-0.005,0.18,0.012,4.00,28,4.50,0.41
0.033,935412.12,1915820.58,1.570,4.00,935412.01,1915820.52,1.566,3.86,0.11,-0.004,0.14,0.014,4.00,29,4.50,0.39
```

### ROS Custom Message

CSV는 저장 포맷이고, RViz와 rqt_multiplot에서 보기 위한 ROS 내부 표준 인터페이스는
custom msg로 둔다. 메시지 패키지는 기존 Autoware 포팅 메시지를 수정하지 않고 새 패키지
`base_ws/src/msgs/ajou_control_msgs`를 만든다.

필요한 메시지는 세 개이다.

```text
ajou_control_msgs/
  msg/
    TrajectoryFollowerDebug.msg
    TrajectoryValidationSample.msg
    TrajectoryValidationRunInfo.msg
```

`TrajectoryFollowerDebug.msg`는 follower가 계산한 Pure Pursuit/PI 내부 상태이다.
validator는 이 topic을 받아 통합 sample을 만든다.

```text
std_msgs/Header header

uint32 nearest_index
uint32 target_index

float64 target_x_m
float64 target_y_m
float64 target_yaw_rad
float64 target_velocity_mps

float64 lookahead_m
float64 lateral_error_m
float64 heading_error_rad
float64 ego_velocity_mps
float64 velocity_error_mps

float64 pi_integral
float64 pi_output

float64 steering_rad
float64 steering_clamped_rad
float64 accel_cmd
float64 brake_cmd
float64 ctrl_velocity_mps

uint8 control_mode
bool is_valid
```

`TrajectoryValidationSample.msg`는 rqt_multiplot과 replay의 핵심 시계열 message이다.
`run_timeseries.csv`의 column은 이 message의 scalar field와 1:1로 맞춘다.

```text
std_msgs/Header header

uint32 sample_index
float64 time_from_start_s

uint32 gt_index
float64 gt_station_m
float64 gt_x_m
float64 gt_y_m
float64 gt_z_m
float64 gt_yaw_rad
float64 gt_velocity_mps
float64 gt_acceleration_mps2
int64 gt_lanelet_id

float64 ego_x_m
float64 ego_y_m
float64 ego_z_m
float64 ego_yaw_rad
float64 ego_velocity_mps
float64 ego_acceleration_mps2

float64 lateral_error_m
float64 longitudinal_error_m
float64 heading_error_rad
float64 velocity_error_mps
float64 station_error_m

uint8 longl_cmd_type
float64 accel_cmd
float64 brake_cmd
float64 front_steer_rad
float64 rear_steer_rad
float64 cmd_velocity_mps
float64 cmd_acceleration_mps2

uint32 nearest_index
uint32 target_index
float64 target_x_m
float64 target_y_m
float64 target_yaw_rad
float64 target_velocity_mps
float64 lookahead_m
float64 pi_integral
float64 pi_output
float64 steering_rad
float64 steering_clamped_rad

bool collision_detected
bool vehicle_collision_detected
uint8 recording_state
```

`TrajectoryValidationRunInfo.msg`는 시간에 따라 변하지 않는 run 제목과 파라미터이다.
이 값은 CSV row마다 반복하지 않고 `run_metadata.yaml`과 이 topic으로만 관리한다.

```text
std_msgs/Header header

string run_id
string run_title
string trajectory_file
string follower_config_file

string lateral_controller
string longitudinal_controller

float64 wheelbase_m
float64 max_steer_rad
float64 min_lookahead_m
float64 lookahead_time_s
float64 max_lookahead_m
float64 steering_gain
float64 pi_kp
float64 pi_ki
float64 pi_integral_limit

string recording_mode
float64 pre_buffer_s
float64 post_buffer_s
```

실시간 주행 중 publish topic:

| Topic | Type | 용도 |
| --- | --- | --- |
| `/control/validator/sample` | `ajou_control_msgs/TrajectoryValidationSample` | rqt_multiplot 실시간 plot, CSV 저장 기준 |
| `/control/validator/run_info` | `ajou_control_msgs/TrajectoryValidationRunInfo` | run 제목, config, 튜닝 파라미터 |

저장 run replay 중 publish topic:

| Topic | Type | 용도 |
| --- | --- | --- |
| `/control/validator/overlay/sample` | `ajou_control_msgs/TrajectoryValidationSample` | 저장 CSV를 rqt_multiplot으로 재생 |
| `/control/validator/overlay/run_info` | `ajou_control_msgs/TrajectoryValidationRunInfo` | 저장 run의 제목/파라미터 재게시 |

rqt_multiplot은 CSV를 직접 읽는 것이 아니라 위 sample topic의 field를 구독한다.
예를 들어 아래 field를 같은 시간축에 겹쳐 볼 수 있다.

```text
/control/validator/sample/gt_velocity_mps
/control/validator/sample/ego_velocity_mps
/control/validator/sample/lateral_error_m
/control/validator/sample/heading_error_rad
/control/validator/sample/front_steer_rad
/control/validator/sample/pi_output
```

RViz는 custom msg를 직접 표시하지 않고, validator가 같은 sample을 기반으로 만든
표준 message를 표시한다.

```text
nav_msgs/Path
geometry_msgs/PoseStamped
visualization_msgs/MarkerArray
```

### follower 내부 수치 저장 항목

나중에 controller tuning을 비교하려면 단순 `/ctrl_cmd`만으로는 부족하다. recorder는
아래 수치를 함께 저장해야 한다.

| Field | 의미 |
| --- | --- |
| `nearest_index` | ego에서 가장 가까운 trajectory point index |
| `target_index` | lookahead target point index |
| `lookahead_m` | Pure Pursuit lookahead 거리 |
| `lateral_error_m` | 현재 ego의 lateral tracking error |
| `heading_error_rad` | trajectory heading 대비 ego heading error |
| `target_velocity_mps` | trajectory에서 선택한 목표 속도 |
| `ego_velocity_mps` | 실제 ego 속도 |
| `velocity_error_mps` | PI 입력 error |
| `pi_integral` | PI 적분 상태 |
| `pi_output` | PI 출력 |
| `steering_rad` | Pure Pursuit steering 결과 |
| `steering_clamped_rad` | saturation 이후 steering |
| `accel_cmd` | 최종 accel command |
| `brake_cmd` | 최종 brake command |
| `ctrl_velocity_mps` | 최종 MORAI velocity command |
| `control_mode` | velocity command / accel-brake command |

이 값들은 `follower_state.csv`에 시계열로 저장한다.

### 저장 디렉터리

기본 구조:

```text
artifacts/control_validation/<run_id>/
  run_metadata.yaml
  run_timeseries.csv
  reference_trajectory.csv
  ego_state.csv
  ctrl_cmd.csv
  follower_state.csv
  target_point.csv
  collision.csv
  vehicle_collision.csv
  clock.csv
  rviz_actual_path.csv
```

`run_metadata.yaml`에는 아래 정보를 저장한다.

```yaml
run_id: 20260717_063000_v44_pp_pi_ld4p0_lt1p5_kp0p60_ki0p05
run_title: v44_pp_pi_ld4p0_lt1p5_kp0p60_ki0p05
started_at: 2026-07-17T06:30:00+09:00
trajectory_file: artifacts/trajectory_editor/v44_section_200889_to_200258.csv
follower_config_file: base_ws/src/control/trajectory_follower/config/trajectory_follower.yaml
controller:
  lateral: pure_pursuit
  longitudinal: pi
parameter_tag:
  map_version: v44
  lateral: pp
  longitudinal: pi
  min_lookahead_m: 4p0
  lookahead_time_s: 1p5
  kp: 0p60
  ki: 0p05
controller_parameters:
  wheelbase_m: 3.0
  max_steer_rad: 0.698
  min_lookahead_m: 4.0
  lookahead_time_s: 1.5
  kp: 0.6
  ki: 0.05
```

제어기 파라미터는 `run_timeseries.csv`에 반복해서 넣지 않는다. 대신 비교용 제목인
`run_title`과 `run_id`에 핵심 파라미터를 짧게 합쳐 넣고, 전체 파라미터는
`run_metadata.yaml`의 `controller_parameters`에 저장한다.

### RViz 확인

recorder는 저장만 하지 말고 RViz에서 실제 주행 경로와 기준 trajectory를 같이 볼 수
있게 debug topic을 publish한다.

| Topic | Type | 용도 |
| --- | --- | --- |
| `/control/validator/reference_path` | `nav_msgs/Path` | 기준 trajectory |
| `/control/validator/actual_path` | `nav_msgs/Path` | MORAI ego 실제 주행 path |
| `/control/validator/error_markers` | `visualization_msgs/MarkerArray` | lateral error, target point, warning |

RViz에서 보면 기준 trajectory와 실제 ego path가 얼마나 벌어졌는지 바로 확인할 수 있다.

### Recording Window

비교 실험에서는 항상 같은 기준으로 기록 구간을 잘라야 한다. trajectory publish 시점은
차량이 실제로 움직이기 전일 수 있으므로, 보조 기능으로 "실제 주행 시작" 기준 recording
window를 둔다.

권장 기준:

```text
start:
  Ego가 trajectory 시작점 근처에 있음
  MORAI가 ExternalCtrl 상태로 전환되어 있음, 확인 가능할 때
  /ctrl_cmd velocity 또는 ego velocity가 threshold 이상
  조건이 hold_time_s 동안 유지됨
  pre_buffer_s 만큼 이전 데이터도 함께 저장

stop:
  Ego가 trajectory 끝점 근처에 있음
  ego velocity가 threshold 이하
  조건이 hold_time_s 동안 유지됨
  post_buffer_s 만큼 이후 데이터도 함께 저장
```

ExternalCtrl 상태를 직접 읽을 수 있는 topic이 없으면 `/ctrl_cmd`가 정상 publish되고
ego velocity가 증가하기 시작한 시점을 실제 주행 시작의 fallback 기준으로 쓴다.

이 방식의 장점:

- 같은 위치, 같은 trajectory를 여러 번 반복할 때 start 기준이 일정하다.
- 사람이 수동으로 start/stop을 누르는 타이밍 차이를 줄인다.
- 출발 전 대기 시간은 metric 계산에서 제외된다.
- pre/post buffer로 출발 직전과 정지 직후 상태도 보존된다.

recorder는 launch 직후 바로 파일에 쓰기 시작하지 않고 `ARMED` 상태로 대기한다.
조건이 만족되면 `RECORDING`으로 전환하고, 종료 조건과 post buffer가 끝나면 파일을
닫는다. 상태 전이는 `run_metadata.yaml`에도 저장한다.

## 기능 2. 저장 데이터 비교 평가

### 입력

`evaluate_trajectory.py`는 recorder가 만든 run directory를 입력으로 받는다.

```bash
python scripts/evaluate_trajectory.py \
  --run-dir artifacts/control_validation/20260717_063000_v44_pp_pi
```

필수 입력 파일:

- `run_timeseries.csv`
- `reference_trajectory.csv`
- `ego_state.csv`
- `ctrl_cmd.csv`

선택 입력 파일:

- `follower_state.csv`
- `target_point.csv`
- `collision.csv`
- `vehicle_collision.csv`

### 비교 방식

각 ego sample을 기준 trajectory에 projection한다.

```text
ego pose at time t
  -> nearest point on reference trajectory
  -> station_s
  -> lateral_error_m
  -> heading_error_rad
  -> reference_velocity_mps
  -> velocity_error_mps
```

이 결과를 시간순으로 `run_timeseries.csv`에 직접 포함하고, evaluator가 후처리하면서
필요한 경우 `metrics_timeseries.csv`로 별도 복사/확장한다.

### Metric 출력

시계열 metric:

```text
metrics_timeseries.csv
  time
  gt_x
  gt_y
  gt_yaw
  ego_x
  ego_y
  ego_yaw
  station_s
  lateral_error_m
  heading_error_rad
  reference_velocity_mps
  ego_velocity_mps
  velocity_error_mps
  steering_rad
  accel_cmd
  brake_cmd
```

요약 metric:

```yaml
summary.yaml
  max_abs_lateral_error_m: 0.82
  mean_abs_lateral_error_m: 0.21
  rms_lateral_error_m: 0.28
  max_abs_heading_error_rad: 0.12
  mean_abs_velocity_error_mps: 0.35
  max_abs_velocity_error_mps: 1.20
  max_abs_steering_rad: 0.42
  collision_detected: false
  reached_goal: true
  total_time_s: 74.3
```

`report.md`에는 run metadata, 주요 그래프 경로, summary, 문제 구간을 사람이 읽기 쉽게
정리한다.

## 기능 3. 저장된 주행 기록 RViz Overlay

### 목적

주행이 끝난 뒤 저장된 run directory를 다시 불러와 RViz에서 기준 trajectory와 실제 주행
기록을 겹쳐 본다. 실시간 recorder가 보여주는 actual path는 주행 중 확인용이고,
overlay 기능은 이미 저장된 여러 run을 비교하거나 report에서 발견한 문제 구간을 다시
확인하기 위한 기능이다.

### 입력

```text
artifacts/control_validation/<run_id>/
  reference_trajectory.csv
  ego_state.csv
  follower_state.csv
  metrics_timeseries.csv
```

`metrics_timeseries.csv`가 없으면 overlay node가 간단한 lateral error를 즉시 계산하거나,
사용자에게 evaluator를 먼저 실행하라고 warning을 낸다.

### Publish topic

| Topic | Type | 용도 |
| --- | --- | --- |
| `/control/validator/overlay/reference_path` | `nav_msgs/Path` | 기준 trajectory |
| `/control/validator/overlay/actual_path` | `nav_msgs/Path` | 저장된 실제 주행 path |
| `/control/validator/overlay/error_markers` | `visualization_msgs/MarkerArray` | lateral error 선/색상 |
| `/control/validator/overlay/ego_pose` | `geometry_msgs/PoseStamped` | replay 중 현재 ego pose |
| `/control/validator/overlay/target_point` | `geometry_msgs/PoseStamped` | 저장된 follower target point |
| `/control/validator/overlay/sample` | `ajou_control_msgs/TrajectoryValidationSample` | rqt_multiplot replay |
| `/control/validator/overlay/run_info` | `ajou_control_msgs/TrajectoryValidationRunInfo` | 저장 run 정보 |

### Overlay 모드

| Mode | 설명 |
| --- | --- |
| `static` | reference path와 actual path 전체를 한 번에 표시 |
| `replay` | 저장된 timestamp 기준으로 ego pose와 target point를 시간 재생 |
| `compare_runs` | 여러 run의 actual path를 색상별로 overlay, 확장 기능 |

MVP는 `static`과 `replay`만 구현한다.

### 색상 기준

`error_markers`는 lateral error 크기에 따라 색상을 바꾼다.

```text
green: abs(error) < lateral_error_warn_m
yellow: lateral_error_warn_m <= abs(error) < lateral_error_fail_m
red: abs(error) >= lateral_error_fail_m
```

이 기준은 evaluator config와 동일한 값을 사용한다.

## Config

예시:

```yaml
topics:
  trajectory: /control/trajectory
  ego: /Ego_topic
  ctrl_cmd: /ctrl_cmd
  follower_state: /control/debug/follower/state
  target_point: /control/debug/follower/target_point
  validation_sample: /control/validator/sample
  validation_run_info: /control/validator/run_info
  collision: /CollisionData
  vehicle_collision: /VehicleCollisionData
  clock: /clock

output:
  root_dir: artifacts/control_validation
  run_id: auto

recording:
  write_rate_hz: 30.0
  save_reference_once: true
  save_follower_config: true
  follower_config_file: base_ws/src/control/trajectory_follower/config/trajectory_follower.yaml

recording_window:
  mode: actual_drive_start
  pre_buffer_s: 3.0
  post_buffer_s: 3.0

  start:
    require_near_trajectory_start: true
    start_distance_m: 3.0
    require_external_control: true
    external_control_topic: ""
    fallback_to_motion_detection: true
    velocity_source: ctrl_cmd
    velocity_threshold_mps: 0.3
    hold_time_s: 0.5

  stop:
    require_near_trajectory_end: true
    goal_distance_m: 3.0
    velocity_source: ego
    velocity_threshold_mps: 0.1
    hold_time_s: 2.0

rviz:
  publish_reference_path: true
  publish_actual_path: true
  publish_error_markers: true

evaluation:
  goal_tolerance_m: 3.0
  lateral_error_warn_m: 1.0
  lateral_error_fail_m: 2.0
```

## 구현 파일 구성

```text
trajectory_validator/
  config/
    trajectory_validator.yaml
  launch/
    trajectory_recorder.launch
    trajectory_overlay.launch
  scripts/
    trajectory_recorder_node.py
    evaluate_trajectory.py
    trajectory_overlay_node.py

base_ws/src/msgs/
  ajou_control_msgs/
    msg/
      TrajectoryFollowerDebug.msg
      TrajectoryValidationSample.msg
      TrajectoryValidationRunInfo.msg
```

## 실행 예시

기록:

```bash
roslaunch trajectory_validator trajectory_recorder.launch \
  config_file:=/path/to/trajectory_validator.yaml
```

평가:

```bash
python base_ws/src/control/trajectory_validator/scripts/evaluate_trajectory.py \
  --run-dir artifacts/control_validation/<run_id>
```

RViz overlay:

```bash
roslaunch trajectory_validator trajectory_overlay.launch \
  run_dir:=artifacts/control_validation/<run_id> \
  mode:=static
```

Replay overlay:

```bash
roslaunch trajectory_validator trajectory_overlay.launch \
  run_dir:=artifacts/control_validation/<run_id> \
  mode:=replay \
  replay_rate:=1.0
```

## 완료 기준

- [ ] recorder가 MORAI ego state를 `ego_state.csv`로 저장한다.
- [ ] recorder가 `/ctrl_cmd`를 `ctrl_cmd.csv`로 저장한다.
- [ ] recorder가 기준 trajectory를 `reference_trajectory.csv`로 저장한다.
- [ ] recorder가 `gt_*`, `ego_*`, `ctrl_cmd`, follower state를 `run_timeseries.csv` 한 row에 함께 저장한다.
- [ ] recorder가 controller tuning 파라미터를 CSV에 반복 저장하지 않고 `run_id`, `run_title`, `run_metadata.yaml`에 저장한다.
- [ ] recorder가 `TrajectoryValidationSample.msg`를 `/control/validator/sample`로 publish한다.
- [ ] recorder가 `TrajectoryValidationRunInfo.msg`를 `/control/validator/run_info`로 publish한다.
- [ ] recorder가 follower 내부 제어 수치를 `follower_state.csv`로 저장한다.
- [ ] recorder가 RViz용 actual/reference path를 publish한다.
- [ ] recorder가 fixed recording window 정책으로 actual-drive-start 기준 기록 시작/종료를 수행한다.
- [ ] recorder가 recording window 조건과 상태 전이를 `run_metadata.yaml`에 저장한다.
- [ ] evaluator가 저장된 run directory를 읽어 lateral/heading/velocity error 시계열을 만든다.
- [ ] evaluator가 summary.yaml과 report.md를 만든다.
- [ ] overlay node가 저장된 reference/actual path를 RViz에 겹쳐 표시한다.
- [ ] overlay node가 replay 모드로 ego pose와 target point를 시간순 publish한다.
- [ ] overlay node가 저장된 CSV를 `TrajectoryValidationSample.msg`로 다시 publish해 rqt_multiplot에서 볼 수 있게 한다.
