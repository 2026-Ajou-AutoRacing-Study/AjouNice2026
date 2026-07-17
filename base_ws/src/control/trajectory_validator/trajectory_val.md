# `trajectory_validator` package design

`trajectory_validator`는 `trajectory_follower`의 추종 결과를 저장하고, 저장된 run 하나를
다시 publish하여 RViz와 `rqt_plot`에서 확인하는 ROS1 catkin 패키지다. trajectory를
수정하거나 제어 명령을 생성하지 않는다.

기능은 두 가지다.

1. `trajectory_validator.launch`: 현재 주행을 CSV/JSON으로 **녹화·저장**한다.
2. `publish_history.py`: 사용자가 지정한 이전 run 하나를 **Python으로 publish**한다.

이전 run publish 기능은 launch에 넣지 않는다. 별도의 replay launch도 만들지 않는다.

## 실행 구조

### 현재 주행 녹화

```text
/control/trajectory  ─┐
/Ego_topic           ├─> trajectory_validator.launch
/ctrl_cmd            ┘        |
                              v
  trajectory_validator/history/<traj_sample>/<run_id>.csv
  trajectory_validator/history/<traj_sample>/<run_id>.json
```

```bash
roslaunch trajectory_validator trajectory_validator.launch \
  input_trajectory_csv:=base_ws/src/control/trajectory_generator/traj_sample/200889_200258.csv \
  run_params:=pp_pi_ld4p0 \
  map_version:=v44
```

`/control/trajectory`의 `autoware_planning_msgs/Trajectory`에는 원본 CSV 파일명이 들어 있지
않다. 따라서 traj sample 폴더를 정확히 만들려면 `input_trajectory_csv`를 launch의 필수
인자로 받아야 한다.

### 이전 run 하나 publish

launch를 사용하지 않고 Python에 traj sample 이름과 run ID를 직접 지정한다.

```bash
python3 base_ws/src/control/trajectory_validator/scripts/publish_history.py \
  --traj-sample 200889_200258 \
  --run-id 0717_143012_pp_pi_ld4p0
```

publisher는 아래 두 파일만 읽는다.

```text
history/200889_200258/0717_143012_pp_pi_ld4p0.csv
history/200889_200258/0717_143012_pp_pi_ld4p0.json
```

`--traj-sample`과 `--run-id`에 해당하는 정확한 run만 publish한다. publish 주기는 별도
인자로 받지 않고 **30 Hz로 고정**한다. 전체 history 자동 재생, 여러 run 동시 overlay,
가장 최근 run 자동 선택은 하지 않는다.

## 패키지 구조

```text
base_ws/src/control/trajectory_validator/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── msg/
│   └── TrajectoryDebug.msg
├── config/
│   └── trajectory_validator.yaml
├── launch/
│   └── trajectory_validator.launch       # 녹화·저장 전용
├── scripts/
│   ├── trajectory_validator_node.py       # launch에서 실행하는 recorder
│   └── publish_history.py                 # 이전 run 하나를 publish
├── src/trajectory_validator/
│   ├── recorder.py
│   ├── history_publisher.py
│   ├── csv_io.py
│   ├── metrics.py
│   └── visualization.py
└── history/
    └── <traj_sample>/
        ├── <run_id>.csv
        └── <run_id>.json
```

## history 경로와 이름

### traj sample 폴더

history는 **trajectory_generator가 만든 입력 trajectory CSV의 basename**별로 나눈다.
이 basename을 문서와 인터페이스에서 `traj_sample`이라고 부른다.

```text
input trajectory CSV                         traj_sample
traj_sample/200889_200258.csv          ->    200889_200258
traj_sample/200889_200300.csv          ->    200889_200300
```

저장 경로:

```text
base_ws/src/control/trajectory_validator/history/<traj_sample>/
```

경로의 디렉터리명에는 `.csv` 확장자를 넣지 않는다.

### run ID와 파일명

run ID는 다음 형식으로 자동 생성한다.

```text
<월일>_<시간>_<param1>_<param2>_<param3>
MMDD_HHMMSS_param1_param2_param3
```

예시:

```text
0717_143012_pp_pi_ld4p0

history/200889_200258/0717_143012_pp_pi_ld4p0.csv
history/200889_200258/0717_143012_pp_pi_ld4p0.json
```

- `MMDD`: 녹화 시작 월·일
- `HHMMSS`: 녹화 시작 시·분·초
- `param1_param2_param3`: launch의 `run_params` 값
- parameter 개수는 세 개로 고정하지 않고 `_`로 연결한다.
- 전체 controller parameter는 파일명에 모두 넣지 않고 JSON에 저장한다.
- 같은 초에 같은 이름이 이미 있으면 `_01`, `_02`를 붙이며 기존 run을 덮어쓰지 않는다.
- traj sample 이름과 parameter tag에는 `[A-Za-z0-9_-]`만 허용한다.

## `TrajectoryDebug.msg`

ROS message 파일명은 `msg/TrajectoryDebug.msg`, 생성 type은
`trajectory_validator/TrajectoryDebug`로 한다.

```text
std_msgs/Header header

string traj_sample
string run_id
uint32 sample_index
float64 elapsed_time_s

# follower가 받은 전체 기준 trajectory와 현재까지의 실제 ego trajectory
autoware_planning_msgs/Trajectory gt_trajectory
autoware_planning_msgs/Trajectory ego_trajectory

# 현재 sample의 MORAI 차량 상태
int32 ego_unique_id
float64 ego_x_m
float64 ego_y_m
float64 ego_z_m
float64 ego_yaw_rad
float64 ego_heading_raw
float64 ego_velocity_x_mps
float64 ego_velocity_y_mps
float64 ego_velocity_z_mps
float64 ego_velocity_mps
float64 ego_acceleration_x_mps2
float64 ego_acceleration_y_mps2
float64 ego_acceleration_z_mps2
float64 ego_acceleration_mps2
float64 ego_accel
float64 ego_brake
float64 ego_wheel_angle_raw
float64 actual_steering_rad
float64 ego_lateral_offset_m

# follower가 publish한 현재 sample의 최종 명령
int32 cmd_longl_type
float64 cmd_accel
float64 cmd_brake
float64 cmd_steering_rad
float64 cmd_velocity_mps
float64 cmd_acceleration_mps2

# 현재 ego와 대응하는 GT 상태 및 비교 지표
uint32 matched_gt_index
float64 matched_gt_station_m
float64 target_velocity_mps
float64 lateral_error_m
float64 longitudinal_error_m
float64 heading_error_rad
float64 velocity_error_mps

bool collision_detected
bool reached_goal
bool sample_valid
```

`gt_trajectory`는 follower가 실제로 받은 입력 trajectory 전체다.
`ego_trajectory`는 `/Ego_topic`을 `TrajectoryPoint[]`로 변환한 실제 주행 궤적이며,
이전 run 재생 중에는 현재 sample까지 누적된 points를 담는다. 차량 상태와 command는
CSV와 같은 이름의 scalar field로 평탄화한다. 따라서 별도의 중첩
`EgoVehicleStatus`/`CtrlCmd` field는 두지 않는다.

## `TrajectoryDebug.msg`와 CSV의 대응

두 형식은 **의미상 거의 1:1로 매칭**한다. 다만 ROS message는 nested message와 배열을
가질 수 있고 CSV는 평면 row이므로 저장 모양만 다르다.

| `TrajectoryDebug.msg` | CSV 저장 형태 |
| --- | --- |
| `traj_sample`, `run_id` | 모든 row의 `traj_sample`, `run_id` |
| `header.stamp` | `SAMPLE.ros_time_ns` |
| `sample_index`, `elapsed_time_s` | 같은 이름의 `SAMPLE` column |
| `gt_trajectory.points[]` | point별 `GT_POINT` row |
| `ego_trajectory.points[]` | 시간순 `SAMPLE` row의 ego pose/motion column |
| `ego_*`, `actual_steering_rad` | 같은 이름의 `SAMPLE` column |
| `cmd_*` | 같은 이름의 `SAMPLE` column |
| `matched_gt_*`, velocity, steering, error | 같은 의미와 단위의 `SAMPLE` column |
| `collision_detected`, `reached_goal`, `sample_valid` | 같은 이름의 `SAMPLE` column |

예외는 다음뿐이다.

- `header.frame_id`는 매 row에 반복하지 않고 JSON의 `frame_id`에 저장한다.
- publish 시각처럼 저장 데이터가 아닌 값은 Python publisher가 다시 만든다.
- `gt_trajectory` 배열은 CSV 한 cell에 넣지 않고 `GT_POINT` 여러 row로 저장한다.
- `header.seq`는 저장하지 않고 publish 순서에 맞춰 `sample_index`로 다시 만든다.

즉 CSV에서 run을 읽으면 `TrajectoryDebug.msg`를 손실 없이 다시 구성할 수 있어야 한다.

## CSV 형식

하나의 `<run_id>.csv`에 GT 전체와 주행 시계열을 같이 저장한다. `row_type`으로 두 종류를
구분한다.

- `GT_POINT`: 입력 GT trajectory의 point 하나
- `SAMPLE`: 해당 시각의 ego 상태, command, 대응 GT, 오차 지표

차량이 중간에 멈춰도 입력 GT 전체를 복원할 수 있도록 `GT_POINT`는 recording 시작 시
모두 저장한다.

### 공통/GT column

```text
schema_version,row_type,traj_sample,run_id
gt_index,gt_time_from_start_s
gt_x_m,gt_y_m,gt_z_m,gt_qx,gt_qy,gt_qz,gt_qw,gt_yaw_rad
gt_longitudinal_velocity_mps,gt_lateral_velocity_mps
gt_acceleration_mps2,gt_heading_rate_rps
gt_front_wheel_angle_rad,gt_rear_wheel_angle_rad
```

### SAMPLE column

```text
sample_index,ros_time_ns,elapsed_time_s,dt_s
matched_gt_index,matched_gt_station_m,target_velocity_mps

ego_unique_id
ego_x_m,ego_y_m,ego_z_m,ego_yaw_rad,ego_heading_raw
ego_velocity_x_mps,ego_velocity_y_mps,ego_velocity_z_mps,ego_velocity_mps
ego_acceleration_x_mps2,ego_acceleration_y_mps2,ego_acceleration_z_mps2
ego_acceleration_mps2
ego_accel,ego_brake,ego_wheel_angle_raw,actual_steering_rad
ego_lateral_offset_m

cmd_longl_type,cmd_accel,cmd_brake,cmd_steering_rad
cmd_velocity_mps,cmd_acceleration_mps2

lateral_error_m,longitudinal_error_m
heading_error_rad,velocity_error_mps
collision_detected,reached_goal,sample_valid
```

`GT_POINT`에서 쓰지 않는 SAMPLE column과 `SAMPLE`에서 쓰지 않는 GT 전용 column은 빈
문자열로 저장한다. 임의의 `0`으로 채우지 않는다. 모든 정규화 값은 SI 단위
(`m`, `s`, `rad`)를 사용하며 MORAI 원본 heading/wheel angle은 `*_raw`로 함께 보존한다.

## JSON 형식

같은 run ID의 JSON에는 시계열이 아닌 고정 정보와 결과 요약을 저장한다.

```json
{
  "schema_version": 1,
  "traj_sample": "200889_200258",
  "run_id": "0717_143012_pp_pi_ld4p0",
  "status": "complete",
  "started_at": "2026-07-17T14:30:12+09:00",
  "finished_at": "2026-07-17T14:31:26+09:00",
  "frame_id": "map",
  "input_trajectory": {
    "file": "base_ws/src/control/trajectory_generator/traj_sample/200889_200258.csv",
    "sha256": "...",
    "point_count": 1840
  },
  "map": {
    "version": "v44",
    "file": "kcity_map/lanelet_map/lanelet2_map_v44.osm",
    "sha256": "..."
  },
  "controller": {
    "run_params": ["pp", "pi", "ld4p0"],
    "config_file": "base_ws/src/control/trajectory_follower/config/trajectory_follower.yaml",
    "config_sha256": "...",
    "parameters": {
      "min_lookahead_m": 4.0,
      "lookahead_time_s": 1.5,
      "kp": 0.6,
      "ki": 0.05
    }
  },
  "recording": {
    "sample_rate_hz": 30.0,
    "sample_count": 2229,
    "duration_s": 74.3,
    "end_reason": "goal_reached"
  },
  "result": {
    "reached_goal": true,
    "collision_detected": false,
    "mean_abs_lateral_error_m": 0.21,
    "rms_lateral_error_m": 0.28,
    "max_abs_lateral_error_m": 0.82,
    "mean_abs_heading_error_rad": 0.031,
    "max_abs_heading_error_rad": 0.12,
    "mean_abs_velocity_error_mps": 0.35,
    "max_abs_velocity_error_mps": 1.20
  },
  "integrity": {
    "csv_sha256": "...",
    "gt_row_count": 1840,
    "sample_row_count": 2229
  }
}
```

파일명에는 비교에 필요한 짧은 parameter tag만 넣고, 실제 적용한 전체 parameter와 config
hash는 JSON에 둔다. 알 수 없는 map/config 정보는 추측하지 않고 `null`로 저장한다.

## metric 정의

```text
dx = ego_x - gt_x
dy = ego_y - gt_y

lateral_error_m      = -sin(gt_yaw) * dx + cos(gt_yaw) * dy
longitudinal_error_m =  cos(gt_yaw) * dx + sin(gt_yaw) * dy
heading_error_rad    = normalize(ego_yaw - gt_yaw)  # [-pi, pi]
velocity_error_mps   = ego_velocity_mps - target_velocity_mps
```

각 ego sample을 GT polyline에 투영하여 `matched_gt_index`와 station을 정한다. 오차 부호와
단위는 녹화와 재생에서 동일하게 유지한다. mean/RMS/max 결과는 recording 종료 시 JSON에
기록한다.

## ROS interface

### recorder subscribe

| Topic | Type |
| --- | --- |
| `/control/trajectory` | `autoware_planning_msgs/Trajectory` |
| `/Ego_topic` | `morai_msgs/EgoVehicleStatus` |
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` |
| `/CollisionData` | `morai_msgs/CollisionData` (선택) |
| `/clock` | `rosgraph_msgs/Clock` (`/use_sim_time=true`일 때) |

### live/replay publish

| Topic | Type | 용도 |
| --- | --- | --- |
| `/trajectory_validator/debug` | `trajectory_validator/TrajectoryDebug` | 전체 debug 데이터, rqt_plot |
| `/trajectory_validator/gt_path` | `nav_msgs/Path` | RViz GT trajectory |
| `/trajectory_validator/ego_path` | `nav_msgs/Path` | RViz ego trajectory |
| `/trajectory_validator/ego_pose` | `geometry_msgs/PoseStamped` | 현재 replay sample |
| `/trajectory_validator/markers` | `visualization_msgs/MarkerArray` | 대응 GT point와 오차 표시 |

RViz는 custom message를 직접 그리지 못하므로 Python publisher와 recorder가 같은
`TrajectoryDebug` 데이터에서 `Path`, `PoseStamped`, `MarkerArray`를 변환해 publish한다.

`rqt_plot`은 다음 scalar field를 직접 사용한다.

```text
/trajectory_validator/debug/lateral_error_m
/trajectory_validator/debug/heading_error_rad
/trajectory_validator/debug/velocity_error_mps
/trajectory_validator/debug/target_velocity_mps
/trajectory_validator/debug/ego_velocity_mps
/trajectory_validator/debug/actual_steering_rad
/trajectory_validator/debug/cmd_steering_rad
/trajectory_validator/debug/cmd_accel
/trajectory_validator/debug/cmd_brake
```

## Config/launch 기준

```yaml
topics:
  trajectory: /control/trajectory
  ego: /Ego_topic
  ctrl_cmd: /ctrl_cmd
  collision: /CollisionData

storage:
  history_dir: base_ws/src/control/trajectory_validator/history
  schema_version: 1
  overwrite: false

recording:
  sample_rate_hz: 30.0
  input_timeout_s: 0.5
  sync_tolerance_s: 0.05
  goal_tolerance_m: 3.0
```

`publish_history.py`의 publish rate는 config로 받지 않고 30 Hz로 고정한다.

`trajectory_validator.launch`가 받는 핵심 인자는 아래와 같다.

| Launch arg | 필수 | 의미 |
| --- | --- | --- |
| `input_trajectory_csv` | 필수 | traj sample basename과 원본 trajectory hash를 결정 |
| `run_params` | 필수 | run ID 뒤에 붙일 `_` 구분 parameter tag |
| `map_version` | 필수 | JSON에 저장할 사용 map version |
| `follower_config_file` | 권장 | 전체 parameter와 hash를 JSON에 저장 |

이 launch에는 `run_id`, `replay`, `rate`, `playback_rate`, `history_select` 같은 이전 run publish
인자를 넣지 않는다.

## 완료 조건

- `trajectory_validator.launch`는 현재 주행 녹화만 수행한다.
- 이전 run은 `publish_history.py --traj-sample ... --run-id ...`로 하나만 선택해
  30 Hz로 publish한다.
- 입력 `200889_200258.csv`의 run은 `history/200889_200258/` 아래에만 저장된다.
- CSV/JSON 이름은 `MMDD_HHMMSS_param1_param2_param3` 형식을 따른다.
- CSV의 `GT_POINT`와 `SAMPLE`에서 `TrajectoryDebug.msg`를 손실 없이 복원할 수 있다.
- JSON에 map version, 입력 trajectory/config hash, 전체 parameter, mean/RMS/max 결과가
  저장된다.
- RViz에서 GT/ego 경로를 확인하고 `rqt_plot`에서 steering과 error를 시간순으로 볼 수
  있다.
