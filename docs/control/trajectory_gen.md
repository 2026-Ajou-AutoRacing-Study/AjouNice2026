# Trajectory Gen Node

`trajectory_gen`은 planning stack이 완성되기 전에도 제어단을 개발, 튜닝, 평가할 수 있도록 임의의 trajectory를 생성해 publish하는 개발용 planning 대체 노드다. 핵심은 경로를 쉽게 만들고, 수정하고, 버전 관리하고, Autoware 형태의 trajectory로 안정적으로 publish하는 것이다.

## 한 줄 정의

주행 기록, ISEV CSV waypoint, 수동 편집 파일, 파라메트릭 shape, rosbag 등 여러 source에서 trajectory asset을 만들고, 이를 `autoware_planning_msgs/Trajectory`와 같은 의미 구조로 변환해 `/planning/trajectory_raw`에 publish한다.

## 설계 원칙

- Control input contract는 처음부터 Autoware `Trajectory` 기준으로 고정한다.
- Source가 `nav_msgs/Path`, CSV waypoint, odometry, GPS, 수동 shape 중 무엇이든 controller 앞에서는 같은 trajectory 형태를 보게 한다.
- Data asset은 사람이 직접 열어 보고 수정하기 쉬워야 한다.
- 속도, 가속도, yaw, heading rate, steering angle은 trajectory의 일부로 관리한다.
- 실제 planning으로 교체될 때 pure pursuit/PID 코드를 바꾸지 않도록 한다.
- MORAI, rosbag, 수동 생성, 기록 주행, ISEV legacy CSV 차이는 importer/exporter에서 흡수한다.
- Generator는 개발용 노드지만 결과물은 validator와 controller가 믿고 볼 수 있을 만큼 엄격하게 만든다.

## 참고한 ISEV 아이디어

참고 대상은 [`5th-IS-EV-autonomous-racing-1-5/ISEV_AJOUNICE2026`](https://github.com/5th-IS-EV-autonomous-racing-1-5/ISEV_AJOUNICE2026.git)이다. 이 레포의 핵심은 `waypoint_pub`이 CSV/UTM waypoint를 만들고 publish하며, `pathplanner`가 후보 경로를 만들고 선택한 뒤 active path와 속도 명령으로 이어지는 구조다.

### `waypoint_pub`에서 가져올 점

- `csv_pub.py`는 `INDEX,UTM_X,UTM_Y` 형태의 CSV를 읽어 `/planning/waypoint` `nav_msgs/Path`로 publish한다.
- `old_odom`과 `fused_odom`을 timestamp pair로 모아 UTM 계열 좌표와 map 계열 좌표 사이 transform을 lock/refine하는 구조가 있다.
- CSV를 뒤집어 publish하는 `reverse_path`가 있고, RViz marker도 같이 publish한다.
- waypoint index를 계산해 `/planning/waypoint_index`로 내보내고, index range 기반으로 `/planning/waypoint_gate_open`, `/planning/waypoint_speed_weight`를 publish한다.
- 설정 파일에는 `gps_allowed_index_ranges: "0-660"`처럼 주행 가능 구간을 range로 관리하고, `speed_weight_ranges: "0-660:1.0, 661-885:0.7"`처럼 구간별 속도 weight를 관리한다.
- `curvature_path_maker.py`는 현재 pose에서 시작해 `straight -> arc -> straight -> arc ...` 형태로 직선 길이, 원호 반지름, 좌/우 방향, 원호 각도를 입력받아 path를 생성한다.
- `waypoint_maker.py`, `waypoint_maker_utm.py`는 키보드로 현재 pose를 waypoint로 찍고, undo/clear/preview/save를 지원한다.

### `pathplanner`에서 가져올 점

- `scene_builder -> path_selector -> path_stitcher -> velocity_planner` 흐름이 있다.
- `path_selector`는 ego pose 기준 straight/arc/cubic primitive bank를 만들고, lane/reference/occupancy grid를 기준으로 후보를 점수화한다.
- 후보 경로는 `CandidatePathArray`로 publish되며, `CandidatePath`에는 `id`, `family`, `name`, `collision`, `inside_lane`, `ade`, `fde`, `score_total`, `nav_msgs/Path path`가 들어간다.
- 선택된 raw path는 `/planning/selected_path_raw`, stitch된 active path는 `/planning/active_path`로 이어진다.
- `velocity_planner`는 active path의 curvature, candidate feasibility, prior mode, waypoint speed weight를 이용해 속도 명령을 만든다.

### 이번 설계에서의 적용 방식

ISEV는 `nav_msgs/Path` 중심이고 이번 제어단 계약은 Autoware-compatible `Trajectory` 중심이다. 따라서 ISEV의 좋은 점은 그대로 가져오되, 최종 output은 `Trajectory`로 승격한다.

- ISEV CSV는 `isev_csv_waypoint` source type으로 import한다.
- ISEV의 index range, gate, speed weight는 `segments.csv` 또는 `trajectory.yaml` metadata로 보존한다.
- ISEV의 `curvature_path_maker` 방식은 `interactive_curvature_builder` 또는 `parametric` source type으로 흡수한다.
- ISEV의 candidate bank 개념은 synthetic trajectory를 빠르게 만들기 위한 `candidate_bank` source type으로 재사용한다.
- Debug와 호환을 위해 `nav_msgs/Path`도 publish할 수 있지만, controller의 기준 입력은 `/planning/trajectory_raw`다.

## Scope

포함하는 것:

- trajectory asset load/save 규칙
- ISEV CSV waypoint import/export 규칙
- recorded path를 trajectory로 변환
- 수동/파라메트릭 trajectory 생성
- straight/arc/candidate primitive 기반 trajectory 생성
- 속도 profile 생성
- Autoware-compatible trajectory publish
- debug path/marker/status publish
- controller tuning을 위한 trajectory switching, loop, pause, replay

포함하지 않는 것:

- 장애물 회피 planning
- 신호등/교차로 판단
- lanelet route generation
- controller 내부 pure pursuit/PID 구현
- MORAI `/ctrl_cmd` 직접 publish

## 기준 출력 메시지

출력은 `autoware_planning_msgs/Trajectory`와 같은 의미 구조를 따른다. Autoware planning interface에서 `/planning/trajectory`는 controller가 따라야 하는 위치, 속도, 가속도 point sequence다.

```text
Trajectory
  std_msgs/Header header
  TrajectoryPoint[] points

TrajectoryPoint
  builtin_interfaces/Duration time_from_start
  geometry_msgs/Pose pose
  float32 longitudinal_velocity_mps
  float32 lateral_velocity_mps
  float32 acceleration_mps2
  float32 heading_rate_rps
  float32 front_wheel_angle_rad
  float32 rear_wheel_angle_rad
```

ROS 1에서 직접 메시지를 만들 경우에도 필드 이름과 단위는 위 의미를 유지한다. `builtin_interfaces/Duration`은 ROS 1의 `duration`에 대응시킨다.

| Field | Unit | Generator에서 채우는 방법 |
|---|---:|---|
| `header.stamp` | ROS time | publish 시각. replay 고정 모드에서는 asset time도 선택 가능 |
| `header.frame_id` | frame | 기본 `map` |
| `points[].time_from_start` | s | trajectory 시작점 또는 local horizon 시작점 기준 누적 시간 |
| `points[].pose.position` | m | source point/resampled point |
| `points[].pose.orientation` | quaternion | yaw에서 변환. yaw는 tangent, recorded pose, 수동 입력 중 하나 |
| `points[].longitudinal_velocity_mps` | m/s | speed profile 결과 |
| `points[].lateral_velocity_mps` | m/s | 일반 주행은 0.0 |
| `points[].acceleration_mps2` | m/s^2 | velocity profile의 시간 미분 또는 segment override |
| `points[].heading_rate_rps` | rad/s | `velocity * curvature` |
| `points[].front_wheel_angle_rad` | rad | bicycle model 기준 `atan(wheelbase * curvature)` |
| `points[].rear_wheel_angle_rad` | rad | 기본 0.0, 후진/특수 조향 시 override 가능 |

## Data Asset 구조

권장 저장 위치는 `configs/control/trajectory_gen/trajectories/`다. 코드는 아직 만들지 않지만, 명세 기준 경로를 먼저 고정해 둔다.

```text
configs/control/trajectory_gen/
  catalog.yaml
  trajectories/
    isev_forward_utm_001/
      trajectory.yaml
      source.csv
      points.csv
      segments.csv
      notes.md
    morai_test_straight_001/
      trajectory.yaml
      points.csv
      notes.md
    curvature_slow_curve_001/
      trajectory.yaml
      generated_points.csv
      segments.csv
      notes.md
```

### `catalog.yaml`

`catalog.yaml`은 사용 가능한 trajectory 목록과 기본 launch target을 관리한다.

```yaml
default: isev_forward_utm_001
trajectories:
  - id: isev_forward_utm_001
    path: trajectories/isev_forward_utm_001/trajectory.yaml
    description: ISEV forward UTM waypoint를 Autoware trajectory로 변환
  - id: morai_test_straight_001
    path: trajectories/morai_test_straight_001/trajectory.yaml
    description: MORAI 직선 주행 제어 튜닝용
```

### `trajectory.yaml`

`trajectory.yaml`은 metadata와 generation policy를 담는다.

```yaml
version: 1
id: isev_forward_utm_001
frame_id: map
source:
  type: isev_csv_waypoint
  file: source.csv
  format: utm_csv
  columns:
    index: INDEX
    x: UTM_X
    y: UTM_Y
  original_repo: https://github.com/5th-IS-EV-autonomous-racing-1-5/ISEV_AJOUNICE2026.git
  original_package: waypoint_pub
  original_file: waypoint_pub/path/최종정방향utm.csv
  reverse_path: false
coordinate:
  output_frame: map
  input_frame: utm
  transform_mode: fixed_asset_transform
  utm_to_map:
    yaw_rad: 0.0
    tx_m: 0.0
    ty_m: 0.0
  note: fixed transform은 import 시 계산하거나 사람이 기록한다.
vehicle:
  wheelbase_m: 3.0
  max_wheel_angle_rad: 0.698
resampling:
  interval_m: 0.5
  smoothing: none
speed_profile:
  mode: segment_override
  default_velocity_mps: 5.0
  max_acceleration_mps2: 1.0
  max_deceleration_mps2: -2.0
publish:
  mode: local_horizon
  planning_hz: 10.0
  backward_length_m: 2.0
  forward_length_m: 80.0
  loop: false
compat:
  publish_nav_path: true
  preserve_source_index: true
```

### `points.csv`

`points.csv`는 publish 가능한 trajectory point table이다. 대량 point 편집과 diff가 쉬운 table format으로 둔다.

```csv
source_index,s_m,x_m,y_m,z_m,yaw_rad,velocity_mps,acceleration_mps2,heading_rate_rps,front_wheel_angle_rad,rear_wheel_angle_rad,gear,tag
0,0.0,402465.98,4132959.49,0.0,,5.0,0.0,,,,forward,start
1,0.5,402466.08,4132959.98,0.0,,5.0,0.0,,,,forward,
2,1.0,402466.18,4132960.47,0.0,,5.0,0.0,,,,forward,
```

비워 둔 값은 generator가 계산한다. 예를 들어 `yaw_rad`가 비어 있으면 이웃 point tangent로 계산하고, `front_wheel_angle_rad`가 비어 있으면 curvature와 wheelbase로 계산한다.

### `segments.csv`

Trajectory를 쉽게 수정하려면 index range 또는 arc-length range에 의미를 붙여야 한다. ISEV에서 code/config에 있던 waypoint index range를 data asset으로 보존한다.

```csv
name,start_source_index,end_source_index,start_s_m,end_s_m,target_velocity_mps,speed_weight,gate_open,gear,behavior
normal_01,0,660,,,5.0,1.0,true,forward,cruise
slow_curve_01,661,885,,,3.5,0.7,true,forward,slow
stop_01,,,,82.0,84.0,0.0,1.0,true,forward,stop
parking_reverse_01,,,,84.0,96.0,1.0,1.0,true,reverse,parking
```

규칙:

- `source_index`는 원본 CSV index를 보존한다.
- `s_m` range는 resampling 후에도 안정적으로 유지되는 기준이다.
- 둘 다 있으면 `s_m`을 우선하고, `source_index`는 추적/debug에 쓴다.
- ISEV의 `gps_allowed_index_ranges`는 `gate_open` segment로 변환한다.
- ISEV의 `speed_weight_ranges`는 `speed_weight` 또는 `target_velocity_mps`로 변환한다.

## Source Type

### 1. `isev_csv_waypoint`

ISEV `waypoint_pub/path/*.csv`를 import한다. 기본 입력 format은 `INDEX,UTM_X,UTM_Y` 또는 `INDEX,MAP_X,MAP_Y`다.

동작:

- CSV header를 읽어 UTM인지 MAP인지 판별한다.
- `reverse_path`가 true면 publish 순서를 뒤집는다.
- `INDEX`는 `source_index`로 보존한다.
- UTM 입력은 `coordinate` 설정에 따라 `map` frame으로 변환한다.
- 변환 후 arc-length 기준으로 resampling하고 yaw/curvature/speed/time을 채운다.
- ISEV config의 `gps_allowed_index_ranges`, `speed_weight_ranges`가 있으면 `segments.csv`로 import한다.

초기 구현 권장:

- 개발 재현성을 위해 runtime마다 transform을 다시 추정하지 말고, import 시점에 계산한 `utm_to_map`을 asset에 저장한다.
- ISEV `csv_pub.py`처럼 `old_odom`/`fused_odom` pair로 transform을 추정하는 기능은 `record/import tool`로 분리하고, publish runtime은 고정 asset을 쓰는 것이 좋다.
- 필요하면 `transform_mode: online_lock_refine`을 별도 실험 옵션으로 둔다.

### 2. `recorded_odom`

실제 주행 또는 MORAI 주행 중 `/localization/odom`을 기록해 trajectory asset을 만든다. 제어 튜닝에 가장 중요하다.

동작:

- odometry pose를 일정 거리 간격으로 sampling한다.
- pose yaw와 twist가 있으면 기록한다.
- raw recording은 별도 파일로 보존하고, publish용 `points.csv`는 resampling/smoothing 후 생성한다.

주의:

- 기록 당시 localization frame과 현재 `map` frame이 같아야 한다.
- 기록 속도를 그대로 쓸지, tuning용 speed profile로 덮어쓸지 metadata에 명시해야 한다.

### 3. `recorded_gps`

MORAI `/gps` 또는 실제 GPS를 기록해 만든다.

동작:

- latitude/longitude를 map frame 좌표로 변환한다.
- 설정된 거리 이상 이동했을 때만 point를 저장한다.
- 이후 odometry 기반 yaw 보정 또는 tangent yaw 계산을 수행한다.

주의:

- 좌표 변환 기준을 반드시 asset metadata에 남긴다.
- GPS만으로 yaw와 속도가 불안정하면 후처리에서 계산한다.

### 4. `interactive_curvature_builder`

ISEV `curvature_path_maker.py`의 직선/원호 입력 방식을 trajectory asset으로 만든다.

사용 예:

```yaml
source:
  type: interactive_curvature_builder
  start_pose_source: current_ego_pose
  point_spacing_m: 0.2
  segments:
    - type: straight
      length_m: 10.0
    - type: arc
      radius_m: 6.0
      direction: left
      angle_deg: 45.0
    - type: straight
      length_m: 12.0
```

장점:

- planning 없이도 곡률이 명확한 test trajectory를 빠르게 만들 수 있다.
- pure pursuit lookahead, steering limit, PID 감속 튜닝에 좋다.
- 직선/원호 segment가 data로 남기 때문에 나중에 다시 수정하기 쉽다.

### 5. `manual_csv`

사람이 `points.csv`를 직접 수정해 trajectory를 만든다.

사용 예:

- 직선, 원호, S자, 특정 곡률 구간만 빠르게 만들기
- stop point나 slow zone을 명확히 넣기
- controller regression test용 작은 trajectory 만들기

### 6. `parametric`

YAML에서 shape parameter를 지정하면 generator가 point를 만든다.

지원 후보:

- `straight`: 길이, 시작 pose, 속도
- `circle`: 반지름, 각도, 방향, 속도
- `arc`: 반지름, sweep angle
- `s_curve`: amplitude, wavelength, length
- `lane_change`: lateral shift, length
- `stop_and_go`: 정지 위치, 정지 시간, 재가속 구간

### 7. `candidate_bank`

ISEV `path_selector`의 primitive bank 아이디어를 offline trajectory 생성에 쓴다.

```yaml
source:
  type: candidate_bank
  start_pose:
    x_m: 0.0
    y_m: 0.0
    yaw_rad: 0.0
  horizon_m: 30.0
  sample_ds_m: 0.2
  families:
    - type: straight
      length_m: 30.0
    - type: arc
      radius_m: [5.0, 8.0, 12.0]
      direction: [left, right]
      angle_deg: [30.0, 60.0]
    - type: cubic
      end_offset_m: [-2.0, 0.0, 2.0]
```

`path_selector`처럼 여러 후보를 동시에 고르는 planner는 아니고, 테스트 목적의 trajectory asset을 생성하는 source다. 후보를 여러 개 만들 경우 각각 별도 trajectory id로 저장하거나, `catalog.yaml`에서 variant로 관리한다.

### 8. `rosbag_replay`

rosbag에서 odometry/gps/ego status를 읽어 asset을 만들거나, replay 중 live로 trajectory를 publish한다.

권장 방식은 bag을 직접 runtime source로 쓰기보다 먼저 asset으로 export하는 것이다. 그래야 trajectory diff, review, 재현이 쉽다.

## 생성 파이프라인

```text
source load
  -> coordinate normalization
  -> source metadata import
  -> point cleanup
  -> arc-length resampling
  -> yaw calculation
  -> curvature calculation
  -> velocity profile generation
  -> time/acceleration/heading-rate/steering fill
  -> self validation
  -> publish
```

### 1. Coordinate normalization

모든 point는 publish 전에 `frame_id: map` 기준으로 정규화한다. Source가 GPS면 UTM 또는 map projector를 거쳐 `x_m`, `y_m`으로 변환한다. Source가 odometry면 odometry frame이 현재 control stack의 map frame과 같은지 metadata에 남긴다.

ISEV UTM CSV import의 경우 두 가지 mode를 둔다.

| Mode | Description | Use case |
|---|---|---|
| `fixed_asset_transform` | import 시 계산한 `yaw/tx/ty`를 asset에 저장하고 runtime에는 그대로 사용 | 재현 가능한 제어 튜닝 기본값 |
| `online_lock_refine` | ISEV `csv_pub.py`처럼 old/fused odom pair로 transform을 추정/보정 | localization frame drift를 보며 실험할 때 |

초기 기본값은 `fixed_asset_transform`이다.

### 2. Source metadata import

ISEV CSV처럼 원본에 담긴 index와 range 의미는 버리면 안 된다.

- `INDEX`는 `source_index`로 보존한다.
- 원본 파일명, 원본 repo, 원본 package를 `trajectory.yaml`에 기록한다.
- `reverse_path` 적용 여부를 metadata에 남긴다.
- gate/speed weight range는 `segments.csv`로 옮긴다.
- imported source와 resampled point 사이 mapping을 가능하면 유지한다.

### 3. Point cleanup

기본 cleanup:

- 중복 point 제거
- 너무 가까운 point 제거
- NaN/Inf 제거
- 비정상적으로 큰 jump 표시
- z 값이 없으면 0.0으로 채움
- tag가 있는 point는 우선 보존
- 원본 index가 사라지는 경우 가장 가까운 source index를 보관

### 4. Resampling

Pure pursuit와 validator가 다루기 쉽도록 arc length 기준으로 일정 간격을 만든다.

권장 기본값:

- `resampling.interval_m: 0.5`
- 저속/주차 trajectory: `0.2` - `0.3 m`
- 고속/긴 직선 trajectory: `0.5` - `1.0 m`

Resampling 후에는 원본 index와 tag를 가능한 한 유지한다. Stop point, gear change, segment boundary는 resampling 중 사라지면 안 된다.

### 5. Yaw calculation

`yaw_rad`가 비어 있으면 다음 point 방향으로 계산한다.

```text
yaw[i] = atan2(y[i+1] - y[i], x[i+1] - x[i])
```

마지막 point는 이전 yaw를 복사한다. 급격한 yaw jump는 unwrap 후 smoothing한다. 후진 segment는 pose yaw와 진행 방향을 분리해 다룰 수 있어야 한다. 초기 버전에서는 후진 segment에 `gear: reverse`를 표시하고, controller가 reverse mode를 지원하기 전까지 validator에서 제한한다.

### 6. Curvature calculation

세 점 또는 local window를 이용해 curvature `kappa`를 계산한다. 이 값은 메시지 필드는 아니지만 다음 값을 채우는 데 필요하다.

```text
heading_rate_rps = longitudinal_velocity_mps * kappa
front_wheel_angle_rad = atan(wheelbase_m * kappa)
```

Ioniq5 예시 vehicle config 기준:

- `wheelbase_m: 3.0`
- `max_wheel_angle_deg: 40.0`
- `max_wheel_angle_rad ~= 0.698`
- `min_turning_radius_m: 5.87`

Validator에서는 steering angle에서 계산한 한계와 최소 회전반경에서 계산한 한계 중 더 보수적인 값을 쓰는 것이 좋다.

### 7. Velocity profile

속도는 trajectory tuning의 핵심이다. Generator는 최소한 아래 profile mode를 지원하도록 설계한다.

| Mode | Description | Use case |
|---|---|---|
| `constant` | 모든 point에 동일 목표 속도 | pure pursuit 기본 튜닝 |
| `recorded` | 기록 당시 속도 사용 | 실제 주행 재현 |
| `curvature_limited` | 곡률이 큰 곳에서 감속 | S자/직각 코스 안정화 |
| `segment_override` | tag/segment별 속도 지정 | stop line, parking, slow zone |
| `isev_speed_weight` | ISEV `speed_weight_ranges`를 base speed에 곱함 | ISEV waypoint 튜닝 이식 |
| `csv` | `points.csv`의 `velocity_mps`를 그대로 사용 | 손 튜닝 |

Curvature-limited speed의 기본 생각:

```text
v_curve_max = sqrt(max_lateral_accel_mps2 / max(abs(kappa), epsilon))
velocity = min(base_velocity, v_curve_max)
```

ISEV `velocity_planner`는 active path curvature를 대표값으로 만들고, `kappa_low`/`kappa_high` 사이에서 speed command를 조절했다. 이번 generator에서는 그 아이디어를 point-wise velocity profile로 바꿔 trajectory에 직접 넣는다.

이후 acceleration/deceleration limit을 적용해 속도 profile을 앞뒤로 smooth한다. Autoware velocity smoother처럼 완전한 jerk 최적화까지는 초기 범위가 아니지만, 제어 튜닝용으로 급격한 속도 점프는 제거해야 한다.

### 8. Time and acceleration fill

`time_from_start`는 point 간 거리와 속도로 적분한다.

```text
dt[i] = ds[i] / max((v[i] + v[i+1]) / 2, min_velocity_for_time)
time_from_start[i+1] = time_from_start[i] + dt[i]
acceleration[i] = (v[i+1] - v[i]) / dt[i]
```

정지 구간은 `stop_duration_s`를 segment metadata로 표현한다. 한 point에서 시간이 무한히 늘어나는 형태보다, 같은 pose 주변에 hold point를 두거나 controller/validator가 이해하는 stop tag를 쓰는 편이 낫다.

## Runtime publish behavior

### Publish mode

| Mode | Behavior |
|---|---|
| `full` | 전체 trajectory를 매 주기 publish |
| `local_horizon` | ego와 가까운 point를 찾고 앞쪽 horizon만 publish |
| `replay_time` | 기록된 시간 흐름에 맞춰 moving window publish |
| `single_shot` | latched/static debug용으로 1회 publish |

초기 pure pursuit 튜닝에는 `local_horizon`을 권장한다. ISEV pure pursuit/path 흐름처럼 global asset에서 local window만 잘라 쓰면 controller가 매번 너무 긴 point list를 볼 필요가 없다.

### Local horizon

입력:

- ego odometry가 있으면 nearest point를 찾는다.
- odometry가 없으면 `start_index`부터 publish한다.

기본 parameter:

```yaml
publish:
  mode: local_horizon
  backward_length_m: 2.0
  forward_length_m: 80.0
  min_points: 20
  max_points: 300
```

`time_from_start`는 horizon 첫 point를 0으로 다시 맞춘다. 이렇게 하면 controller가 항상 현재 기준 future trajectory로 해석할 수 있다.

### Loop and end behavior

| End behavior | Description |
|---|---|
| `hold_last` | 마지막 valid trajectory를 계속 publish |
| `stop_at_end` | 끝점 근처 velocity를 0으로 만든 stop trajectory publish |
| `loop` | 처음으로 돌아가 반복 |
| `empty` | 빈 trajectory publish. controller 개발 초기에는 비권장 |

제어 안전 관점에서는 `stop_at_end`가 기본이어야 한다.

## ROS interface 초안

### Publishers

| Topic | Type | Note |
|---|---|---|
| `/planning/trajectory_raw` | Autoware-compatible `Trajectory` | main output |
| `/planning/trajectory_gen/debug/path` | `nav_msgs/Path` | RViz path 확인 |
| `/planning/trajectory_gen/debug/markers` | `visualization_msgs/MarkerArray` | 속도, tag, stop point, curvature 표시 |
| `/planning/trajectory_gen/status` | local status msg or `diagnostic_msgs/DiagnosticArray` | loaded asset, current index, source mode |
| `/planning/trajectory_gen/compat/path` | `nav_msgs/Path` | ISEV-style path/debug 호환용 optional output |
| `/planning/trajectory_gen/compat/waypoint_index` | `std_msgs/Int32` | source index 추적용 optional output |
| `/planning/trajectory_gen/compat/speed_weight` | `std_msgs/Float32` | ISEV speed weight debug용 optional output |

### Subscribers

| Topic | Type | Required | Note |
|---|---|---|---|
| `/localization/odom` | `nav_msgs/Odometry` | local horizon mode에서 필요 | 현재 pose/velocity |
| `/trajectory_gen/command` | local command msg | optional | pause/resume/switch/reload |
| `/gps` | `morai_msgs/GPSMessage` | record mode에서만 | GPS path recording |
| `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | optional | MORAI GT 기반 recording/debug |
| `/localization/old_odom` | `nav_msgs/Odometry` | optional | ISEV-style UTM alignment import 실험용 |
| `/localization/fused_odom` | `nav_msgs/Odometry` | optional | ISEV-style UTM alignment import 실험용 |

### Services 후보

| Service | Purpose |
|---|---|
| `/trajectory_gen/load` | trajectory id/file 변경 |
| `/trajectory_gen/reload` | 파일 수정 후 runtime reload |
| `/trajectory_gen/save_recording` | record buffer를 asset으로 저장 |
| `/trajectory_gen/set_mode` | full/local/replay/pause 변경 |
| `/trajectory_gen/reset` | start index와 replay time 초기화 |
| `/trajectory_gen/export_isev_csv` | 현재 asset을 `INDEX,MAP_X,MAP_Y` 또는 `INDEX,UTM_X,UTM_Y` CSV로 export |

서비스는 구현 시 편의 기능이다. 초기 버전에서는 rosparam과 launch arg만으로 시작해도 된다.

## Parameter 초안

| Parameter | Default | Description |
|---|---:|---|
| `trajectory_id` | catalog default | load할 trajectory asset id |
| `asset_root` | `configs/control/trajectory_gen` | catalog와 trajectory asset root |
| `output_topic` | `/planning/trajectory_raw` | main output topic |
| `frame_id` | `map` | output frame |
| `planning_hz` | `10.0` | publish frequency |
| `publish_mode` | `local_horizon` | full/local_horizon/replay_time/single_shot |
| `resample_interval_m` | `0.5` | point 간격 |
| `forward_length_m` | `80.0` | local horizon 앞쪽 길이 |
| `backward_length_m` | `2.0` | local horizon 뒤쪽 길이 |
| `target_velocity_mps` | `5.0` | constant mode 기본 속도 |
| `max_acceleration_mps2` | `1.0` | 가속 제한 |
| `max_deceleration_mps2` | `-2.0` | 감속 제한 |
| `max_lateral_accel_mps2` | `2.0` | curvature speed 제한 |
| `wheelbase_m` | `3.0` | vehicle config와 맞출 값 |
| `max_front_wheel_angle_rad` | `0.698` | Ioniq5 40 deg 기준 |
| `end_behavior` | `stop_at_end` | trajectory 끝 동작 |
| `publish_debug` | `true` | debug path/marker publish |
| `publish_compat_path` | `false` | ISEV-style `nav_msgs/Path` 호환 출력 |
| `preserve_source_index` | `true` | source CSV index를 debug/status에 보존 |

## Editing workflow

### ISEV CSV import

1. ISEV repo의 `waypoint_pub/path/*.csv`에서 사용할 CSV를 고른다.
2. `source.csv`로 복사하고 원본 파일명, repo, package를 `trajectory.yaml`에 남긴다.
3. `INDEX,UTM_X,UTM_Y` 또는 `INDEX,MAP_X,MAP_Y` header를 확인한다.
4. UTM 입력이면 `coordinate.utm_to_map`을 import tool로 계산하거나 수동으로 기록한다.
5. ISEV `csv_pub_locksafe.yaml`에 있던 `gps_allowed_index_ranges`, `speed_weight_ranges`를 `segments.csv`로 옮긴다.
6. Generator가 `points.csv`를 만들고 yaw/curvature/speed/time을 채운다.
7. RViz에서 debug path와 source index marker를 확인한다.
8. Validator를 병렬로 켜서 곡률, 속도, steering limit을 확인한다.

### Recorded trajectory 만들기

1. MORAI 또는 실제 차량에서 localization/odom이 안정적인지 확인한다.
2. `trajectory_gen` record mode 또는 별도 recorder로 `/localization/odom`을 sampling한다.
3. Raw record를 `raw_points.csv`로 저장한다.
4. Resampling과 cleanup을 수행해 `points.csv`를 만든다.
5. `trajectory.yaml`에 source, frame, speed profile, vehicle parameter를 적는다.
6. RViz에서 debug path와 marker를 확인한다.
7. Validator를 통과하는지 확인한다.
8. 제어 튜닝 중 바꾼 속도/stop/slow 구간은 `segments.csv`나 `points.csv`로 남긴다.

### 수동 trajectory 만들기

1. `trajectory.yaml`에서 `source.type: parametric`, `interactive_curvature_builder`, `candidate_bank`, `manual_csv` 중 하나를 선택한다.
2. 직선/원호/S자 등 anchor 또는 segment parameter를 적는다.
3. Generator가 resampling, yaw, curvature, speed를 채우게 한다.
4. `points.csv`의 `tag`, `velocity_mps` 또는 `segments.csv`의 `target_velocity_mps`를 손으로 수정해 원하는 상황을 만든다.
5. Debug marker로 속도 변화와 stop point를 확인한다.

### 제어 튜닝 loop

1. 낮은 속도 constant profile로 pure pursuit lookahead를 맞춘다.
2. 곡률 구간에서 lateral error와 steering oscillation을 본다.
3. Curvature-limited speed profile을 켜고 PID를 맞춘다.
4. Stop/slow/parking segment를 추가한다.
5. 동일 trajectory id/version으로 결과를 비교한다.

## Self validation

Generator는 publish 전에 최소한의 자체 검사를 수행한다. 실패하면 publish하지 않거나, debug status를 error로 낸다.

- point 개수 >= `min_points`
- 모든 숫자 finite
- `time_from_start` 단조 증가
- point 간 거리 <= interval threshold
- velocity >= 0 for forward-only mode
- acceleration/deceleration limit 초과 여부
- steering angle limit 초과 여부
- frame_id 존재
- local horizon이 비어 있지 않음
- ISEV import일 경우 source index mapping이 비정상적으로 끊기지 않았는지 확인

정식 gate는 `trajectory_validator`가 담당한다. Generator self validation은 잘못된 asset을 빠르게 찾기 위한 개발 편의 기능이다.

## Debug output

RViz에서 최소한 아래가 보여야 한다.

- 전체 path line
- 현재 publish horizon line
- nearest point
- lookahead 후보 point
- 속도 profile 색상
- stop/slow/reverse segment marker
- invalid point marker
- source CSV index marker
- ISEV speed weight/gate range marker
- 현재 trajectory id, mode, current index text

## Naming rule

Trajectory asset은 재현성을 위해 이름에 목적과 버전을 넣는다.

```text
<map_or_scenario>_<purpose>_<variant>_<version>

isev_forward_utm_001
isev_reverse_utm_001
morai_straight_pid_001
morai_scurve_pure_pursuit_001
parking_t_sequence_001
recorded_lab_lap_20260628_001
```

파일을 직접 고쳤다면 `notes.md`에 변경 이유와 튜닝 결과를 짧게 남긴다.

## Open decisions

- ROS 1에서 Autoware-compatible `Trajectory` 메시지를 어느 package에 둘지 결정해야 한다.
- `/planning/trajectory_raw`와 `/planning/trajectory` naming을 최종 topic interface 문서와 맞춰야 한다.
- Reverse trajectory를 초기부터 지원할지, parking 전까지 forward-only로 제한할지 결정해야 한다.
- `Control` 메시지를 Autoware `autoware_control_msgs/Control` 형태로 맞출지, MORAI 중심 local command로 먼저 갈지 결정해야 한다.
- ISEV-style online UTM alignment를 runtime 기능으로 둘지, import tool 기능으로만 둘지 결정해야 한다.
