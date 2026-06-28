# Trajectory Validator

`trajectory_validator`는 planning 또는 `trajectory_gen`이 만든 trajectory를 제어기에 넣기 전에 검사하는 gate 노드다. Autoware의 planning validator/trajectory checker 개념을 AjouNice2026의 ROS 1 + MORAI 개발 흐름에 맞춰 단순화해 시작한다.

## 목표

- 제어기가 NaN, 빈 trajectory, 지나치게 큰 곡률, 비현실적인 속도 profile을 받지 않게 한다.
- Generator를 튜닝할 때 어떤 point/구간이 문제인지 RViz와 diagnostics로 빠르게 알 수 있게 한다.
- 초기에는 warn-only observer로 쓰고, 이후에는 `/planning/trajectory`를 publish하는 gate로 사용한다.

## 위치

```text
trajectory_gen or planning stack
  -> /planning/trajectory_raw
  -> trajectory_validator
  -> /planning/trajectory
  -> pure_pursuit + pid
```

개발 초기에는 controller가 `/planning/trajectory_raw`를 직접 구독하고 validator는 병렬 관찰만 해도 된다. 단, 최종 제어 파이프라인에서는 controller가 validated topic인 `/planning/trajectory`를 보도록 고정한다.

## ROS interface 초안

### Subscribers

| Topic | Type | Required | Note |
|---|---|---|---|
| `/planning/trajectory_raw` | Autoware-compatible `Trajectory` | yes | 검증 전 trajectory |
| `/localization/odom` | `nav_msgs/Odometry` | recommended | ego deviation, forward length 검사 |
| `/localization/accel` | `geometry_msgs/AccelWithCovarianceStamped` or local equivalent | optional | acceleration deviation 검사 |

### Publishers

| Topic | Type | Note |
|---|---|---|
| `/planning/trajectory` | Autoware-compatible `Trajectory` | 통과 또는 fallback 처리된 trajectory |
| `/planning/trajectory_validator/status` | local status msg or diagnostics | 전체 verdict와 실패 reason |
| `/planning/trajectory_validator/debug/markers` | `visualization_msgs/MarkerArray` | invalid point, threshold 초과 구간 표시 |
| `/planning/trajectory_validator/debug/path` | `nav_msgs/Path` | validator output path 확인 |

## 검사 항목

### Basic data checks

- point 개수가 `min_points` 이상인가
- `header.frame_id`가 비어 있지 않은가
- 모든 numeric field가 finite인가
- `time_from_start`가 단조 증가하는가
- point 간 거리가 너무 작거나 크지 않은가
- pose orientation이 normalize 가능한 quaternion인가

### Geometry checks

- yaw 변화가 비정상적으로 크지 않은가
- relative angle이 threshold 이하인가
- curvature가 vehicle kinematics로 추종 가능한 범위인가
- self-intersection 또는 급격한 path jump가 없는가
- ego와 nearest trajectory point 사이의 lateral/longitudinal deviation이 허용 범위인가

### Velocity and dynamics checks

- velocity가 음수/비정상 값이 아닌가
- longitudinal acceleration이 상한/하한 안에 있는가
- lateral acceleration `v^2 * curvature`가 threshold 이하인가
- lateral jerk 추정값이 threshold 이하인가
- forward trajectory length가 현재 속도에서 정지하기에 충분한가

### Steering checks

- curvature에서 추정한 front wheel angle이 vehicle limit 이하인가
- front wheel angle field가 직접 들어온 경우 vehicle limit 이하인가
- steering rate 추정값이 threshold 이하인가

### Temporal checks

- trajectory stamp가 너무 오래되지 않았는가
- publish 주기가 기대 범위 안에 있는가
- 이전 trajectory 대비 nearest point 주변 shift가 너무 크지 않은가

### ISEV import/compat checks

ISEV CSV waypoint나 `pathplanner` 계열 source를 `trajectory_gen`으로 가져오는 경우에는 아래 값도 debug 차원에서 확인한다. 이것들은 Autoware `Trajectory` 필드는 아니므로 validator의 필수 입력은 아니지만, source 문제를 찾는 데 유용하다.

- `source_index`가 단조롭게 증가하거나, `reverse_path`일 때 단조롭게 감소하는가
- ISEV `gps_allowed_index_ranges`에서 gate가 닫힌 구간을 publish하고 있지 않은가
- ISEV `speed_weight_ranges`가 segment 속도에 의도대로 반영됐는가
- import 전 `nav_msgs/Path` 길이와 변환 후 trajectory 길이가 크게 달라지지 않았는가
- candidate source를 쓸 경우 `collision=false`였던 후보를 가져왔는가
- candidate source의 `score_total`, `inside_lane`, `ade/fde` metadata가 debug report에 남아 있는가

## Handling policy

Autoware planning validator처럼 invalid trajectory를 발견했을 때 처리 방식을 parameter로 고른다.

| Policy | Behavior | Use case |
|---|---|---|
| `warn_only` | 원본을 그대로 publish하고 status만 warn | 개발 초기 |
| `drop` | invalid면 publish하지 않음 | controller가 timeout stop을 잘 처리할 때 |
| `hold_last_valid` | 마지막 valid trajectory publish | 안정적인 튜닝 |
| `soft_stop` | 현재 trajectory를 감속/정지 trajectory로 대체 | 안전 기본값 후보 |
| `empty` | 빈 trajectory publish | 일반 주행에서는 비권장 |

초기 기본값은 `warn_only`가 좋다. 제어기가 안정되면 `hold_last_valid` 또는 `soft_stop`으로 바꾼다.

## Parameter 초안

| Parameter | Default | Description |
|---|---:|---|
| `validation_hz` | `10.0` | 검사 주기 |
| `handling_policy` | `warn_only` | invalid trajectory 처리 |
| `min_points` | `10` | 최소 point 개수 |
| `max_point_interval_m` | `5.0` | 연속 point 최대 거리 |
| `min_point_interval_m` | `0.01` | 중복 point 판단 거리 |
| `max_curvature` | `0.17` | Ioniq5 최소 회전반경 5.87 m 기준 시작값 |
| `max_lateral_accel_mps2` | `2.0` | 제어 튜닝용 횡가속 제한 |
| `max_lateral_jerk_mps3` | `5.0` | 승차감/안정성 제한 |
| `max_longitudinal_accel_mps2` | `1.5` | 가속 제한 |
| `min_longitudinal_accel_mps2` | `-3.0` | 감속 제한 |
| `max_front_wheel_angle_rad` | `0.698` | Ioniq5 40 deg 기준 |
| `max_steering_rate_radps` | `0.5` | 초기 보수값 |
| `max_yaw_deviation_rad` | `1.57` | ego yaw와 nearest trajectory yaw 차이 |
| `max_lateral_deviation_m` | `3.0` | ego와 trajectory 사이 lateral deviation |
| `max_stamp_age_s` | `0.5` | stale trajectory 판단 |
| `forward_length_margin_m` | `2.0` | 정지 가능 길이 여유 |

값은 시작점이다. MORAI에서 pure pursuit와 PID 튜닝 결과를 보며 조정한다.

## Debug와 diagnostics

Validator는 실패 여부만 말하면 부족하다. 어떤 검사에서, 몇 번째 point가, 얼마만큼 초과했는지가 보여야 한다.

권장 status:

```text
level: OK/WARN/ERROR
trajectory_id
point_count
nearest_index
failed_checks:
  - check_name
  - point_index
  - measured_value
  - threshold
  - handling_policy
```

RViz marker:

- invalid point를 빨간 sphere로 표시
- curvature/steering 초과 구간을 선 색상으로 표시
- stop/soft-stop fallback 구간 표시
- nearest point와 ego deviation 표시

## 개발 순서

1. Basic data checks만 구현해 generator asset 오류를 잡는다.
2. Geometry checks를 추가해 pure pursuit가 흔들리는 구간을 찾는다.
3. Velocity/dynamics checks를 추가해 PID 튜닝 중 속도 profile 문제를 잡는다.
4. Handling policy를 `warn_only`에서 `hold_last_valid`/`soft_stop`으로 바꾼다.
5. 실제 planning stack이 붙으면 `/planning/trajectory_raw` 입력을 generator 대신 planning output으로 바꾼다.

## Generator와의 관계

`trajectory_gen`도 publish 전 self validation을 하지만, 그것은 asset 편집 실수를 빠르게 찾기 위한 것이다. `trajectory_validator`는 controller 앞의 공식 gate다.

Generator에서 반드시 채워야 validator가 잘 동작하는 값:

- `pose`
- `longitudinal_velocity_mps`
- `acceleration_mps2`
- `time_from_start`
- `heading_rate_rps`
- `front_wheel_angle_rad`

비어 있어도 validator가 계산할 수 있는 값:

- curvature
- yaw from pose sequence
- steering from curvature
- lateral acceleration

하지만 계산 가능한 값도 generator가 채워 주면 debug와 비교가 쉬워진다.
