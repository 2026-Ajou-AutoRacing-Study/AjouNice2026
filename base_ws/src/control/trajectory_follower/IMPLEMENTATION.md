# Trajectory Follower Implementation

`trajectory_follower`는 독립 실행되는 제어기이다. trajectory를 생성하지 않고,
입력 topic으로 들어온 `autoware_planning_msgs/Trajectory`를 추종해 MORAI
`/ctrl_cmd`를 publish한다.

MVP 목표 제어기는 아래 조합이다.

```text
lateral controller: Pure Pursuit
longitudinal controller: PI
output: morai_msgs/CtrlCmd
```

향후 Stanley, PID, MPC 등 다른 제어기를 추가할 수 있으므로 제어기 종류와 내부
파라미터는 YAML config에서 선택/수정한다.

## 기본 구조

```text
trajectory_follower.yaml
  -> trajectory_follower.launch
  -> scripts/trajectory_follower_node.py

/control/trajectory
  -> trajectory_follower_node.py

/Ego_topic 또는 /localization/odom
  -> trajectory_follower_node.py

trajectory_follower_node.py
  -> Pure Pursuit steering
  -> PI velocity control
  -> morai_msgs/CtrlCmd
  -> /ctrl_cmd
  -> ajou_control_msgs/TrajectoryFollowerDebug
  -> /control/debug/follower/state
```

follower는 editor에 종속되지 않는다. `/control/trajectory` 대신 다른 topic에서
동일한 `autoware_planning_msgs/Trajectory`가 들어와도 동작해야 한다.

## YAML Config

파라미터는 YAML에서 관리한다. 사용자는 편집기로 YAML 값을 바꾸고 저장한 뒤 launch를
다시 실행해 튜닝값을 적용한다. MVP에서는 runtime hot reload를 지원하지 않는다.

예시:

```yaml
topics:
  trajectory: /control/trajectory
  ego: /Ego_topic
  odom: /localization/odom
  ctrl_cmd: /ctrl_cmd
  debug_target_point: /control/debug/follower/target_point
  debug_path: /control/debug/follower/path
  debug_state: /control/debug/follower/state

vehicle:
  wheelbase_m: 3.0
  max_steer_rad: 0.698

controller:
  control_rate_hz: 30.0
  source_pose: ego_topic

  lateral:
    type: pure_pursuit
    min_lookahead_m: 4.0
    lookahead_time_s: 1.5
    max_lookahead_m: 20.0
    steering_gain: 1.0

  longitudinal:
    type: pi
    kp: 0.6
    ki: 0.05
    integral_limit: 3.0
    target_velocity_source: trajectory
    max_accel_cmd: 1.0
    max_brake_cmd: 1.0
    velocity_deadband_mps: 0.1

morai:
  longlCmdType: 2
  use_velocity_command: true
  use_accel_brake_command: false

safety:
  trajectory_timeout_s: 0.5
  ego_timeout_s: 0.5
  min_points: 2
  stop_on_error: true
```

## 입력

### Trajectory

기본 입력 topic:

```text
/control/trajectory
```

Type:

```text
autoware_planning_msgs/Trajectory
```

follower가 사용하는 필드:

| Field | 용도 |
| --- | --- |
| `header.frame_id` | trajectory frame |
| `points[].pose.position` | path point |
| `points[].pose.orientation` | 진행 방향 확인 또는 fallback yaw |
| `points[].longitudinal_velocity_mps` | target velocity |

### Ego state

MVP에서는 MORAI의 `/Ego_topic`을 우선 사용한다.

```text
/Ego_topic: morai_msgs/EgoVehicleStatus
```

사용 가능하면 `/localization/odom`을 pose/twist 기준으로 선택할 수도 있다.

```text
/localization/odom: nav_msgs/Odometry
```

YAML의 `controller.source_pose`로 어떤 입력을 기준으로 쓸지 정한다.

## 제어 루프

`trajectory_follower_node.py`는 `control_rate_hz` 주기로 아래 순서를 반복한다.

```text
1. 최신 trajectory 확인
2. 최신 ego state 확인
3. timeout / point 개수 검사
4. ego와 가장 가까운 trajectory point 탐색
5. lookahead target point 선택
6. Pure Pursuit로 steering 계산
7. trajectory target velocity 계산
8. PI로 accel/brake 또는 velocity command 계산
9. morai_msgs/CtrlCmd 생성
10. /ctrl_cmd publish
11. debug topic publish
```

## Lateral: Pure Pursuit

Pure Pursuit는 ego 위치에서 일정 lookahead 거리 앞의 target point를 보고 조향각을
계산한다.

### Target point 선택

```text
lookahead_m = min_lookahead_m + ego_speed_mps * lookahead_time_s
lookahead_m = clamp(lookahead_m, min_lookahead_m, max_lookahead_m)
```

1. ego와 가장 가까운 trajectory index를 찾는다.
2. 해당 index부터 누적 거리로 `lookahead_m` 이상 떨어진 point를 target으로 고른다.
3. target point를 ego 좌표계로 변환한다.

### Steering 계산

```text
curvature = 2.0 * target_y / (lookahead_m * lookahead_m)
steer_rad = atan(wheelbase_m * curvature)
steer_rad = steer_rad * steering_gain
steer_rad = clamp(steer_rad, -max_steer_rad, max_steer_rad)
```

MORAI 출력에서는 `front_steer = steer_rad`, `rear_steer = 0.0`으로 둔다.

## Longitudinal: PI

PI 제어기는 trajectory의 target velocity와 현재 ego velocity 차이를 줄인다.

```text
error = target_velocity_mps - ego_velocity_mps
integral = clamp(integral + error * dt, -integral_limit, integral_limit)
u = kp * error + ki * integral
```

출력 정책은 두 가지를 지원할 수 있게 둔다.

### MVP 기본: velocity command

MORAI 테스트에서 확인된 방식처럼 `longlCmdType = 2`와 `velocity`를 사용한다.

```text
longlCmdType = 2
velocity = target_velocity_mps
acceleration = 0.0
accel = 0.0
brake = 0.0
```

이 경우 PI 값은 debug 또는 향후 accel/brake mode 튜닝에 사용하고, 실제 가감속은 MORAI
velocity command에 맡긴다.

### 확장: accel/brake command

`use_accel_brake_command`를 켜면 PI 출력 `u`를 accel/brake로 나눈다.

```text
if u >= velocity_deadband_mps:
  accel = clamp(u, 0.0, max_accel_cmd)
  brake = 0.0
elif u <= -velocity_deadband_mps:
  accel = 0.0
  brake = clamp(-u, 0.0, max_brake_cmd)
else:
  accel = 0.0
  brake = 0.0
```

초기 실차/시뮬레이터 검증 전까지는 velocity command mode를 기본으로 둔다.

## MORAI CtrlCmd

Publish topic:

```text
/ctrl_cmd
```

Type:

```text
morai_msgs/CtrlCmd
```

MVP 출력:

```text
longlCmdType: 2
accel: 0.0
brake: 0.0
front_steer: <pure_pursuit_steer_rad>
rear_steer: 0.0
velocity: <target_velocity_mps>
acceleration: 0.0
```

## Debug State

validator가 나중에 제어 결과를 비교하기 쉽도록 follower는 내부 계산값을 debug topic으로
publish한다.

Topic:

```text
/control/debug/follower/state
```

Type:

```text
ajou_control_msgs/TrajectoryFollowerDebug
```

저장할 값:

| Field | 의미 |
| --- | --- |
| `nearest_index` | ego에서 가장 가까운 trajectory point index |
| `target_index` | lookahead target point index |
| `lookahead_m` | Pure Pursuit lookahead 거리 |
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
| `is_valid` | 이번 control step의 debug 값 유효 여부 |

MVP 최종 기준은 전용 custom msg를 쓰는 것이다. 임시 구현 단계에서만
`diagnostic_msgs/DiagnosticArray` 또는 `std_msgs/Float64MultiArray`를 사용할 수 있지만,
validator/rqt_multiplot/replay까지 연결하려면 `TrajectoryFollowerDebug.msg`로 고정하는 것이 좋다.

메시지 후보:

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

이 topic은 follower 자체 디버깅에도 쓰고, validator가 `TrajectoryValidationSample.msg`를
만들 때 입력으로 사용한다.

## Safety Policy

아래 상황에서는 stop command를 publish한다.

| 상황 | 동작 |
| --- | --- |
| trajectory timeout | velocity 0, accel 0, brake optional, steer 0 |
| ego timeout | velocity 0, accel 0, brake optional, steer 0 |
| trajectory point 부족 | stop command |
| target point 선택 실패 | stop command |
| NaN/inf 값 발견 | stop command |

stop command MVP:

```text
longlCmdType: 2
velocity: 0.0
front_steer: 0.0
rear_steer: 0.0
accel: 0.0
brake: 1.0
acceleration: 0.0
```

## 구현 파일 구성

```text
trajectory_follower/
  config/
    trajectory_follower.yaml
  launch/
    trajectory_follower.launch
  scripts/
    trajectory_follower_node.py
```

MVP에서는 제어 로직을 `trajectory_follower_node.py` 하나에 함수 또는 class 단위로 둔다.
제어기 종류가 늘어나면 나중에 lateral/longitudinal controller 모듈로 분리한다.

## Launch 초안

```xml
<launch>
  <arg name="config_file" default="$(find trajectory_follower)/config/trajectory_follower.yaml"/>

  <rosparam command="load" file="$(arg config_file)"/>

  <node pkg="trajectory_follower"
        type="trajectory_follower_node.py"
        name="trajectory_follower"
        output="screen"/>
</launch>
```

## 완료 기준

- [ ] YAML config로 topic, vehicle, controller, safety 파라미터를 읽는다.
- [ ] `/control/trajectory`를 subscribe한다.
- [ ] `/Ego_topic` 또는 `/localization/odom`에서 ego pose/velocity를 얻는다.
- [ ] Pure Pursuit로 `front_steer`를 계산한다.
- [ ] PI 구조로 velocity error를 계산하고 target velocity 또는 accel/brake 명령을 만든다.
- [ ] `/ctrl_cmd`에 `morai_msgs/CtrlCmd`를 publish한다.
- [ ] validator 기록용 `ajou_control_msgs/TrajectoryFollowerDebug`를 `/control/debug/follower/state`로 publish한다.
- [ ] trajectory/ego timeout 시 stop command를 publish한다.
- [ ] YAML 값을 바꾼 뒤 launch 재실행으로 튜닝값이 적용된다.
