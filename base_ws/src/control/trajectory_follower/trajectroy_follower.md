# Trajectory Follower

`trajectory_follower`는 입력 trajectory를 추종해 MORAI 제어 명령을 만드는 ROS1 catkin
패키지이다.

```text
횡방향: Pure Pursuit
종방향: PI
최종 출력: /ctrl_cmd (morai_msgs/CtrlCmd)
```

`trajectory_generator`와는 독립된 패키지이며, 기본적으로
`/control/trajectory`와 `/Ego_topic`을 입력으로 사용한다.

## MORAI 제어 규격

대회용 Ego Ctrl Cmd는 다음 형식으로 고정한다.

- ROS topic: `/ctrl_cmd`
- 종방향 제어: `longlCmdType = 1`
- 종방향 명령: `accel`, `brake` (`0.0 ~ 1.0`)
- 횡방향 명령: `steering`
- type 1에서 사용하지 않는 `velocity`, `acceleration`: `0.0`

현재 레포의 `base_ws/src/morai/morai_msgs/msg/CtrlCmd.msg`도 다음 필드로 정의되어 있다.

```text
int32 longlCmdType
float64 accel
float64 brake
float64 steering
float64 velocity
float64 acceleration
```

`front_steer`, `rear_steer` 필드는 사용하지 않는다. UDP 변환은
`sensor_interface/morai_ctrl_cmd_udp_sender`가 담당하며, 여기서도 type 1을 강제한다.

## 기본 구조

Python 진입점이나 별도 `follower.py`는 두지 않는다. Pure Pursuit와 PI를 각각 C++ ROS
node로 구현하고 `pp_pi.launch`에서 함께 실행한다.

```text
base_ws/src/control/trajectory_follower/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── trajectory_follower.yaml
│   ├── purepursuit.yaml
│   └── longi_pi.yaml
├── launch/
│   └── pp_pi.launch
└── src/
    ├── purepursuit.cpp
    └── longi_pi.cpp
```

| 파일 | 역할 |
| --- | --- |
| `trajectory_follower.yaml` | 공통 topic, 주기, 차량 제원, 출력 제한, MORAI 출력, safety 설정 |
| `purepursuit.yaml` | lookahead, steering gain 설정 |
| `longi_pi.yaml` | PI gain, 적분 제한, command deadband 설정 |
| `purepursuit.cpp` | trajectory와 ego 상태로 steering 계산 |
| `longi_pi.cpp` | 목표/현재 속도로 accel/brake 계산 후 최종 `/ctrl_cmd` 발행 |
| `pp_pi.launch` | 공통/개별 YAML을 로드하고 두 C++ node 실행 |

`setup.py`, `scripts/`, Python module, `test/` 디렉터리는 만들지 않는다.

## Node 연결

```text
/control/trajectory ------------------------+
                                            |
/Ego_topic -> purepursuit -> /control/command/steering
                                            |
                                            v
/Ego_topic ----------------------------> longi_pi -> /ctrl_cmd
```

두 node가 각각 `/ctrl_cmd`를 발행하면 publisher 충돌이 생긴다. 따라서 출력 책임은 다음과
같이 고정한다.

- `purepursuit`: `std_msgs/Float64` 조향 명령을
  `/control/command/steering`으로 발행
- `longi_pi`: 조향 명령을 구독하고 PI의 accel/brake와 결합해
  `morai_msgs/CtrlCmd`를 `/ctrl_cmd`로 단독 발행

`/control/command/steering`은 두 제어 node를 연결하기 위한 내부 command topic이며 debug
topic이 아니다.

## Topic Interface

| Node | 방향 | Topic | Type |
| --- | --- | --- | --- |
| `purepursuit` | Subscribe | `/control/trajectory` | `autoware_planning_msgs/Trajectory` |
| `purepursuit` | Subscribe | `/Ego_topic` | `morai_msgs/EgoVehicleStatus` |
| `purepursuit` | Publish | `/control/command/steering` | `std_msgs/Float64` |
| `longi_pi` | Subscribe | `/control/trajectory` | `autoware_planning_msgs/Trajectory` |
| `longi_pi` | Subscribe | `/Ego_topic` | `morai_msgs/EgoVehicleStatus` |
| `longi_pi` | Subscribe | `/control/command/steering` | `std_msgs/Float64` |
| `longi_pi` | Publish | `/ctrl_cmd` | `morai_msgs/CtrlCmd` |

MVP의 ego 입력은 `/Ego_topic`으로 고정한다. `/localization/odom` 선택 기능과
`source_pose` 파라미터는 두지 않는다.

별도 custom debug message와 다음 debug topic도 만들지 않는다.

```text
/control/debug/follower/target_point
/control/debug/follower/path
/control/debug/follower/state
```

## Config

### `trajectory_follower.yaml`

두 node가 함께 사용하는 설정만 둔다.

```yaml
topics:
  trajectory: /control/trajectory
  ego: /Ego_topic
  steering_cmd: /control/command/steering
  ctrl_cmd: /ctrl_cmd

control_rate_hz: 30.0

vehicle:
  wheelbase_m: 3.0
  max_steer_rad: 0.698

control_limits:
  max_accel_cmd: 1.0
  max_brake_cmd: 1.0

morai:
  longlCmdType: 1

safety:
  trajectory_timeout_s: 0.5
  ego_timeout_s: 0.5
  steering_timeout_s: 0.5
  min_points: 2
  stop_brake_cmd: 1.0
```

### `purepursuit.yaml`

```yaml
min_lookahead_m: 4.0
lookahead_time_s: 1.5
max_lookahead_m: 20.0
steering_gain: 1.0
```

### `longi_pi.yaml`

```yaml
kp: 0.6
ki: 0.05
integral_limit: 3.0
command_deadband: 0.05
```

`longlCmdType`은 반드시 `1`이어야 한다. 다른 값이 설정되면 `longi_pi` node가 실행을
중단하도록 한다.

## Pure Pursuit

`purepursuit.cpp`는 trajectory에서 ego와 가장 가까운 point를 찾은 후 lookahead target을
선택한다.

```text
lookahead_m = min_lookahead_m + ego_speed_mps * lookahead_time_s
lookahead_m = clamp(lookahead_m, min_lookahead_m, max_lookahead_m)

curvature = 2.0 * target_y / (lookahead_m * lookahead_m)
steering = atan(wheelbase_m * curvature)
steering = clamp(steering * steering_gain, -max_steer_rad, max_steer_rad)
```

계산한 `steering`을 `/control/command/steering`으로 발행한다. trajectory 또는 ego 상태가
timeout되거나 유효한 target을 찾지 못하면 `0.0`을 발행한다.

## Longitudinal PI

`longi_pi.cpp`는 trajectory 목표 속도와 `/Ego_topic.velocity`로 계산한 현재 속도의 차이를
PI 입력으로 사용한다.

```text
error = target_velocity_mps - ego_velocity_mps
integral = clamp(integral + error * dt, -integral_limit, integral_limit)
u = kp * error + ki * integral
```

```text
if u > command_deadband:
  accel = clamp(u, 0.0, max_accel_cmd)
  brake = 0.0
elif u < -command_deadband:
  accel = 0.0
  brake = clamp(-u, 0.0, max_brake_cmd)
else:
  accel = 0.0
  brake = 0.0
```

필수 조건은 다음과 같다.

- `accel`과 `brake`를 동시에 사용하지 않는다.
- 출력은 `0.0 ~ 1.0`으로 제한한다.
- saturation 시 integral이 계속 증가하지 않도록 anti-windup을 적용한다.
- timeout 또는 safety stop 시 integral을 reset한다.

## 최종 CtrlCmd

정상 제어 시 `longi_pi.cpp`가 다음 메시지를 만든다.

```yaml
longlCmdType: 1
accel: <PI accel, 0.0 ~ 1.0>
brake: <PI brake, 0.0 ~ 1.0>
steering: <purepursuit steering>
velocity: 0.0
acceleration: 0.0
```

trajectory, ego 또는 steering command가 timeout되거나 입력값이 유효하지 않으면 다음 정지
명령을 주기적으로 발행한다.

```yaml
longlCmdType: 1
accel: 0.0
brake: 1.0
steering: 0.0
velocity: 0.0
acceleration: 0.0
```

## `pp_pi.launch` 초안

```xml
<launch>
  <arg name="common_config"
       default="$(find trajectory_follower)/config/trajectory_follower.yaml"/>
  <arg name="pp_config"
       default="$(find trajectory_follower)/config/purepursuit.yaml"/>
  <arg name="pi_config"
       default="$(find trajectory_follower)/config/longi_pi.yaml"/>

  <node pkg="trajectory_follower"
        type="purepursuit"
        name="purepursuit"
        output="screen">
    <rosparam command="load" file="$(arg common_config)"/>
    <rosparam command="load" file="$(arg pp_config)"/>
  </node>

  <node pkg="trajectory_follower"
        type="longi_pi"
        name="longi_pi"
        output="screen">
    <rosparam command="load" file="$(arg common_config)"/>
    <rosparam command="load" file="$(arg pi_config)"/>
  </node>
</launch>
```

## CMake 구성

`CMakeLists.txt`는 두 C++ executable만 빌드한다.

```cmake
add_executable(purepursuit src/purepursuit.cpp)
add_executable(longi_pi src/longi_pi.cpp)

add_dependencies(purepursuit ${catkin_EXPORTED_TARGETS})
add_dependencies(longi_pi ${catkin_EXPORTED_TARGETS})

target_link_libraries(purepursuit ${catkin_LIBRARIES})
target_link_libraries(longi_pi ${catkin_LIBRARIES})
```

필수 catkin 의존성은 `roscpp`, `std_msgs`, `morai_msgs`,
`autoware_planning_msgs`이다.

## 완료 기준

- [ ] `trajectory_follower`가 ROS1 catkin 패키지로 구성되어 있다.
- [ ] `pp_pi.launch`가 `purepursuit`와 `longi_pi` C++ node를 실행한다.
- [ ] Python node, `follower.py`, test 디렉터리가 없다.
- [ ] 공통, Pure Pursuit, PI 설정이 세 YAML로 분리되어 있다.
- [ ] ego 입력은 `/Ego_topic` 하나만 사용한다.
- [ ] `purepursuit`가 steering command를 발행한다.
- [ ] `longi_pi`만 최종 `/ctrl_cmd`를 발행한다.
- [ ] `/ctrl_cmd.longlCmdType`은 항상 `1`이다.
- [ ] `CtrlCmd.steering`, `accel`, `brake` 실제 필드를 사용한다.
- [ ] custom/debug message와 follower debug topic을 만들지 않는다.
- [ ] timeout 시 type 1 정지 명령을 주기적으로 발행한다.
