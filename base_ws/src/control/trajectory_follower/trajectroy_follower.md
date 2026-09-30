# Trajectory Follower 설계 요약

## 목적과 구조

`trajectory_follower`는 Pure Pursuit 횡제어와 PI 종제어로 MORAI type 1 제어 명령을 만든다.

```text
/control/trajectory + /localization/kinematic_state
  -> purepursuit -> /control/command/steering
  -> longi_pi    -> /ctrl_cmd
```

`purepursuit`는 조향만 publish하고 `longi_pi`만 최종 `/ctrl_cmd`를 publish한다.

## ROS 인터페이스

| Node | 방향 | Topic | Type |
| --- | --- | --- | --- |
| `purepursuit` | Sub | `/control/trajectory` | `autoware_planning_msgs/Trajectory` |
| `purepursuit` | Sub | `/localization/kinematic_state` | `nav_msgs/Odometry` |
| `purepursuit` | Pub | `/control/command/steering` | `std_msgs/Float64` |
| `longi_pi` | Sub | `/control/trajectory` | `autoware_planning_msgs/Trajectory` |
| `longi_pi` | Sub | `/localization/kinematic_state` | `nav_msgs/Odometry` |
| `longi_pi` | Sub | `/control/command/steering` | `std_msgs/Float64` |
| `longi_pi` | Pub | `/ctrl_cmd` | `morai_msgs/CtrlCmd` |

위치·자세·속도는 localization 공통 계약인 `/localization/kinematic_state`를 사용한다.
trajectory와 state의 `header.frame_id`가 다르면 조향 0과 완전 제동을 출력한다. MORAI 원시
`/Ego_topic` 좌표를 trajectory에 직접 대입하지 않는다.

## Pure Pursuit

trajectory에서 Ego와 가장 가까운 point를 찾고 경로 누적 거리로 lookahead target을 고른다.

```text
lookahead = clamp(min_lookahead + speed * lookahead_time,
                  min_lookahead, max_lookahead)
curvature = 2 * target_y / lookahead^2
steering = clamp(gain * atan(wheelbase * curvature),
                 -max_steer, max_steer)
```

target이 차량 뒤에 있거나 입력이 유효하지 않으면 steering `0.0`을 publish한다.

## Longitudinal PI

```text
error = target_velocity - ego_velocity
integral = clamp(integral + error * dt, -integral_limit, integral_limit)
u = kp * error + ki * integral
```

- `u > deadband`: accel만 사용
- `u < -deadband`: brake만 사용
- 그 외: accel과 brake 모두 0
- 감속 오차가 시작될 때 남아 있는 가속 방향 적분만 reset
- brake mode 진입/해제 속도 오차에 hysteresis를 적용하고, 계획 궤적이 계속 감속 중이면
  짧은 오차 반전 동안 최소 brake를 유지하며 accel 전환을 금지
- 큰 양의 속도 오차에 의한 강제 해제는 설정 시간 이상 지속될 때만 적용
- saturation 방향으로 integral이 증가하지 않도록 conditional anti-windup 적용

최종 MORAI 명령은 다음 계약을 지킨다.

```yaml
longlCmdType: 1
accel: 0.0..1.0
brake: 0.0..1.0
steering: -max_steer_rad..max_steer_rad
velocity: 0.0
acceleration: 0.0
```

## Safety

trajectory, kinematic state, steering 중 하나라도 timeout되거나 값이 유효하지 않으면 주기적으로 다음
정지 명령을 publish하고 PI integral을 reset한다.

```yaml
longlCmdType: 1
accel: 0.0
brake: 1.0
steering: 0.0
velocity: 0.0
acceleration: 0.0
```

trajectory의 가장 가까운 point가 마지막 point이면 동일하게 정지한다. 다른 controller와
동시에 `/ctrl_cmd`를 publish하면 안 된다.

## 설정

- `trajectory_follower.yaml`: topic, 주기, wheelbase, 출력 제한, timeout
- `purepursuit.yaml`: lookahead와 steering gain
- `longi_pi.yaml`: `kp`, `ki`, integral limit, deadband

`morai.longlCmdType`이 `1`이 아니거나 파라미터가 유효하지 않으면 node가 시작하지 않는다.
실행 방법은 [README.md](README.md)를 참고한다.
