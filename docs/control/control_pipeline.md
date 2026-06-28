# Control Pipeline

이 문서는 AjouNice2026 제어 파이프라인의 초기 설계를 정리한다. 현재 목표는 planning 전체가 완성되기 전에도 Autoware 형태의 trajectory를 만들어 pure pursuit와 PID 제어기를 튜닝하고, 나중에 실제 planning 출력으로 자연스럽게 교체할 수 있게 하는 것이다.

## 목표

- Planning 출력과 개발용 trajectory generator 출력의 메시지 형태를 통일한다.
- 제어기는 입력이 실제 planning에서 왔는지, 개발용 generator에서 왔는지 몰라도 동작해야 한다.
- Validator는 trajectory가 제어기에 들어가기 전에 최소한의 형식, 물리 제약, 추종 가능성을 확인한다.
- MORAI 전용 제어 토픽은 마지막 adapter에서만 다룬다. Pure pursuit와 PID 내부는 가능한 한 simulator 독립적인 입력과 출력을 사용한다.

## 전체 흐름

초기 개발에서는 generator 출력을 제어기에 바로 넣고, validator는 병렬로 관찰하게 둘 수 있다. 제어 튜닝이 어느 정도 끝나면 validator를 gate로 세워 제어기가 validated trajectory만 받도록 바꾼다.

```text
Development direct mode

trajectory_gen
  -> /planning/trajectory_raw
  -> pure_pursuit + pid
  -> /control/command
  -> morai_control_adapter
  -> /ctrl_cmd
```

```text
Validated mode

trajectory_gen or planning stack
  -> /planning/trajectory_raw
  -> trajectory_validator
  -> /planning/trajectory
  -> pure_pursuit + pid
  -> /control/command
  -> morai_control_adapter
  -> /ctrl_cmd
```

## ISEV 참고 흐름과 이번 흐름

ISEV 레포의 planning/control 흐름은 `waypoint_pub`과 `pathplanner`를 중심으로 아래처럼 이어진다.

```text
waypoint_pub/csv_pub
  -> /planning/waypoint
  -> scene_builder
  -> /planning/scene
  -> path_selector
  -> /planning/candidate_paths
  -> /planning/selected_path_raw
  -> path_stitcher
  -> /planning/active_path
  -> velocity_planner + pure pursuit/stanley 계열 controller
```

이번 AjouNice2026 제어 파이프라인에서는 이 아이디어를 그대로 `nav_msgs/Path` 계약으로 가져오지 않는다. ISEV의 CSV waypoint, 직선/원호 path maker, candidate primitive bank, speed weight range는 `trajectory_gen`의 source/import 기능으로 흡수하고, 제어단으로 넘어가는 공식 계약은 Autoware-compatible `Trajectory`로 고정한다.

```text
ISEV CSV/recorded odom/parametric primitive
  -> trajectory_gen
  -> /planning/trajectory_raw
  -> trajectory_validator
  -> /planning/trajectory
  -> pure_pursuit + pid
```

이렇게 두면 초기에는 ISEV처럼 빠르게 임의 path를 만들 수 있고, 나중에 Autoware식 planning stack이 붙어도 controller input contract는 유지된다.

## 노드 책임

| Node | Responsibility | Main input | Main output |
|---|---|---|---|
| `trajectory_gen` | 개발용 trajectory 생성, 기록된 주행 경로 재생, 수동/파라메트릭 경로 생성 | trajectory asset, optional odometry | `/planning/trajectory_raw` |
| `trajectory_validator` | trajectory 형식, 연속성, 속도/가속도/조향/곡률 제약 검사 | `/planning/trajectory_raw`, ego odometry | `/planning/trajectory`, status/debug |
| `pure_pursuit_controller` | trajectory의 local horizon을 추종해 조향 명령 계산 | `/planning/trajectory`, ego odometry | lateral command |
| `pid_longitudinal_controller` | 목표 속도와 현재 속도 오차로 가감속 명령 계산 | target velocity, ego velocity | longitudinal command |
| `control_command_mux` | lateral/longitudinal 명령을 내부 control command로 합성 | lateral, longitudinal | `/control/command` |
| `morai_control_adapter` | 내부 control command를 `morai_msgs/CtrlCmd`로 변환 | `/control/command` | `/ctrl_cmd` |

## 기준 메시지

Planning에서 control로 넘어오는 trajectory는 Autoware의 `autoware_planning_msgs/Trajectory`와 같은 의미 구조로 맞춘다.

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

ROS 1에서 직접 구현할 때도 필드 이름과 단위는 위 의미를 유지한다. `builtin_interfaces/Duration`은 ROS 1의 `duration`에 대응시키면 된다.

## 토픽 초안

| Topic | Type | Publisher | Subscriber | Note |
|---|---|---|---|---|
| `/planning/trajectory_raw` | Autoware-compatible `Trajectory` | `trajectory_gen` or planning | `trajectory_validator`, debug tools | 검증 전 trajectory |
| `/planning/trajectory` | Autoware-compatible `Trajectory` | `trajectory_validator` | controller | 제어기가 기본으로 구독할 validated trajectory |
| `/planning/trajectory_debug/path` | `nav_msgs/Path` | `trajectory_gen` or validator | RViz | pose 확인용 |
| `/planning/trajectory_debug/markers` | `visualization_msgs/MarkerArray` | `trajectory_gen` or validator | RViz | 속도/태그/검사 결과 시각화 |
| `/planning/trajectory_gen/compat/path` | `nav_msgs/Path` | `trajectory_gen` | RViz, legacy debug tools | ISEV-style path 호환 출력. 제어 공식 입력은 아님 |
| `/planning/trajectory_gen/compat/waypoint_index` | `std_msgs/Int32` | `trajectory_gen` | debug, validator optional | ISEV CSV source index 추적용 |
| `/control/command` | Autoware-compatible `Control` or local equivalent | controller | simulator/vehicle adapter | simulator 독립 내부 제어 명령 |
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` | MORAI adapter | MORAI bridge | MORAI 전용 최종 명령 |

## 제어기 입력 해석

Pure pursuit는 trajectory의 `pose`와 ego pose를 사용해 lookahead target을 찾는다. 속도는 longitudinal PID의 target으로 쓰고, 필요한 경우 lookahead distance를 속도에 따라 조정한다.

PID는 가장 가까운 trajectory point 또는 lookahead point의 `longitudinal_velocity_mps`를 목표 속도로 사용한다. `acceleration_mps2`는 feedforward로 사용할 수 있지만, 초기 버전에서는 PID 출력 안정화 후 별도 옵션으로 둔다.

`front_wheel_angle_rad`와 `heading_rate_rps`는 pure pursuit가 직접 쓰지 않아도 validator와 debug에 중요하다. Generator 단계에서 채워두면 곡률, 조향 한계, yaw rate 한계를 미리 확인할 수 있다.

## 개발 단계

1. `trajectory_gen`이 파일 기반 trajectory를 `/planning/trajectory_raw`로 publish한다.
2. Pure pursuit + PID가 `/planning/trajectory_raw` 또는 remap된 `/planning/trajectory`를 구독해 MORAI에서 주행한다.
3. `trajectory_validator`를 병렬로 켜서 현재 trajectory의 문제를 diagnostics와 marker로 본다.
4. Validator가 안정되면 controller input을 `/planning/trajectory`로 고정한다.
5. 실제 planning stack이 준비되면 `/planning/trajectory_raw` publisher만 교체한다.

## 관련 문서

- [trajectory_gen.md](trajectory_gen.md): 개발용 trajectory generator 상세 명세
- [trajectory_validator.md](trajectory_validator.md): trajectory validator 초안
