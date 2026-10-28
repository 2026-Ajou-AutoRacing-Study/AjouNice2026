# trajectory_follower

속도 단위의 전체 계약과 MORAI GT 입력 주의사항은
[`docs/control_velocity_units.md`](../../../../docs/control_velocity_units.md)를 참고한다.

## 실행 준비

```bash
cd /home/jihun/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps trajectory_follower
source devel/setup.bash
```

`--only-pkg-with-deps`는 package whitelist를 유지한다. 전체 workspace를 다시 구성할 때는
`catkin_make -DCATKIN_WHITELIST_PACKAGES=""`로 해제한다.

실행 전 다음 입력이 실제로 publish되고 있어야 한다.

```bash
rostopic hz /control/trajectory
rostopic hz /localization/kinematic_state
```

- `/control/trajectory`: `trajectory_generator` 또는 호환 planner가 제공한다.
- `/localization/kinematic_state`: 선택한 localization provider가 `map` 기준
  `base_link` pose와 twist를 제공한다. MORAI GT에서는 `localization_morai_gt` 또는
  `maptf_morai_deg_correction.launch`를 실행한다.

현재 MORAI GT를 포함한 `/localization/kinematic_state` 입력은 m/s이므로 공통 설정에서
`kinematic_velocity_scale_to_mps: 1.0`을 적용한다. Trajectory 목표 속도와 제어기 내부
계산도 m/s를 사용한다. 단위 계약이 다른 localization provider를 연결할 때만 실제 출력
단위를 측정한 뒤 이 scale을 변경한다.

## 사용법

### 1. Follower 실행

```bash
roslaunch trajectory_follower pp_pi.launch
```

등록된 단축 명령은 다음과 같다.

```bash
traj_fol
```

Autoware Pure Pursuit 구현은 같은 controller에서 LD 결정 방식만 바꾼다. 모드와
고정 LD는 항상 `config/purepursuit_autoware.yaml`의 `lookahead` 값으로 결정한다.
launch 인자로 덮어쓰지 않는다.

```bash
# config/purepursuit_autoware.yaml 편집 예시
lookahead:
  mode: fixed                 # 또는 adaptive
  fixed_distance_m: 4.0       # mode: fixed일 때 사용

# YAML을 저장한 뒤 실행
roslaunch trajectory_follower pp_pi_autoware.launch
```

Localization continuity A/B에서는 planning/TF는 그대로 둔 채 두 controller가 같은
후보 state를 보도록 다음 인자만 바꿀 수 있다. 기본값은 기존 Global state다.

```bash
roslaunch trajectory_follower pp_pi_autoware.launch \
  kinematic_state_topic:=/localization/control_kinematic_state
```

두 모드는 `path_processing/resampling_ds_m`과
`path_processing/curvature_calculation_distance_m`을 공유한다. 따라서 같은 trajectory와
차량 상태에서는 `path_curvature`가 같고 LD 선택 이후의 target/steering 계산도 동일하다.

튜닝 파일을 바꾸어 실행할 수도 있다.

```bash
roslaunch trajectory_follower pp_pi.launch \
  common_config:=$(rospack find trajectory_follower)/config/trajectory_follower.yaml \
  pp_config:=$(rospack find trajectory_follower)/config/purepursuit.yaml \
  pi_config:=$(rospack find trajectory_follower)/config/longi_pi.yaml
```

### 2. MORAI UDP 제어 연결

`/ctrl_cmd`를 MORAI로 보내려면 UDP interface의 control sender가 함께 실행되어야 한다.
이미 실행 중인 UDP interface가 있다면 중복 실행하지 않는다.

```bash
roslaunch sensor_interface morai_udp_interface.launch \
  enable_sensors:=false \
  enable_lidar:=false \
  enable_ego_network:=true \
  enable_control:=true
```

### 3. 동작 확인

```bash
rostopic hz /control/command/steering
rostopic hz /ctrl_cmd
rostopic echo -n 1 /ctrl_cmd
rostopic info /ctrl_cmd
```

정상 출력은 `longlCmdType: 1`이며 `accel`, `brake`, `steering`을 사용한다.
`/ctrl_cmd` publisher는 `longi_pi` 하나만 있어야 한다.

## 기능과 역할

### Spawn 위치 점프 시 1초 제동

Autoware 종방향 제어기는 플래너의 전역 측위 토픽
`/localization/kinematic_state`를 별도로 감시한다. 같은 좌표계의 연속된 최신
메시지 사이 XY 거리가 7 m 이상이고 정상 이동 상한도 초과하면 즉시 기존 safety stop 명령
(`accel=0`, 기본 `brake=1`, `steering=0`)을 내고 실제 경과 시간 1초 동안 유지한다.
브레이크 증가율 제한보다 우선하며 PI 적분·가속 FF·가속도 추정 상태를 초기화한다.
유지 중 다시 점프하면 마지막 점프부터 1초로 연장한다. 첫 위치, 오래된 메시지,
중복·역순 메시지, 좌표계 변경은 위치 점프로 처리하지 않는다.
이는 Spawn 여부를 직접 확인하지 않는 위치 불연속 감지이므로 측위 리셋에도 적용된다.
1초 후 기존 입력 유효성 검사를 통과하면 정상 제어로 돌아오며, 정지 완료 자체를
보장하는 조건은 아니다. 제어기가 Smooth 측위를 써도 감지는 전역 측위로 한다.
설정은 `longi_pi_autoware.yaml`의 `safety.pose_jump`
(`enabled`, `state_topic`, `distance_m`, `brake_duration_s`)이며 제어기 재시작 후 적용된다.

정상 이동 상한은 `max(이전 속력, 현재 속력) × 메시지 시각 차이 +
0.5 × max_acceleration_mps2 × 시각 차이² + motion_tolerance_m`다.
기본 가속도 여유는 10 m/s², 위치 여유는 1 m다. 이 값은 차량 제어의 가속도
한계를 바꾸지 않는다. 예를 들어 80 km/h에서 0.35초 수신 공백 뒤의 약 7.8 m
이동은 정상 범위로 통과한다. 장시간 입력 공백은 기존 freshness watchdog이 처리한다.

- `purepursuit`: trajectory와 공통 kinematic state로 조향각을 계산해
  `/control/command/steering`으로 publish한다.
- `longi_pi`: 목표 속도와 kinematic state 속도로 accel/brake를 계산하고 조향 명령을 결합해
  `/ctrl_cmd`를 단독 publish한다.
- 입력 timeout, 잘못된 값, 마지막 point 도달 시 type 1 정지 명령을 출력한다.

| 구분 | Topic | Type |
| --- | --- | --- |
| 입력 | `/control/trajectory` | `autoware_planning_msgs/Trajectory` |
| 입력 | `/localization/kinematic_state` | `nav_msgs/Odometry` |
| 내부 출력 | `/control/command/steering` | `std_msgs/Float64` |
| 최종 출력 | `/ctrl_cmd` | `morai_msgs/CtrlCmd` |

튜닝은 `config/purepursuit.yaml`과 `config/longi_pi.yaml`, 공통 안전 설정은
`config/trajectory_follower.yaml`에서 변경한다. 세부 제어 규칙은
[trajectroy_follower.md](trajectroy_follower.md)를 참고한다.
