# Trajectory Follower 설계 지시서

## 1. 목적

`trajectory_follower`는 controller 공통 계약과 운영 규칙이다. 패키지 이름 규칙은 아래와 같지만, **현재 파이프라인에서 구현·실행·튜닝하는 제어기는 `purepursuit_pi` 하나뿐**이다.

```text
purepursuit_pi  = Pure Pursuit lateral + PI longitudinal  # current scope
mpc_pi          = MPC lateral + PI longitudinal           # package name reserved; deferred
```

대회장에 가져가는 `purepursuit_pi`는 C++로 구현한다. Python은 generator/validator와의 연결, topic 확인, 임시 safe controller에만 허용하며 PP 파라미터 튜닝 결과를 대신하지 않는다.

이 문서는 controller 구현의 기준이다. package 경계·공통 topic/frame·현재 GT-first 운영 순서는 [control_pipeline.md](control_pipeline.md)를 기준으로 하며, run 기록/비교 방법은 [trajectory_validator.md](trajectory_validator.md)를 따른다.

## 2. 공통 input/output 계약

| 방향 | Topic | Type | 계약 |
|---|---|---|---|
| 입력 | `/planning/trajectory/reference` | `autoware_planning_msgs/Trajectory` | generator가 만든 map-frame 기준 trajectory |
| 입력 | `/Ego_topic` 또는 `/localization/odom` | YAML toggle | `map` 기준 ego pose, heading, longitudinal/lateral velocity, timestamp |
| 입력 | `/tf` | TF | `map -> base_link` lookup. controller가 TF publisher가 되지 않음 |
| 출력 | `/ctrl_cmd` | `morai_msgs/CtrlCmd` | MORAI 제어 명령 |
| 출력 | `/control/purepursuit_pi/run_config` | latched `std_msgs/String` | controller가 시작 시 실제로 읽은 resolved config, run label, checksum의 UTF-8 YAML snapshot |
| 출력 | `/control/<controller>/debug/*` | std/visualization msg | target point, local horizon, error, solve time 등 |

초기 제어단은 MORAI에 직접 들어가야 하므로 controller가 `/ctrl_cmd`를 publish한다. 대회 규정집은 `Network Settings > Ego-0 > Cmd Control`의 Ego Ctrl Cmd와 `/Ctrl_cmd`의 `longi type 1(accel, brake)` 사용을 명시한다. ROS message field 이름은 `longlCmdType`이므로 대회용 controller는 `longlCmdType: 1`로 고정한다. `CtrlCmd.front_steer`는 front wheel steer angle rad이고, PI의 종 출력은 normalized pedal `accel`/`brake`로 변환한다. type 2 velocity, type 3 acceleration mode는 대회용 controller에서 사용하지 않는다.

```text
PI signed output u_long
  u_long > deadband : accel = clamp(accel_gain * u_long, 0, 1), brake = 0
  u_long < -deadband: accel = 0, brake = clamp(brake_gain * -u_long, 0, 1)
  otherwise         : accel = 0, brake = 0
```

### 2.1 ego-state source toggle

`purepursuit_pi`와 `trajectory_validator`는 `purepursuit_pi/config/pp_pi.yaml` 한 파일의 같은 `ego_state` section을 읽는다. 별도의 활성/비활성 설정이나 run별 tuning YAML은 만들지 않는다. route를 바꾸거나 follower가 TF를 새로 publish하는 방식으로 source를 바꾸지 않는다.

**현재 PP 구현과 tuning에서는 이 값이 반드시 `morai_gt`다.** 아래 localization 항목은 후속 전환 계약이며, FAST-LIO가 준비되기 전에는 launch argument로 바꾸지 않는다.

```yaml
ego_state:
  source: morai_gt             # morai_gt | localization
  map_frame: map
  base_frame: base_link
  morai_gt_topic: /Ego_topic
  localization_odom_topic: /localization/odom
  localization_message_type: nav_msgs/Odometry
  localization_pose_from_tf: true
  localization_expected_parent: camera_init
  localization_expected_child: body
  localization_required_tf: [map->camera_init, body->base_link]
  localization_velocity_source: odom_twist_or_pose_delta
```

- `morai_gt`: `/Ego_topic` callback의 timestamp에서 `lookupTransform(map, base_link)`를 사용해 pose/heading을 얻는다. velocity는 `EgoVehicleStatus.velocity`와 heading을 body frame으로 정리해 사용한다. legacy controller의 `velocity_scale`은 이전 MORAI 단위 보정값일 뿐이므로, raw value와 변환값을 validator에 함께 기록해 m/s 여부를 첫 run에서 검증한다.
- `localization`: FAST-LIO launch가 `/Odometry`를 remap한 **`/localization/odom` (`nav_msgs/Odometry`)**을 freshness source로 사용한다. 확인된 raw FAST-LIO frame은 `camera_init`(parent), `body`(child)이며 시작 시점 기준 local odometry다. 따라서 message pose를 map trajectory에 바로 대입하지 않고, 같은 timestamp의 `map -> base_link` TF로 pose/heading을 얻는다.
- FAST-LIO `twist`가 유효하면 body-frame longitudinal/lateral velocity로 변환해 쓰고, 초기화 중 zero/stale이면 map pose 차분을 ego heading으로 투영해 계산한다. 어떤 경로를 썼는지는 run metadata와 sample에 남긴다.
- `map -> camera_init`, `body -> base_link`는 localization stack의 책임이다. follower/validator는 TF publisher가 아니다. 기존 `fast_lio_odom_map_bridge.py`의 GT 초기정렬 `/localization/odom_map`은 MORAI 비교용이며, 대회용 source로 대체하지 않는다.
- TF가 없거나 source message가 stale이면 controller는 safe stop으로 간다. controller 자신은 TF를 publish하지 않는다.

### 2.2 단일 제어기 활성 규칙

현재는 `purepursuit_pi` 하나만 `/ctrl_cmd`를 publish한다.

- MORAI를 시작하기 전 `rostopic info /ctrl_cmd`로 PP node 하나만 publisher인지 확인한다.
- 임시 Python smoke controller와 C++ PP controller를 동시에 실행하지 않는다.
- 같은 `/localization/odom`에도 FAST-LIO와 GT `ego_to_odom` adapter가 동시에 publish하지 않도록 launch에서 하나만 선택한다.

## 3. C++ 설계 규칙

```text
purepursuit_pi/
  include/purepursuit_pi/
    controller.hpp
    trajectory_cache.hpp
    vehicle_model.hpp
    ego_state_provider.hpp
  src/
    controller_node.cpp
    pure_pursuit_lateral.cpp
    pi_longitudinal.cpp
    ego_state_provider.cpp
    command_guard.cpp
  config/
    vehicle.yaml
    pp_pi.yaml              # 사용자가 tuning 때 수정하는 유일한 YAML
  launch/
    pp_pi.launch            # controller만 실행
```

- subscriber callback은 최신 trajectory와 ego state를 lock-protected cache에 넣기만 한다.
- control timer가 고정 주기로 cache snapshot을 읽어 lateral/longitudinal command를 계산한다.
- callback 안에서 control 계산, file I/O, RViz marker 대량 생성은 하지 않는다.
- timestamp가 오래된 ego/trajectory, point 부족, nearest-point 재획득 실패가 나면 command guard가 safe-stop 상태를 publish하고 reason을 debug topic으로 남긴다.
- `pp_pi.yaml`은 `pp_pi.launch` 시작에 한 번만 읽으며 주행 중 다시 읽지 않는다. controller는 실제로 읽은 resolved config, `run_label`, SHA-256을 latched `/control/purepursuit_pi/run_config`으로 publish한다. validator는 이 snapshot을 `trajectory_validator/history/pp_pi/<run_id>/parameters.yaml`으로 복사한다. 주행 중 원본 YAML 파일을 수정해도 현재 run에는 적용되지 않는다.
- state machine은 최소 `WAIT_FOR_INPUT → READY → TRACKING → FINISHED/SAFE_STOP`을 갖는다.

## 4. 차량 모델과 PP 기본 파라미터 기준

새 `purepursuit_pi/config/vehicle.yaml`은 workspace의 `configs/vehicle/ioniq5.yaml`, `ioniq5_mesh_measurements.json`, URDF의 rear-axle `base_link` 정의를 기준으로 만든다. 이는 **2023 Hyundai IONIQ 5**의 제원이며, controller 내부 단위는 m, rad, s, m/s로 통일한다.

| 항목 | 값 | 사용 원칙 |
|---|---:|---|
| wheelbase | 3.000 m | Pure Pursuit bicycle model의 `L`. 기존 Python 예제의 2.8 m를 승계하지 않음 |
| length | 4.635 m | RViz/collision 및 finish 여유 확인 |
| body width / mirror width | 1.890 m / 2.134 m | 차체 판단과 시각화 폭을 혼동하지 않음 |
| nominal width | 1.892 m | 저장된 차량 제원의 공식 폭 값 |
| height | 1.600 m | vehicle visualization 기준 |
| wheel radius | 0.3705 m | rear-axle-centered `base_link` visual offset 기준 |
| front / rear overhang | 0.845 m / 0.790 m | bumper clearance 판단용 |
| min turning radius | 5.87 m | geometry sanity check용 |
| `max_wheel_angle_deg` | 40.0 deg | 저장된 vehicle config 값. MORAI `front_steer` saturation으로 바로 단정하지 않음 |

`base_link`는 rear axle center이고 x-forward, y-left, z-up이다. 따라서 PP의 nearest projection, lookahead, bicycle model은 이 기준점을 사용한다. `front_steer`는 MORAI가 받는 front wheel angle rad이므로, wheel-lock/inner-wheel 표기인 40 deg와 같은 뜻으로 가정하지 않는다. 첫 PP run의 command cap은 legacy tuning에 있던 **0.35 rad를 보수적 초기값**으로 쓰되, 조향 방향·응답·saturation을 확인한 뒤에만 바꾼다.

## 5. 제어 주기와 trajectory 해상도

control rate은 20~50 Hz다.

```text
tick travel [m] = v_max[km/h] / (3.6 * control_rate[Hz])
```

| v_max | 20 Hz | 30 Hz | 40 Hz | 50 Hz |
|---:|---:|---:|---:|---:|
| 30 km/h | 0.417 m | 0.278 m | 0.208 m | 0.167 m |
| 40 km/h | 0.556 m | 0.370 m | 0.278 m | 0.222 m |
| 50 km/h | 0.694 m | 0.463 m | 0.347 m | 0.278 m |
| 60 km/h | 0.833 m | 0.556 m | 0.417 m | 0.333 m |
| 80 km/h | 1.111 m | 0.741 m | 0.556 m | 0.444 m |
| 100 km/h | 1.389 m | 0.926 m | 0.694 m | 0.556 m |

첫 운용 가정은 아래와 같다.

- baseline은 **20 Hz**다. PP와 임시 Python pipeline이 이 주기에서 안정적으로 command를 내는 것을 먼저 확인한다.
- PP는 20 Hz 뒤 30/40/50 Hz를 단계적으로 측정한다. 각 단계에서 worst-case cycle time과 deadline miss가 control period 안에 드는지 validator artifact로 확인한다.
- trajectory point spacing은 control rate과 독립적으로 충분히 작게 둔다. 한 tick 이동거리보다 point spacing이 지나치게 크면 nearest projection/lookahead가 계단형이 된다.
- 실제 timer period, maximum cycle time, deadline miss 횟수는 `trajectory_validator`가 기록한다.

시내는 보통 0~40 km/h, 필요시 50 km/h, 고속주회로는 추월을 위해 최대 80 km/h를 운용 목표로 둔다. Lanelet 상 일반도로/고속주회로 limit 60/100 km/h는 profile 상한 metadata다. 최종 Hz는 localization update rate, MORAI command acceptance rate, 실제 controller cycle time을 측정한 뒤 확정한다.

## 6. `purepursuit_pi` — 대회 보험 제어기

이 패키지가 5단계에서 먼저 완성·튜닝할 controller다.

### 6.1 횡(Pure Pursuit)

- ego rear axle 기준 pose와 reference polyline의 nearest projection을 구한다.
- 첫 PP tuning은 run label의 `ld`가 그대로 뜻하는 fixed lookahead를 사용한다. 예: `pp_ld50_*`은 `L_d=5.0 m`이다. fixed lookahead가 안정화된 뒤에만 속도 기반 `L_d = clamp(L_min + gain * v, L_min, L_max)`로 전환하며, 이때 run label에는 `ldmin`, `ldg`, `ldmax`를 모두 기록한다.
- reference arc length 위의 lookahead target을 택한다. 단순 point index lookahead는 쓰지 않는다.
- bicycle model로 steering tire angle을 계산하고 steering angle/rate limit를 적용한다.
- debug: nearest point, lookahead point, local reference, raw/final steering, cross-track/heading error.

### 6.2 종(PI)

- trajectory point의 target velocity와 ego longitudinal velocity의 error를 사용한다.
- anti-windup, integral limit, acceleration/brake saturation, speed deadband를 명시적으로 둔다.
- trajectory acceleration을 optional feedforward로 두되, 최초 튜닝에서는 feedforward 사용 여부를 run metadata에 명시한다.
- 대회용 output은 `longlCmdType: 1`이며, signed PI output을 `accel`/`brake` [0,1]로 분리한다. `accel_gain`, `brake_gain`, deadband, saturation은 YAML tuning parameter다.
- stop/finish/trajectory stale에서 integral reset과 safe stop을 수행한다.

### 6.3 튜닝 순서

1. MORAI ExternalCtrl, `/ctrl_cmd` 단일 publisher, `longlCmdType=1`, accel/brake/steering 방향을 정지·저속에서 확인
2. 직선 저속에서 wheelbase=3.000 m, rear-axle ego frame, map/reference frame 일치와 MORAI velocity 단위를 확인
3. 종 제어만: 짧은 직선·20 km/h 고정 속도에서 `Kp`, `Ki`, accel/brake gain, saturation/anti-windup을 튜닝한다. PP lookahead와 조향 제한은 안전한 기준값으로 고정한다.
4. 횡 제어만: 3단계에서 통과한 PI를 고정하고, 20 km/h 고정 속도·직선+완만한 곡선에서 fixed lookahead `ld`, steering rate/limit을 튜닝한다.
5. 결합 검증: 곡선·`regulation_curvature` speed profile을 추가하고 횡/종 상호작용을 이전 run과 비교한다. 이 단계에서는 새 gain을 찾기보다 3·4단계 결과가 함께 안정적인지 확인한다. fixed `ld`가 안정화된 뒤에만 velocity-scaled lookahead를 별도 tuning case로 연다.
6. 전체 preferred course에서 finite finish까지 한 번 주행해 비교 artifact를 남김

## 7. `mpc_pi` — 현 단계 보류

`mpc_pi`는 package 이름만 유지한다. PP baseline, vehicle/actuator calibration, generator, validator가 끝날 때까지 package skeleton·solver·fallback 설계·튜닝을 시작하지 않는다.

## 8. 현재 코드에서 가져올 것과 바꿀 것

기존 Python `ajou_control/scripts/pure_pursuit_pi_controller.py`는 30 Hz default, `/recorded_trajectory_path` `nav_msgs/Path`, `/Ego_topic`, MORAI `/ctrl_cmd`에 직접 연결된다. 이는 MORAI `CtrlCmd` type 1 behavior와 lookahead/PI의 참고 자료다. 다만 파일의 `wheel_base=2.8` 및 실행 YAML의 2.8 m는 현재 차량 config의 3.000 m와 다르므로 신규 C++ PP의 기준값으로 복사하지 않는다.

최종 C++ 설계에서는 다음을 바꾼다.

| 현재 | 최종 |
|---|---|
| `nav_msgs/Path` + 별도 speed topic | 하나의 `Trajectory` |
| MORAI `EgoVehicleStatus` 직접 의존 | YAML로 고르는 MORAI GT/localization source + TF pose |
| `/ctrl_cmd` 직접 publish | 계속 직접 publish하되 C++와 `CtrlCmd.msg` 단위 계약으로 정리 |
| Python runtime | C++ controller runtime |
| same-name CSV overwrite | validator의 immutable run artifact |

## 9. 구현 전 확인 항목

1. `/localization/odom` topic/type은 확정됐다. FAST-LIO `camera_init -> body`를 `map -> base_link`로 정렬하는 TF ownership, 그리고 FAST-LIO/GT odometry의 단일 publisher 선택은 localization 구현 시 확인한다.
2. 규정에 맞는 `longlCmdType: 1`에서 PI output이 MORAI의 `accel`/`brake` 반응으로 어떻게 대응되는지, steer sign과 command cap이 맞는지는 첫 PP 실험에서 보정해야 한다.
3. wheelbase와 차체 치수는 확정했지만, MORAI `front_steer`의 실제 saturation, steering rate, accel/brake response limit은 저속 step 시험으로 정해야 한다.
4. stale input·TF lookup failure·trajectory 종료 때 PP가 적용할 safe-stop brake profile과 stop hold 정책을 정해야 한다.
5. 최대 80 km/h 운용은 PP가 20 Hz 및 상향 Hz에서 deadline miss 없이 주행한 결과가 나온 뒤에만 검토한다.
