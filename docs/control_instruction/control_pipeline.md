# 제어 파이프라인 설계 지시서

## 0. 이 문서의 범위

이 폴더는 구현된 제어단의 설계·운영 기준이다. 코스 CSV와 Lanelet 맵은 제어단이 수정하지 않으며, package별 YAML과 ROS 계약을 이 문서 기준으로 유지한다.

목표는 제어기 하나를 빨리 만드는 데 있지 않다. 같은 기준 궤적에 대해 `주행 → 기록 → 비교 → 파라미터 하나 변경 → 재주행`을 빠르게 반복할 수 있는 환경을 먼저 만드는 것이다. **현재 구축 범위는 `purepursuit_pi` 하나로 이 폐루프를 완성하는 것**이며, MPC 비교는 이 기준선이 완성된 뒤의 별도 단계다.

### 문서 읽는 순서와 기준 문서

이 문서는 네 문서의 **공통 결정과 구현 순서**를 정하는 상위 매뉴얼이다. 같은 정보가 여러 문서에 있을 때 아래 표의 기준 문서를 따른다.

| 알고 싶은 것 | 기준 문서 | 다른 문서의 역할 |
|---|---|---|
| package 경계, 공통 topic/frame, 현재 운영 source, 단계 순서 | 이 문서 | 각 package 문서는 자기 책임만 상세화 |
| Lanelet CSV, route finite 정책, speed profile | [trajectory_generator.md](trajectory_generator.md) | 이 문서는 입력/출력 연결만 설명 |
| PP/PI, 차량값, MORAI command, C++ 구조 | [trajectory_follower.md](trajectory_follower.md) | 이 문서는 command의 공통 wire contract만 설명 |
| run artifact, RViz/rqt, history, 지표, gate | [trajectory_validator.md](trajectory_validator.md) | 이 문서는 artifact가 pipeline을 잇는다는 점만 설명 |

현재 모든 최초 구현과 tuning run의 기준은 `purepursuit_pi` + `ego_state.source: morai_gt` + `/Ego_topic`이다. localization/MPC는 문서에 계약만 보존한 **후속 단계**다.

## 1. 확정된 역할과 패키지 이름

| 영역 | 패키지 이름 | 언어 | 책임 |
|---|---|---|---|
| 궤적 생성 | `trajectory_generator` | Python 허용 | Lanelet CSV 중심선으로 기준 궤적 생성, publish, RViz 확인. RViz 클릭 Bézier는 이번 구축 범위에서 제외 |
| 기준 제어기 | `purepursuit_pi` | **C++** | Pure Pursuit 횡제어 + PI 종제어 |
| 고급 제어기 | `mpc_pi` | **C++** | 이름만 예약. PP 파이프라인·튜닝이 끝날 때까지 생성·실행하지 않음 |
| 궤적 평가 | `trajectory_validator` | Python 허용 | 실제 주행을 기록하고 기준 궤적과 비교하여 RViz/rqt/파일 결과 제공 |

`trajectory_follower`는 별도 패키지 이름이 아니라 두 제어기의 **공통 인터페이스와 설계 기준**을 뜻한다. 자세한 기준은 [trajectory_follower.md](trajectory_follower.md)에 둔다.

### 1.1 ROS package 경계 — 확정

`base_ws/src/control/`은 ROS package가 아니라 제어 관련 package를 모아 두는 상위 디렉터리로만 사용한다. 현재 구현할 package 경계는 아래와 같다.

```text
base_ws/src/control/
├── trajectory_generator/   # Python: CSV 중심선, speed profile, Trajectory publish
├── purepursuit_pi/         # C++: PP 횡 + PI 종, /ctrl_cmd publish
└── trajectory_validator/   # Python: run 기록, RViz/rqt/history 비교
    └── history/
        └── pp_pi/          # PP/PI run artifact의 저장 위치
```

- `trajectory_generator`와 `trajectory_validator`는 둘 다 Python이지만 생성과 평가의 실행 시점·입출력·artifact 책임이 달라 합치지 않는다.
- `purepursuit_pi`는 대회용 C++ controller와 MORAI command ownership을 독립적으로 관리한다.
- 공통 계약은 기존 `Trajectory`, YAML, topic으로 맞춘다. 초기 PP 단계에서 `control_common` 같은 별도 공통 package는 만들지 않는다.
- `mpc_pi`는 PP pipeline 완료 뒤 같은 상위 디렉터리에 별도 package로 추가한다. 현재는 생성하지 않는다.

### 1.2 package별 YAML과 user-facing launch — 확정

YAML은 package의 책임 범위를 넘지 않는다. 어느 launch도 다른 control package의 YAML을 읽지 않는다.

```text
trajectory_generator/config/trajectory_generator.yaml
  └─ route CSV, catalog/map version, sampling, speed profile

purepursuit_pi/config/pp_pi.yaml
  └─ MORAI/localization input, PP/PI gain, command/safety limit

trajectory_validator/config/trajectory_validator.yaml
  └─ validation gate, history, timeout, visualization policy

ioniq5/config/vehicle.yaml
  └─ IONIQ 5 물리 제원의 유일한 원본
```

- 경로·속도 profile은 generator YAML에서만 바꾼다. PP/PI parameter는 `pp_pi.yaml`에서만 바꾼다. 기록/표시 policy는 validator YAML에서만 바꾼다.
- `pp_pi.launch`는 시작 시 자신이 읽은 resolved config, `run_label`, checksum을 latched `/control/purepursuit_pi/run_config` (`std_msgs/String`, UTF-8 YAML text)로 publish한다. validator는 이 snapshot을 `trajectory_validator/history/pp_pi/<run_id>/parameters.yaml`으로 보관한다.
- `ioniq5/config/vehicle.yaml`은 generator와 follower가 함께 읽는 차량 제원의 단일 원본이다. validator는 controller run snapshot에서 해당 정보를 기록한다.

`pp_pi_tuning.launch`는 만들지 않는다. 세 노드를 한꺼번에 재시작하면 controller gain 하나를 바꿀 때 generator와 validator까지 불필요하게 다시 올라오기 때문이다. 사용자가 직접 실행하는 launch는 다음 세 개다.

| launch | package | 포함 노드 | 사용 시점 |
|---|---|---|---|
| `trajectory_generator.launch` | `trajectory_generator` | generator | 시작 후 유지. 경로·속도 profile을 바꿀 때만 재시작 |
| `trajectory_validator.launch` | `trajectory_validator` | live validator + RViz/rqt/history panel | 시작 후 유지. live 기록과 과거 run 비교를 모두 담당 |
| `pp_pi.launch` | `purepursuit_pi` | PP+PI controller만 | **gain 하나를 바꿀 때마다 이것만 재시작** |

각 launch는 자기 package YAML만 읽는다. validator는 YAML로 route/controller/ego source를 복제하지 않고, generator의 trajectory/status와 follower의 run snapshot·normalized ego state를 구독한다. launch argument는 `start_rqt`, `start_control_rviz`처럼 UI를 켜고 끄는 용도만 허용한다.

`tuning.scope` 같은 controller와 무관한 YAML toggle은 없다. `traj_val` RQT에는 종방향·횡방향·통합 탭이 항상 있고, 사용자는 보고 싶은 탭만 클릭한다. 탭 선택은 주행, controller config, history에 영향을 주지 않는다.

YAML 변경 뒤의 재시작 범위는 아래로 고정한다. controller를 현재 `RECORDING` 중에 내리면 run이 `aborted`가 되므로, finish 또는 abort 처리가 끝난 `ARMED` 상태에서만 바꾼다.

| 변경한 값 | 재시작할 launch | 유지할 launch |
|---|---|---|
| `pp_pi.yaml`의 controller 또는 `ego_state` | `pp_pi.launch` | generator, validator |
| `trajectory_generator.yaml`의 route/profile | `trajectory_generator.launch` | controller, validator |
| `trajectory_validator.yaml`의 gate/history | `trajectory_validator.launch` | generator, controller |
| `ioniq5/vehicle.yaml` | generator와 controller | validator |

## 2. 기존 스택 재사용 원칙

제어 패키지가 map, TF, 센서 시각화를 새로 소유하거나 중복 publish하지 않는다.

```text
기존 환경
  bringup/maptf_morai.launch
    map -> morai_map -> base_link TF
  bringup/sensing.launch
    sensor TF, LiDAR/camera, vehicle model/RViz
  lanelet_map_visualizer
    Lanelet 맵 marker

신규 제어 파이프라인
  위 map frame을 그대로 사용해 trajectory/ego/evaluation overlay만 추가
```

- RViz Fixed Frame은 `map`으로 고정한다.
- `trajectory_generator`는 `map` frame reference를 publish하고, follower는 `map -> morai_map -> base_link` TF를 소비한다. validator는 follower가 publish한 normalized ego state만 소비한다. control package가 TF를 새로 publish하지 않는다.
- 초기 MORAI 단계에서는 `maptf_morai.launch`가 `/Ego_topic`에서 만든 `map -> morai_map -> base_link`를 사용한다. follower가 이 TF로 `map` 기준 차량 pose를 얻어 `/control/purepursuit_pi/ego_state`로 publish한다.
- `sensing.launch`의 센서/차량 모델 시각화와 Lanelet map marker는 그대로 사용한다. 새 RViz config에는 기준 궤적·실주행 궤적·평가 marker display만 추가한다.
- `maptf_morai.launch` 자체는 `/localization/odom`을 만들지 않고 TF와 `/Ego_topic`을 사용한다. 초기 MORAI GT 주행은 이 경로를 그대로 쓴다.
- localization source의 입력 topic은 **`/localization/odom` (`nav_msgs/Odometry`)**으로 확정한다. 현재 FAST-LIO launch는 `/Odometry`를 이 이름으로 remap한다. 다만 raw FAST-LIO odometry는 시작점 기준 `camera_init -> body`이므로, `map` 기준 reference와 직접 좌표를 섞지 않는다. localization stack이 `map -> camera_init` 및 `body -> base_link`를 제공해 최종 `map -> base_link`가 성립한 뒤에만 follower/validator가 사용한다.

## 3. 최종 목표 파이프라인

```text
                         existing map / TF / sensing / RViz
                                      │  frame: map
                                      ▼
competition_route CSV ───────► trajectory_generator ─────► /planning/trajectory/reference
                                        │                   /planning/trajectory/reference_path
                                        │                   /planning/trajectory/reference_markers
                                        │
      /Ego_topic + map→base_link TF ───┼──────────────► purepursuit_pi
      (current PP/tuning source)        │
      /localization/odom + map→base_link TF toggle ────┘       (future switch; inactive now)
                                                         ▼
                                           /ctrl_cmd (morai_msgs/CtrlCmd, type 1)
                                                         │
                                                       MORAI vehicle
                                                         │
                 reference + generator status + normalized ego state + /ctrl_cmd ─► trajectory_validator
                                                                  ├─ RViz overlays
                                                                  ├─ rqt_multiplot topics
                                                                  └─ immutable run artifacts
```

### 3.1 궤적 계약

제어기의 공식 기준 입력은 이 워크스페이스에 이미 있는 `autoware_planning_msgs/Trajectory`로 한다.

```text
/planning/trajectory/reference       autoware_planning_msgs/Trajectory
/planning/trajectory/reference_path  nav_msgs/Path                  # RViz 호환 출력
```

`TrajectoryPoint`에는 최소한 pose, `longitudinal_velocity_mps`, `acceleration_mps2`, `time_from_start`를 채운다. yaw, curvature에서 계산 가능한 `heading_rate_rps`, `front_wheel_angle_rad`도 generator가 채워 debug와 제약 검증에 사용한다. `nav_msgs/Path`와 별도 속도 토픽만을 제어기 공식 계약으로 유지하지 않는다.

### 3.2 MORAI 제어 명령 계약

초기 제어단의 최종 출력은 MORAI 문서와 이 워크스페이스의 `morai_msgs/CtrlCmd.msg`를 그대로 따른다. 현재는 `purepursuit_pi`가 `/ctrl_cmd`의 유일한 publisher다. 이후 `mpc_pi`를 추가해도 동시에 publish하지 않고 controller 하나만 활성화한다.

```text
/ctrl_cmd  morai_msgs/CtrlCmd
  longlCmdType  1=Throttle, 2=Velocity, 3=Acceleration
  front_steer   front wheel steering angle [rad]
  rear_steer    rear wheel steering angle [rad]
  accel, brake  pedal command [0..1], longlCmdType=1에서 사용
  velocity      target velocity [km/h], longlCmdType=2에서 사용
  acceleration  target acceleration [m/s²], longlCmdType=3에서 사용
```

대회 규정집은 `Network Settings > Ego-0 > Cmd Control`의 Ego Ctrl Cmd를 통한 종·횡 제어와 `/Ctrl_cmd`의 `longi type 1`, 즉 `accel`·`brake` 방식을 명시한다. ROS message field 이름은 `longlCmdType`이므로 대회용 controller는 `longlCmdType: 1`로 고정한다. 횡은 `front_steer` rad, 종은 PI의 signed output을 pedal command로 변환한다. 규정 표기의 `/Ctrl_cmd`와 ROS message 주석의 `/ctrl_cmd`는 이 스택에서 같은 제어 계약으로 취급하며, 실제 ROS publisher topic은 **`/ctrl_cmd`**로 고정한다.

```text
u_long > deadband : accel = clamp(accel_gain * u_long, 0, 1), brake = 0
u_long < -deadband: accel = 0, brake = clamp(brake_gain * -u_long, 0, 1)
otherwise         : accel = 0, brake = 0
```

`velocity`와 `acceleration` fields는 type 1 운용에서 inactive다. `front_steer` sign, steering limit, `accel_gain`, `brake_gain`, brake deadband는 첫 PP bring-up에서 `/Ego_topic`과 실제 MORAI 거동으로 검증해 YAML에 고정한다. run 전 확인 순서는 `ExternalCtrl 활성 → /ctrl_cmd 단일 publisher → longlCmdType=1 → 저속 accel/brake/steer 방향 확인`이다.

### 3.3 ego-state source YAML toggle

초기에는 아직 localization이 완성되지 않았으므로 MORAI GT를 쓴다. 이후 **follower의 `pp_pi.yaml`만** source를 바꾼다. validator YAML은 바꾸지 않으며, follower가 정규화 ego state를 계속 publish한다. TF를 새로 만들지 않는다.

```yaml
ego_state:
  source: morai_gt             # morai_gt | localization
  morai_gt_topic: /Ego_topic
  localization_odom_topic: /localization/odom
```

- `morai_gt`: `/Ego_topic`을 freshness/velocity source로 받고, pose/heading은 message 좌표를 임의로 더하지 않고 해당 stamp의 `map -> base_link` TF로 얻는다.
- `localization`: `/localization/odom`의 freshness를 확인하고, 같은 stamp의 `map -> base_link` TF로 pose/heading을 얻는다. FAST-LIO의 raw pose frame(`camera_init`, `body`)를 reference의 `map` 좌표로 직접 비교하지 않는다. `twist`가 유효하지 않으면 map pose의 timestamp 차분으로 longitudinal/lateral velocity를 계산하고 그 사실을 run metadata에 남긴다.
- 현재 `fast_lio_odom_map_bridge.py`의 `/localization/odom_map` 출력은 `/Ego_topic`으로 초기 map 정렬을 하는 **MORAI 검증용 보조 경로**다. 대회 localization mode의 의존성으로 쓰지 않는다.
- source toggle은 `trajectory_generator`나 `trajectory_validator`가 아니라 `purepursuit_pi`에만 적용한다. validator 입력은 항상 `/control/purepursuit_pi/ego_state`다.

#### 현재 PP pipeline의 GT-first 운영 규칙

localization toggle은 미리 만들어 두되, **첫 PP 제어·튜닝·validator 비교 run은 전부 `ego_state.source: morai_gt`로 고정**한다. 즉 `/Ego_topic`의 timestamp와 `maptf_morai.launch`의 `map -> morai_map -> base_link` TF를 차량 상태의 기준으로 쓴다.

- 초기 run에서 `/localization/odom`, FAST-LIO, GT-to-odom adapter를 ego-state source로 선택하지 않는다.
- follower는 `morai_gt` source를 쓰며, run metadata에는 `ego_state_mode: morai_gt`, `ego_state_topic: /control/purepursuit_pi/ego_state`를 기록한다.
- `/localization/odom`으로의 전환은 FAST-LIO의 map alignment/TF/freshness가 검증된 뒤, 별도 run부터 YAML 한 값으로 명시적으로 수행한다. GT run과 localization run을 동일 tuning 비교군으로 섞지 않는다.

### 3.4 실행 단위(run)

각 주행은 한 개의 `run_id`로 묶는다. 동일 파라미터 재주행도 덮어쓰지 않는다. run label codec, metadata field, history 선택 방식의 상세 기준은 [trajectory_validator.md](trajectory_validator.md)를 따른다.

```text
base_ws/src/control/trajectory_validator/history/
  pp_pi/
    index.csv
    <run_id>/                 # <YYYYmmdd_HHMMSS>__<run_label>
      metadata.yaml           # map/route/controller/version/frame/start condition
      parameters.yaml         # 실행 시작 순간의 pp_pi.yaml 실제 적용값 복사본
      reference.csv           # generator profile report + 기준 geometry snapshot
      samples.csv             # time-series 원자료
      metrics.yaml            # run 종료 후 요약 지표
      events.csv              # start/finish/abort/stale 등 이산 이벤트
```

`pp_pi`는 현재 유일한 controller이므로 history의 첫 controller directory 이름을 `pp_pi`로 고정한다. 나중에 `mpc_pi`를 실제 구현할 때만 같은 위치에 `history/mpc_pi/`를 별도 추가한다. 이 구조가 "파라미터 하나만 바꾼 이전 run과 비교"의 기준이다. `samples.csv`, `reference.csv`, `events.csv`, `index.csv`의 고정 column/단위/기록 시점은 [trajectory_validator.md의 CSV schema](trajectory_validator.md#71-csv-기록-원칙--확정)를 단일 기준으로 삼는다. current run과 history run은 `route_id`, map version, frame, ego-state mode, vehicle/reference 조건이 다르면 기본 filter에서 제외하고, 사용자가 강제로 고르면 경고를 표시한다.

## 4. 단계별 구현 순서

| 순서 | 구현 범위 | 완료 판정 |
|---:|---|---|
| 1 | PP 전용 topic, frame, route/run artifact, launch 구성 | package 간 입출력과 `/ctrl_cmd` 단일 publisher 규칙이 문서·launch에 고정됨 |
| 2 | `trajectory_generator`의 Lanelet CSV mode와 RViz 검증 | 전체 코스/부분 코스 CSV에서 `map` frame 기준 중심선과 target trajectory가 정확히 겹쳐 보임 |
| 3 | 임시 Python smoke test와 `purepursuit_pi` C++ skeleton | generator 출력, ego-source toggle, MORAI `/ctrl_cmd`가 한 PP 흐름으로 이어짐. Python은 연결 확인에만 사용 |
| 4 | `trajectory_validator` 및 RViz/rqt/run artifact | 기준/현재/과거 run 비교와 횡·종 시계열이 같은 실행에서 보임 |
| 5 | `purepursuit_pi` C++ 완성 및 튜닝 | 대회 보험용 제어기를 반복 가능한 run 비교로 완성 |
| 6 | PP pipeline 종료 판정 | 저속 직선·곡선·선택 구간·전체 preferred course run을 비교 가능하게 남김. `mpc_pi`는 이 단계 뒤에 별도 논의 |

RViz 클릭 기반 Bézier mode는 정적 장애물 회피 scope가 확정될 때까지 **구현하지 않고 보류**한다. 전체 코스 중심선 생성과 기본제어·validator 튜닝의 선행조건이 아니다.

## 5. 제어 주기 원칙

제어 주기는 20~50 Hz 범위이며, 최초 PP pipeline은 **20 Hz**로 시작한다. 30/40/50 Hz 상승은 controller worst-case cycle time, validator deadline miss, MORAI command 수용률을 측정한 뒤에만 한다. 속도별 tick 이동거리 표와 고속 운용 조건은 [trajectory_follower.md](trajectory_follower.md)에 한 번만 유지한다.

## 6. 이번 설계에서 명확히 제외하는 것

- competition route CSV, Lanelet map, 기존 `maptf_morai.launch`, `sensing.launch`의 수정
- 새 TF publisher 또는 map publisher 작성
- 정적/동적 장애물 인지와 충돌 판정의 구현
- 실제 차량 adapter, CAN mapping의 단정
- `trajectory_validator`를 제어기 앞 trajectory 차단 gate로 사용하는 것
- 이번 PP 파이프라인 중 `mpc_pi` package 생성·solver 설계·MPC tuning

후자의 안전성/형식 검사는 generator의 load-time validation으로 시작한다. `trajectory_validator`의 주 역할은 **주행 후 추종 성능 평가**다.

## 7. 현재 확정 사항과 후속 확인

### 현재 확정 사항

- `preferred_course`만 쓰며, order 1의 `200889` 시작부터 order 42의 `200889` 끝까지 한 번만 주행한다. candidate corridor와 loop는 1차 범위에서 제외한다.
- 첫 PP profile은 `constant_kph: 20`이고, 정상 run 3회 뒤 `regulation_curvature`로 전환한다. 상세 speed 규칙은 generator 문서를 따른다.
- 첫 PP 제어와 평가는 follower가 `/Ego_topic` MORAI GT를 정규화한 `/control/purepursuit_pi/ego_state` 기준이다.

### 후속 확인

1. PP bring-up에서 `longlCmdType: 1`의 accel/brake gain, deadband, `front_steer` sign/limit을 MORAI 거동으로 보정한다.
2. stale input·TF lookup failure·trajectory 종료 때 PP가 적용할 safe-stop brake profile과 stop hold 정책을 정한다.
3. localization 전환 전에는 FAST-LIO `camera_init -> body`를 `map -> base_link`로 정렬하는 TF ownership과 `/localization/odom` 단일 publisher를 검증한다.
