# Control Pipeline

이 문서는 `base_ws/src/control` 아래에 만들 제어 파이프라인 패키지의 역할을
정리한다. 제어 파이프라인은 크게 세 패키지로 나눈다.

1. `trajectory_editor`
2. `trajectory_follower`
3. `trajectory_validator`

핵심 기준은 단순하다. editor는 trajectory를 만들고 저장하며, publish 단계에서는
저장된 trajectory를 `/control/trajectory`로 쏜다. follower는
독립 제어기로서 어떤 source에서 온 trajectory든 정해진 입력 규격만 맞으면 받아서
MORAI로 `/ctrl_cmd`를 쏜다. validator는 MORAI에서 실제로 주행된 결과를 기록하고
평가한다.

## 전체 구조

```text
trajectory source
  -> trajectory_follower
  -> /ctrl_cmd
  -> MORAI

default trajectory source:
  trajectory_editor -> /control/trajectory

MORAI state + planned trajectory + control command
  -> trajectory_validator
  -> logs / metrics / report / RViz overlay / rqt_multiplot replay
```

`trajectory_validator`는 follower 앞에서 trajectory를 막는 safety filter가 아니다.
주행 중/주행 후 평가를 위한 observer로 둔다.

## 패키지 역할

| 우선 | 패키지 | 역할 | 주 출력 |
| ---: | --- | --- | --- |
| 1 | `trajectory_editor` | 주행할 trajectory 생성/저장 및 publish | trajectory file, `/control/trajectory` |
| 2 | `trajectory_follower` | 독립 trajectory 추종 제어기, MORAI `/ctrl_cmd` publish | `/ctrl_cmd` |
| 3 | `trajectory_validator` | MORAI 실제 주행 기록, 성능 평가, RViz/rqt_multiplot overlay | report, metrics, log, overlay |

## Topic Interface

| 단계 | Topic | Type | 비고 |
| --- | --- | --- | --- |
| Trajectory input | configurable, default `/control/trajectory` | `autoware_planning_msgs/Trajectory` | editor/planning/test/rosbag 등 어떤 source든 가능 |
| MORAI ego state | `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | follower/validator의 실차량 상태 기준 |
| Localization odometry | `/localization/odom` | `nav_msgs/Odometry` | 사용 가능하면 follower pose/twist 기준 |
| MORAI command | `/ctrl_cmd` | `morai_msgs/CtrlCmd` | follower가 MORAI로 직접 publish |
| Follower debug | `/control/debug/follower/state` | `ajou_control_msgs/TrajectoryFollowerDebug` | Pure Pursuit/PI 내부 수치 |
| Validator sample | `/control/validator/sample` | `ajou_control_msgs/TrajectoryValidationSample` | 실시간 rqt_multiplot/CSV 기준 |
| Validator run info | `/control/validator/run_info` | `ajou_control_msgs/TrajectoryValidationRunInfo` | run 제목, config, 튜닝 파라미터 |
| Validator overlay | `/control/validator/overlay/*` | `nav_msgs/Path`, `MarkerArray`, `PoseStamped` | 저장된 run을 RViz에 재표시 |
| Validator overlay sample | `/control/validator/overlay/sample` | `ajou_control_msgs/TrajectoryValidationSample` | 저장 CSV replay를 rqt_multiplot에 표시 |

메시지는 우선 이미 포팅된 `autoware_planning_msgs/Trajectory`를 trajectory 표준으로
쓴다. follower는 editor에 직접 의존하지 않는다. follower 출력은 별도 중간 command
topic을 필수로 두지 않고, MORAI 입력인 `morai_msgs/CtrlCmd`를 직접 publish한다.
제어 파이프라인 전용 debug/validation message는 기존 Autoware message를 수정하지 않고
`base_ws/src/msgs/ajou_control_msgs`에 추가한다.

## 1. `trajectory_editor`

상세 구현 기준: [`trajectory_editor/IMPLEMENTATION.md`](trajectory_editor/IMPLEMENTATION.md)

### 역할

`trajectory_editor`는 제어 파이프라인의 trajectory source이다. YAML config에 적힌
Lanelet2 map, 같은 버전의 drive-course lanelet CSV, 시작/종료 lanelet id CSV,
고정 최종 전역경로, 저장 경로를 입력으로 받아 follower가 따라갈 trajectory를 직접
만들고 저장한다. publish 단계에서는 저장된 trajectory를 읽어 `/control/trajectory`로
쏜다. Lanelet2 map 파일은 특정 버전에 고정하지 않고
config/launch argument로 받아 `lanelet2_map_v45.osm`처럼 버전이 바뀌어도 같은 node가
동작하게 한다. 최종 전역경로는 일반 실행 중 바뀌는 값이 아니라
`configs/global_path/2026_molit_comp_global_path.txt`를 기본 기준선으로 쓰고,
시작/종료 CSV만 바꿔 같은 route 위의 부분 trajectory를 만든다.

초기 기능:

- MVP 기준 trajectory 생성 로직은 `generate_trajectory.py` 하나의 Python 파일로 구현
- trajectory 생성은 launch 없이 Python script 직접 실행
- publish는 `trajectory_publisher.launch`로 저장된 trajectory를 읽어 쏘는 방식
- 생성 입력은 YAML config 하나로 관리
- Lanelet2 OSM과 projector YAML 로드
- 같은 version의 `drive_course_*_lanelets.csv` 로드
- 시작 CSV/종료 CSV의 `lanelet_id`로 생성 구간 결정
- drive-course lanelet `centerline_xy` 연결
- 최종 전역경로 기준으로 centerline 검사 및 보정
- centerline 기반 path resampling
- `TrajectoryPoint` pose, yaw, target velocity 채우기
- 생성 trajectory를 YAML config의 output path에 저장
- `/control/trajectory` publish
- RViz 확인용 debug path/marker publish

확장 기능:

- RViz click 또는 tool UI 기반 point 편집
- Lanelet relation adjacency 기반 정식 routing
- waypoint/anchor 추가/삭제/순서 변경
- 속도 구간 편집
- CSV/JSON/YAML trajectory import/export

### 인터페이스

| 방향 | Topic/File | Type | 비고 |
| --- | --- | --- | --- |
| Input | trajectory config | YAML | map/route/start/end/output 경로 |
| Input | Lanelet2 map | `.osm` | 중심선 후보, `lanelet_map_file` 인자로 지정 |
| Input | drive-course lanelets | CSV | OSM과 같은 version의 route 순서표 |
| Input | projector config | `.yaml` | WGS84 -> map 좌표 변환 |
| Input | start/end anchor | CSV | `lanelet_id`로 생성 구간 지정 |
| Input | global path | TXT (`x y z`) | 고정 최종 전역경로 기준선 |
| Output | generated trajectory | CSV/YAML metadata | publish/validator 기준 trajectory |
| Publish | `/control/trajectory` | `autoware_planning_msgs/Trajectory` | follower 입력 |
| Publish | `/control/debug/editor/path` | `nav_msgs/Path` | RViz 확인 |
| Publish | `/control/debug/editor/markers` | `visualization_msgs/MarkerArray` | 속도/point 확인 |

### MVP 기준

- launch 후 start lanelet id부터 end lanelet id까지의 trajectory가 `/control/trajectory`로 반복 publish된다.
- `header.frame_id`는 `map`으로 통일한다.
- point마다 position, orientation, longitudinal velocity가 채워진다.
- 생성 결과가 config의 output path에 저장된다.
- RViz에서 trajectory가 최종 전역경로와 도로 중심선 기준으로 자연스럽게 이어지는지 확인할 수 있다.

## 2. `trajectory_follower`

상세 구현 기준: [`trajectory_follower/IMPLEMENTATION.md`](trajectory_follower/IMPLEMENTATION.md)

### 역할

`trajectory_follower`는 독립적으로 실행되는 제어기이다. editor가 만든 trajectory뿐
아니라 planning output, test publisher, rosbag replay trajectory 등 어떤 trajectory가
들어와도 입력 규격만 맞으면 MORAI에 맞는 `/ctrl_cmd`를 계산해야 한다.

즉 follower는 trajectory 생성 방식에 대해 알지 않는다. follower의 책임은 현재 ego
상태와 입력 trajectory를 이용해 조향, 가감속 명령을 계산하고 `/ctrl_cmd`로 publish하는
것이다.

초기 기능:

- MVP 제어기는 `Pure Pursuit + PI` 조합으로 구현
- 제어기 종류와 내부 튜닝값은 YAML config에서 선택/수정
- configurable trajectory topic subscribe, default `/control/trajectory`
- `/Ego_topic` 또는 `/localization/odom` 기반 현재 pose/twist 추정
- ego와 가장 가까운 trajectory point 탐색
- lookahead target point 선택
- Pure Pursuit 기반 steering 계산
- PI 기반 target velocity 추종
- MORAI `morai_msgs/CtrlCmd` publish
- trajectory/ego timeout 시 정지 command publish

확장 기능:

- Stanley controller 비교
- PID/MPC longitudinal controller
- acceleration/brake split tuning
- steering, acceleration, jerk 제한
- low-speed stop 안정화
- control debug plot/marker publish

### 인터페이스

| 방향 | Topic | Type | 비고 |
| --- | --- | --- | --- |
| Subscribe | `~trajectory_topic`, default `/control/trajectory` | `autoware_planning_msgs/Trajectory` | source 독립 입력 |
| Subscribe | `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | MORAI ego 상태 |
| Subscribe | `/localization/odom` | `nav_msgs/Odometry` | 사용 가능 시 pose/twist 기준 |
| Publish | `/ctrl_cmd` | `morai_msgs/CtrlCmd` | MORAI 제어 입력 |
| Publish | `/control/debug/follower/target_point` | `geometry_msgs/PoseStamped` | 추종 target |
| Publish | `/control/debug/follower/path` | `nav_msgs/Path` | follower가 보는 trajectory |

### 입력 trajectory 규격

follower가 독립적으로 동작하려면 trajectory source는 아래 최소 규격을 만족해야 한다.

- Type은 `autoware_planning_msgs/Trajectory`이다.
- `header.frame_id`는 ego pose와 같은 frame이거나 TF로 변환 가능해야 한다.
- `points`는 주행 순서대로 정렬되어 있어야 한다.
- 각 point의 `pose.position`, `pose.orientation`은 유효해야 한다.
- 각 point의 `longitudinal_velocity_mps`는 follower가 추종할 목표 속도이다.
- point 간격은 너무 조밀하거나 성기지 않아야 한다.
- 마지막 point 또는 stop 구간은 velocity profile로 표현한다.

trajectory source 예시:

- `trajectory_editor`의 `/control/trajectory`
- planning stack의 final trajectory
- 테스트용 synthetic trajectory publisher
- rosbag replay trajectory
- 외부 waypoint 변환 node

### 기본 파라미터 후보

| Param | 기본값 후보 | 비고 |
| --- | ---: | --- |
| `lateral.type` | `pure_pursuit` | MVP lateral controller |
| `longitudinal.type` | `pi` | MVP longitudinal controller |
| `wheelbase_m` | `3.0` | `vehicle/config/vehicle.yaml` 기준 |
| `trajectory_topic` | `/control/trajectory` | follower 입력 topic |
| `max_steer_rad` | `0.698` | 40 deg |
| `min_lookahead_m` | `4.0` | 저속 기준 |
| `lookahead_time_s` | `1.5` | 속도 비례 lookahead |
| `pi.kp` | `0.6` | velocity error proportional gain |
| `pi.ki` | `0.05` | velocity error integral gain |
| `control_rate_hz` | `30.0` | `/ctrl_cmd` publish rate |
| `trajectory_timeout_s` | `0.5` | 초과 시 stop command |
| `ego_timeout_s` | `0.5` | 초과 시 stop command |

### MVP 기준

- 직선 trajectory에서 steering command가 0 근처로 유지된다.
- 곡선 trajectory에서 steering command가 연속적으로 변한다.
- target velocity가 trajectory velocity를 따라간다.
- trajectory 또는 ego state가 끊기면 MORAI에 정지 command를 낸다.
- `/ctrl_cmd` publish 후 MORAI ego velocity가 기대 방향으로 변한다.
- editor 없이 test trajectory publisher만 띄워도 follower가 동작한다.

## 3. `trajectory_validator`

상세 구현 기준: [`trajectory_validator/IMPLEMENTATION.md`](trajectory_validator/IMPLEMENTATION.md)

### 역할

`trajectory_validator`는 제어 입력을 막거나 trajectory를 수정하지 않는다. MORAI에서
실제로 주행된 결과와 follower 내부 제어 수치를 기록하고, 저장된 데이터를 기준 trajectory와
비교해 주행 품질을 평가한다.

validator의 주 분석 CSV는 `run_timeseries.csv`로 둔다. 이 파일은 row마다 기준 trajectory
값인 `gt_*`, MORAI `/Ego_topic`의 실제 주행 지표인 `ego_*`, `/ctrl_cmd`, follower 내부값을
함께 저장한다. Pure Pursuit/PI 튜닝값은 시계열 값이 아니므로 CSV에 반복 저장하지 않고
`run_id`, `run_title`, `run_metadata.yaml`에 따로 저장한다. topic별 CSV는 디버깅과
재처리를 위한 raw log로 남긴다.

ROS에서 실시간 확인하거나 저장된 CSV를 replay할 때는 custom msg를 표준 인터페이스로
쓴다. recorder는 `TrajectoryValidationSample.msg`를 `/control/validator/sample`로
publish하고, overlay node는 같은 형식의 message를 `/control/validator/overlay/sample`로
다시 publish한다. 그래서 rqt_multiplot은 실시간/저장 replay를 같은 field 이름으로 볼 수
있다. RViz는 sample message를 직접 표시하지 않고, validator가 변환한 `nav_msgs/Path`,
`geometry_msgs/PoseStamped`, `visualization_msgs/MarkerArray`를 표시한다.

구조는 세 단계로 나눈다.

- `trajectory_recorder_node.py`: MORAI 실제 주행 데이터와 제어기 세부 수치를 저장하고 RViz debug path/marker를 publish한다.
- `evaluate_trajectory.py`: 저장된 run directory를 불러와 기준 trajectory와 비교해 시계열 metric/report를 만든다.
- `trajectory_overlay_node.py`: 저장된 run directory를 다시 불러와 RViz에서 기준 trajectory, 실제 주행 path, error marker를 overlay/replay한다.

반복 실험 비교를 위해 recorder는 고정 recording window 정책을 사용한다. 기본 보조
정책은 `actual_drive_start`로 두고, ExternalCtrl 전환 후 실제 차량이 움직이기 시작한
시점을 기준으로 기록을 시작한다. ExternalCtrl 상태를 직접 확인할 수 없으면 `/ctrl_cmd`
velocity 또는 ego velocity threshold를 fallback 기준으로 사용한다.

기록 대상:

- planned trajectory: `/control/trajectory`
- control command: `/ctrl_cmd`
- follower debug state: `/control/debug/follower/state`
- follower target point: `/control/debug/follower/target_point`
- ego state: `/Ego_topic`
- localization output: `/localization/odom`, `/localization/pose`, `/localization/twist`
- collision topic: `/CollisionData`, `/VehicleCollisionData`
- simulation time: `/clock`

평가 항목:

- lateral tracking error
- longitudinal/station error
- heading error
- target velocity 대비 actual velocity error
- stop point overshoot
- maximum steering command
- acceleration/brake command saturation
- trajectory 이탈 여부
- collision 발생 여부
- goal 도달 여부와 도달 시간

### 인터페이스

| 방향 | Topic/File | Type | 비고 |
| --- | --- | --- | --- |
| Subscribe | `/control/trajectory` | `autoware_planning_msgs/Trajectory` | 기준 trajectory |
| Subscribe | `/ctrl_cmd` | `morai_msgs/CtrlCmd` | 실제 입력 command |
| Subscribe | `/control/debug/follower/state` | `ajou_control_msgs/TrajectoryFollowerDebug` | Pure Pursuit/PI 내부 수치 |
| Subscribe | `/control/debug/follower/target_point` | `geometry_msgs/PoseStamped` | follower target |
| Subscribe | `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | MORAI 실제 주행 상태 |
| Subscribe | `/CollisionData` | `morai_msgs/CollisionData` | 충돌 평가 |
| Publish | `/control/validator/sample` | `ajou_control_msgs/TrajectoryValidationSample` | 실시간 rqt_multiplot/CSV 기준 |
| Publish | `/control/validator/run_info` | `ajou_control_msgs/TrajectoryValidationRunInfo` | run 제목/파라미터 |
| Publish | `/control/validator/reference_path` | `nav_msgs/Path` | RViz 기준 trajectory |
| Publish | `/control/validator/actual_path` | `nav_msgs/Path` | RViz 실제 주행 path |
| Publish | `/control/validator/error_markers` | `visualization_msgs/MarkerArray` | RViz 오차 표시 |
| Publish | `/control/validator/overlay/reference_path` | `nav_msgs/Path` | 저장 run 기준 trajectory overlay |
| Publish | `/control/validator/overlay/actual_path` | `nav_msgs/Path` | 저장 run 실제 주행 path overlay |
| Publish | `/control/validator/overlay/error_markers` | `visualization_msgs/MarkerArray` | 저장 run lateral error overlay |
| Publish | `/control/validator/overlay/ego_pose` | `geometry_msgs/PoseStamped` | replay 중 ego pose |
| Publish | `/control/validator/overlay/sample` | `ajou_control_msgs/TrajectoryValidationSample` | 저장 run rqt_multiplot replay |
| Output | `artifacts/control_validation/<run_id>/` | CSV/JSON/MD | 주행 로그와 report |

### Report 후보

```text
artifacts/control_validation/<run_id>/
  run_metadata.yaml
  run_timeseries.csv
  reference_trajectory.csv
  ego_state.csv
  ctrl_cmd.csv
  follower_state.csv
  target_point.csv
  metrics_timeseries.csv
  summary.yaml
  report.md
```

### MVP 기준

- 주행 시작부터 종료까지 ego pose, velocity, `/ctrl_cmd`를 기록한다.
- planned trajectory 대비 lateral error를 계산한다.
- max/mean/RMS tracking error를 summary로 저장한다.
- collision 발생 여부를 report에 남긴다.
- goal 근처 도달 여부를 판단한다.
- 저장된 run을 RViz에서 reference/actual/error overlay로 다시 확인할 수 있다.

## 패키지 디렉터리 초안

```text
base_ws/src/control/
  trajectory_editor/
    CMakeLists.txt
    package.xml
    README.md
    launch/
    config/
    scripts/
    test/
  trajectory_follower/
    CMakeLists.txt
    package.xml
    README.md
    launch/
    config/
    scripts/
    test/
  trajectory_validator/
    CMakeLists.txt
    package.xml
    README.md
    launch/
    config/
    scripts/
    test/
base_ws/src/msgs/
  ajou_control_msgs/
    CMakeLists.txt
    package.xml
    msg/
      TrajectoryFollowerDebug.msg
      TrajectoryValidationSample.msg
      TrajectoryValidationRunInfo.msg
```

## 구현 순서 제안

1. `trajectory_editor`
   - `generate_trajectory.py`로 YAML config 기반 trajectory를 생성/저장하고, `trajectory_publisher.launch`로 저장된 trajectory를 `/control/trajectory`에 publish한다.
2. `trajectory_follower`
   - 독립 node로 실행해 configurable trajectory topic과 `/Ego_topic`을 받아 `/ctrl_cmd`와 `/control/debug/follower/state`를 publish한다.
3. `trajectory_validator`
   - recorder node로 MORAI ego state, `/ctrl_cmd`, follower 내부 수치를 `TrajectoryValidationSample.msg`와 CSV로 저장하고 evaluator script로 tracking error를 계산한다. 이후 overlay node로 저장된 run을 RViz와 rqt_multiplot에서 재확인한다.
4. 통합 launch
   - editor, follower, validator를 한 번에 실행한다.
5. 반복 tuning
   - validator report를 보고 follower 파라미터를 조정한다.

## 테스트 체크리스트

- [ ] editor가 `/control/trajectory`를 정상 publish한다.
- [ ] RViz에서 editor trajectory의 위치와 방향이 맞다.
- [ ] follower가 `/control/trajectory` 수신 후 `/ctrl_cmd`를 publish한다.
- [ ] follower가 `/control/debug/follower/state`를 `TrajectoryFollowerDebug.msg`로 publish한다.
- [ ] follower가 editor 없이 다른 trajectory topic을 입력으로 받아도 `/ctrl_cmd`를 publish한다.
- [ ] 직선 trajectory에서 차량이 직진한다.
- [ ] 곡선 trajectory에서 차량이 trajectory를 따라 회전한다.
- [ ] trajectory timeout 시 차량이 정지한다.
- [ ] validator가 ego state와 `/ctrl_cmd`를 파일로 기록한다.
- [ ] validator가 `/control/validator/sample`을 `TrajectoryValidationSample.msg`로 publish한다.
- [ ] validator report에 tracking error와 collision 여부가 남는다.
- [ ] validator overlay가 저장된 run의 reference/actual path와 error marker를 RViz에 표시한다.
- [ ] validator overlay가 저장된 run을 `/control/validator/overlay/sample`로 publish해 rqt_multiplot에서 재생한다.
