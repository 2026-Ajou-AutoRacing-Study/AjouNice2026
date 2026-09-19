# MORAI Trajectory Batch Runner

## 1. 목적

`trajectory_batch_runner`는 manifest에 적힌 sample trajectory들을 MORAI에서 순서대로
주행하고, 기존 `trajectory_validator` 기록이 완료될 때까지 반복 실행하는 ROS 패키지다.

한 번의 launch로 다음 항목을 처리한다.

1. config와 manifest 전체 사전 검증
2. trajectory 시작 pose·속도 확인
3. manifest에 지정된 MORAI scenario JSON 검증
4. `udp`, `maptf_morai_cor` 실행
5. iteration별 ScenarioLoad UDP 송신
6. `traj_gen`, `traj_val`, `traj_fol_a` 실행
7. validator 기록 완료 확인
8. iteration stack 종료 후 다음 행 실행

## 2. 실행 전 MORAI 준비

MORAI에서 다음 항목은 batch 시작 전에 한 번 준비한다.

1. Lane Change는 K-City 고속주로, 장애물 회피는 K-City 시험 환경을 연다.
2. F4에서 `configs/network/ego_network_1.json`을 로드하고 Connect한다.
3. F3에서 `configs/sensors/Sensor_5.json`을 로드하고 Connect한다.
4. Ego를 External Control 상태로 준비한다.
5. manifest에 지정한 scenario JSON과 `EgoNetwork/<stem>_MN.json`을 MORAI의
   Scenario 저장 위치에 함께 둔다.

runner의 ScenarioLoad UDP는 JSON 전체 경로가 아니라 확장자를 제외한 파일명만 전송한다.
따라서 저장소의 `configs/scenario`와 실제 MORAI scenario 저장 위치가 다르면 본문 JSON과
`EgoNetwork` sidecar를 모두 MORAI 쪽에 먼저 반영해야 한다.

## 3. 실행

### Highway Lane Change

```bash
roslaunch trajectory_batch_runner trajectory_batch_runner.launch \
  config_file:=lane_change.yaml \
  manifest_file:=highway_lc_manifest.csv
```

### Obstacle Avoidance

```bash
roslaunch trajectory_batch_runner trajectory_batch_runner.launch \
  config_file:=obstacle_avoidance.yaml \
  manifest_file:=obstacle_avoidance_manifest.csv
```

launch에서 바꾸는 값은 `config/` 안의 config 파일명과 `manifests/` 안의 CSV 파일명이다.
경로 전체를 launch에 적지 않는다.

현재 두 manifest는 header만 있다. 실제 trajectory CSV와 행을 추가하기 전에는 runner가
`실행할 iteration이 manifest에 없습니다` 오류로 종료한다.

## 4. Config 구조

`lane_change.yaml`과 `obstacle_avoidance.yaml`은 설정값을 직접 길게 담지 않고, 실행에
필요한 네 종류의 config를 선택하는 진입점이다.

```text
config/
├── lane_change.yaml
├── obstacle_avoidance.yaml
├── runtime/
│   └── default.yaml
├── map/
│   └── kcity.yaml
├── batch/
│   ├── lane_change.yaml
│   └── obstacle_avoidance.yaml
├── scenario/
│   └── start_pos.yaml
└── monitoring/
    └── default.yaml
```

| 변경하려는 내용 | 수정할 config |
| --- | --- |
| `roscore`, `udp`, `maptf_morai_cor`와 follower 실행 | `runtime/default.yaml` |
| 지도 폴더, 파일 선택 규칙, projector와 MORAI map 이름 | `map/kcity.yaml` |
| batch 이름, manifest, sample trajectory와 validator 결과 위치 | `batch/<시험>.yaml` |
| 원본 JSON, 생성 규칙, 생성 JSON 저장 위치 | `scenario/start_pos.yaml` |
| ROS topic, 완료 조건과 timeout | `monitoring/default.yaml` |

예를 들어 Obstacle Avoidance 진입점은 다음 조합만 보여준다.

```yaml
config_version: 4
includes:
  runtime: "runtime/default.yaml"
  map: "map/kcity.yaml"
  batch: "batch/obstacle_avoidance.yaml"
  scenario: "scenario/start_pos.yaml"
  monitoring: "monitoring/default.yaml"
```

include 경로는 진입점이 있는 `config/` 폴더 기준이다. 로더는 역할별 파일을 재귀적으로
병합하며, 두 파일이 같은 최종 키를 정의하면 덮어쓰지 않고 설정 오류로 처리한다.
실행 명령과 `config_file` 값은 분리 전과 동일하다.

`map/kcity.yaml`은 `kcity_map/lanelet_map/lanelet2_map_v84.osm`만 선택하고,
map과 projector 경로를 `maptf_morai_cor`에 명시적으로 전달한다.
v77 파일이 없으면 오류로 종료하며 구버전 지도로 대체하지 않는다.

## 5. Manifest

파일:

```text
manifests/
├── highway_lc_manifest.csv
└── obstacle_avoidance_manifest.csv
```

두 파일은 같은 6개 열을 사용한다.

```csv
iteration,experiment_type,sub_type,trajectory_file,scenario_file,speed_kph
```

| 열 | 의미 | 예 |
| --- | --- | --- |
| `iteration` | 전체 실행 순서, 1 이상의 중복 없는 정수 | `1` |
| `experiment_type` | `highway_lc` 또는 `obstacle_avoidance` | `highway_lc` |
| `sub_type` | 시험 세부 종류 | `single_lc` |
| `trajectory_file` | config의 trajectory 폴더 기준 상대경로 | `single_lc/sample_001.csv` |
| `scenario_file` | 생성·검증 후 해당 iteration에서 MORAI에 로드할 JSON 파일명 | `lc_start_200128.json` |
| `speed_kph` | Ego 초기 속도와 trajectory 첫 목표 속도 | `100` |

Highway LC 예:

```csv
iteration,experiment_type,sub_type,trajectory_file,scenario_file,speed_kph
1,highway_lc,single_lc,single_lc/sample_001.csv,lc_start_200128.json,100
2,highway_lc,double_lc,double_lc/sample_002.csv,highway_start_120.json,120
```

Obstacle Avoidance 예:

```csv
iteration,experiment_type,sub_type,trajectory_file,scenario_file,speed_kph
1,obstacle_avoidance,static_obstacle,avoid_001.csv,obstacle_start_40.json,40
```

`enabled`, 시작/종료 lanelet, sweep min/max/interval은 사용하지 않는다. 실행하지 않을 행은
manifest에서 제거한다. lanelet과 시작 pose는 trajectory CSV에서 직접 읽는다.

## 6. Trajectory 폴더

### Highway Lane Change

```text
base_ws/src/control/trajectory_generator/traj_sample/highway_lc/
├── single_lc/
├── double_lc/
└── continuous_lc/
```

config:

```yaml
paths:
  directories:
    trajectory_directory: "base_ws/src/control/trajectory_generator/traj_sample/highway_lc"
```

manifest의 `trajectory_file=single_lc/sample_001.csv`는 다음 파일을 뜻한다.

```text
base_ws/src/control/trajectory_generator/traj_sample/highway_lc/single_lc/sample_001.csv
```

### Obstacle Avoidance

```text
base_ws/src/control/trajectory_generator/traj_sample/obstacle_avoidance/
```

세부 폴더를 만들면 `trajectory_file`에 그 폴더까지 상대경로로 적는다.

## 7. Scenario 생성과 로드

`scenario_json_generator.py`는 manifest 전체에 대해 다음 작업을 수행한다.

1. trajectory 첫 행의 `x`, `y`, `z`, `yaw`, `lanelet_id`, `velocity_mps`를 읽는다.
2. `velocity_mps * 3.6`과 manifest의 `speed_kph`가 허용 오차 안에서 같은지 확인한다.
3. static TF로 Lanelet map pose를 MORAI pose로 변환한다.
4. 기존 `configs/scenario/start_pos.json`을 복사한다.
5. Ego pose를 갱신하고 초기 속도를 `scenario_generation.initial_speed_kph`(기본 0 km/h),
   제어 모드를 AV-ExternalCtrl(`currentControlMode=2`)로 설정한다. trajectory와 manifest의
   `speed_kph`는 controller의 주행 목표속도로 유지되며 초기 저장 속도와 구분된다.
6. manifest의 `scenario_file` 이름으로 `configs/scenario/`에 저장한다.
7. `configs/network/ego_network_1.json`을
   `configs/scenario/EgoNetwork/<scenario_stem>_MN.json`으로 함께 생성한다.

예를 들어 manifest의 `scenario_file`이 `lc_start_200128.json`이면
`configs/scenario/lc_start_200128.json`과
`configs/scenario/EgoNetwork/lc_start_200128_MN.json`을 생성하거나 기존 파일을 검증한다.
같은 map과 시작 pose를 사용하는 trajectory들은 목표속도가 달라도 이 scenario를 공유하고,
batch runner는 실제 iteration에서 이 파일명을 MORAI ScenarioLoad로 전송한다.

`scenario_file`은 폴더를 포함하지 않는 `.json` 파일명이어야 하며, 확장자를 제외한
stem은 MORAI ScenarioLoad의 ASCII 30-byte 제한을 만족해야 한다.

기존 scenario 파일이 있으면 pose, map, 정지 초기속도, ExternalCtrl, gear를 확인하고,
EgoNetwork sidecar가 network template과 같은지도 확인한다. 내용이 다르고
`overwrite_existing: false`이면 생성기는 자동으로 덮어쓰지 않고 중지한다.

batch runner 자체는 scenario JSON을 생성하거나 덮어쓰지 않는다. manifest의
`scenario_file`과 대응하는 `_MN.json`이 실제로 존재하는지, pose·map·초기 상태·네트워크가
설정과 일치하는지 사전 검증한 뒤 로드한다. 파일이 없다면 아래 생성기를 먼저 실행해야 한다.

`start_pos.json`의 다음 목록과 그 밖의 설정은 수정하지 않는다.

```text
vehicleList
pedestrianList
objectList
spawnPointList
```

두 config 모두 `scenario_template_json`으로 기존 `start_pos.json`을 사용한다. 이 값은
생성기 입력에만 사용되며, batch runner의 실제 로드 대상은 각 manifest 행의
`scenario_file`이다.

scenario만 미리 생성하고 싶으면 다음 명령을 사용한다.

```bash
rosrun trajectory_batch_runner scenario_json_generator.py \
  --config "$(rospack find trajectory_batch_runner)/config/lane_change.yaml" \
  --manifest highway_lc_manifest.csv
```

## 8. Batch 시작 시 한 번 유지하는 stack

| 항목 | 실제 launch | 역할 |
| --- | --- | --- |
| ROS Master | 바깥 `roslaunch`가 준비 | ROS 통신 |
| `udp` | `sensor_interface/morai_udp_interface.launch` | Sensor/Ego 수신과 제어 송신 |
| `maptf_morai_cor` | `bringup/maptf_morai_deg_correction.launch` | MORAI localization과 TF |

상위 `roslaunch`가 ROS Master를 준비하고, runner가 시작한 `udp`와
`maptf_morai_cor`만 batch 종료 시 정리한다. 사용자가 별도로 실행한 다른 프로세스를
이름으로 찾아 종료하지 않는다.

## 9. Iteration 한 번의 순서

```text
manifest 행 확인
  -> trajectory 시작 pose와 speed_kph 재검증
  -> traj_gen 실행
  -> /control/trajectory 확인
  -> ScenarioLoad UDP: network=true, ego=true, pause=false
  -> _MN.json으로 F4 네트워크 복구 및 /Ego_topic 시작 XY 확인
  -> traj_val 실행
  -> 새 validator history run 폴더 확인
  -> traj_fol_a 실행
  -> /ctrl_cmd 확인
  -> validator metadata 완료 대기
  -> follower, validator, generator 역순 종료
  -> 다음 iteration
```

ScenarioLoad는 validator를 시작하기 전에 실행한다. 따라서 Ego teleport와 F4 재연결 구간은
tracking 기록에 포함되지 않는다. Ego 재생성 시 F4 세션도 초기화되므로
`load_network_connection_data=true`로 대응하는 `_MN.json`을 함께 읽어 연결을 복구한다.
Ego는 0 km/h로 대기하고, validator가 armed된 뒤 시작한 controller가 trajectory
목표속도까지 가속한다.

## 10. 완료 판정

runner는 현재 trajectory 이름 아래에 validator가 새로 만든 run 폴더만 감시한다.

```text
base_ws/src/control/trajectory_validator/history/
└── <trajectory_basename>/
    └── <validator_run_directory>/
        ├── tracking_samples.csv
        └── metadata.json
```

config 기본 완료 조건:

```text
metadata.status = complete
metadata.recording.end_reason = goal_reached
```

timeout이나 다른 종료 사유가 발생하면 현재 iteration stack과 persistent stack을 정리하고
runner가 오류로 종료한다. 자동 retry와 resume는 구현하지 않는다.

## 11. Python 모듈

```text
scripts/
├── batch_runner.py
└── scenario_json_generator.py

src/trajectory_batch_runner/
├── config_loader.py
├── manifest_loader.py
├── trajectory_input.py
├── scenario_builder.py
├── morai_udp_client.py
├── ros_stack_manager.py
└── validation_monitor.py
```

| 모듈 | 책임 |
| --- | --- |
| `config_loader.py` | 역할별 YAML 병합, 검증과 저장소 상대경로 해석 |
| `manifest_loader.py` | manifest 6개 열 검증과 iteration 정렬 |
| `trajectory_input.py` | trajectory 첫 행과 시작 속도 확인 |
| `scenario_builder.py` | 정지/ExternalCtrl scenario와 EgoNetwork `_MN.json` 생성·검증 |
| `morai_udp_client.py` | 69-byte ScenarioLoad UDP payload와 송신 |
| `ros_stack_manager.py` | runner가 시작한 roslaunch의 생명주기 관리 |
| `validation_monitor.py` | topic 준비와 validator metadata 완료 확인 |
| `batch_runner.py` | 위 모듈들을 iteration 순서로 조율 |

모든 Python 파일 상단에는 해당 파일의 목적, 역할과 주요 흐름을 한글 주석으로 기록한다.

## 12. 구현하지 않은 기능

- trajectory 생성 알고리즘
- follower 또는 validator 알고리즘 변경
- controller parameter 자동 탐색
- 자동 retry 또는 중단 지점 resume
- 별도 결과 DB, dashboard, GUI와 자동 그래프
