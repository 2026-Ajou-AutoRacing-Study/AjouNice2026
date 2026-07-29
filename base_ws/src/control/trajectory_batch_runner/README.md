# MORAI Trajectory Batch Runner 계획

## 1. 현재 단계

이 문서는 `trajectory_batch_runner` 패키지를 앞으로 어떻게 구성할지 정하는 계획서다.

현재 패키지 디렉터리와 config는 구성되어 있다. launch와 Python 파일에는 실행 코드 없이
목적, 역할, 간략한 실행 흐름만 한글로 기록되어 있다. 실제 batch 실행 로직은 아직 구현하지
않는다.

## 2. 목표

config에서 지정한 `traj_sample` 폴더의 모든 trajectory CSV를 순서대로 주행한다.

trajectory 하나의 실행 순서는 다음과 같다.

```text
trajectory CSV 확인
  -> CSV의 시작 lanelet과 시작 목표 속도 확인
  -> 같은 시작 lanelet과 속도를 가진 MORAI 시나리오 로드
  -> traj_gen 실행
  -> traj_val 실행
  -> traj_fol_a 실행
  -> trajectory_validator 기록 완료
  -> follower, validator, generator 종료
  -> 다음 CSV
```

시험 종류는 두 개로 분리한다.

- Lane Change: highway 맵에서 실행
- Obstacle Avoidance: K-City 맵에서 실행

두 시험을 한 번의 batch에 섞지 않는다. 각각 별도의 config와 MORAI map 환경으로 실행한다.

## 3. 구현 범위

### 포함할 기능

1. 실행 config 하나 읽기
2. config에 지정된 trajectory 폴더의 모든 `*.csv` 찾기
3. 각 CSV의 시작 lanelet과 시작 목표 속도 읽기
4. 시작 lanelet과 속도에 맞는 시나리오 JSON 찾기
5. MORAI 시나리오 로드
6. 필수 ROS 스택 상태 확인
7. `traj_gen`, `traj_val`, `traj_fol_a` 순서대로 실행
8. `trajectory_validator/history`에 기록이 끝났는지 확인
9. 현재 run을 종료하고 다음 trajectory 실행

### 포함하지 않을 기능

- trajectory 생성
- follower 또는 validator 알고리즘 변경
- controller parameter 자동 탐색
- 여러 controller 조합 sweep
- 별도의 metric 재계산
- 별도의 dashboard 또는 GUI
- 자동 그래프 생성
- 자동 retry, resume, 분산 실행
- 기존 validator 결과를 복사한 별도 결과 데이터베이스

분석 데이터는 기존 `trajectory_validator`가 저장한 파일을 그대로 사용한다.

## 4. 패키지 구성안

현재 패키지는 아래와 같이 단순하게 구성한다.

```text
base_ws/src/control/trajectory_batch_runner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── lane_change.yaml
│   └── obstacle_avoidance.yaml
├── launch/
│   └── trajectory_batch_runner.launch
└── scripts/
    └── run_trajectory_batch.py
```

각 파일의 책임:

| 파일 | 책임 |
| --- | --- |
| `config/lane_change.yaml` | K-City 고속주로 Lane Change 시험의 모든 경로와 실행값 지정 |
| `config/obstacle_avoidance.yaml` | K-City Obstacle Avoidance 시험의 모든 경로와 실행값 지정 |
| `launch/trajectory_batch_runner.launch` | 사용할 config 경로를 runner에 전달 |
| `scripts/run_trajectory_batch.py` | 폴더 순회, 시나리오 선택, ROS launch 순서 관리 |
| `README.md` | 설정값, 실행 순서, 미확정 사항 설명 |

시나리오, trajectory, sensor, network JSON을 이 패키지 안으로 복사하지 않는다. config에서
원래 파일 또는 폴더의 경로를 지정한다. 현재 실제 설정값은
`config/lane_change.yaml`과 `config/obstacle_avoidance.yaml`을 기준으로 한다.

## 5. Config에서 지정할 값

두 config는 `config_version: 1`의 같은 스키마를 사용하며 시험 종류와 입력 경로만 다르다.
실제 키와 한글 설명은 config 파일 자체를 기준으로 한다.

- `config/lane_change.yaml`
- `config/obstacle_avoidance.yaml`

최상위 설정 묶음:

| 설정 | 내용 |
| --- | --- |
| `experiment` | 시험 이름과 Lane Change/Obstacle Avoidance 구분 |
| `paths` | 저장소, trajectory, scenario, sensor, network, map, controller, history 경로 |
| `map` | MORAI map 이름과 ROS 기준 frame |
| `trajectory_input` | 지정 폴더의 CSV 선택·정렬 규칙 |
| `scenario_selection` | CSV 첫 lanelet/속도에서 시나리오 파일을 결정하는 규칙 |
| `scenario_loader` | MORAI ScenarioLoad UDP 주소와 요청 flag |
| `persistent_stack` | `roscore`, `udp`, `maptf_morai_cor` |
| `per_run_stack` | `traj_gen`, `traj_val`, `traj_fol_a` |
| `topics` | 준비 및 실행 상태를 확인할 topic |
| `completion` | validator의 완료 판정값 |

모든 상대경로는 `paths.repository_root`를 기준으로 해석한다. 다른 PC로 저장소를 옮기면
`repository_root`만 먼저 바꾼다.

`paths.trajectory_dir`는 반드시 `paths.trajectory_package_dir` 내부의 `traj_sample`
하위여야 한다. 현재 publisher와 validator가 package 내부 경로만 허용하기 때문이다.
runner는 실제 launch에 전달할 때 다음과 같이 package 기준 상대경로로 바꾼다.

```text
실제 파일:
base_ws/src/control/trajectory_generator/traj_sample/60kph/example.csv

trajectory_csv launch 인자:
traj_sample/60kph/example.csv
```

Lane Change 기본값은 K-City 고속주로의 `170kph` 폴더다. Obstacle Avoidance 기본값인
`65kph`는 config와 속도별 시나리오 매칭을 검증하기 위한 값이며, 실제 장애물 회피
trajectory와 시나리오 폴더가 준비되면 `trajectory_dir`와 `scenario_dir`를 바꾼다.

## 6. Trajectory 폴더 사용 규칙

`paths.trajectory_dir`에 지정된 폴더 바로 아래의 모든 `*.csv`를 사용한다.

예를 들어 다음과 같이 지정하면:

```yaml
paths:
  trajectory_dir: "base_ws/src/control/trajectory_generator/traj_sample/60kph"
```

실행 대상은 다음과 같다.

```text
traj_sample/60kph/*.csv
```

새로 만든 trajectory 100개를 별도 폴더에 넣는 경우:

```text
base_ws/src/control/trajectory_generator/traj_sample/lane_change_batch/
├── lane_change_001.csv
├── lane_change_002.csv
├── ...
└── lane_change_100.csv
```

config에는 해당 폴더만 지정한다.

```yaml
paths:
  trajectory_dir: "base_ws/src/control/trajectory_generator/traj_sample/lane_change_batch"
```

규칙:

1. 폴더 바로 아래의 `*.csv`를 파일명 순서로 모두 실행한다.
2. 하위 폴더는 탐색하지 않는다.
3. JSON, 이미지, 임시 파일은 실행 대상이 아니다.
4. trajectory 폴더가 비어 있으면 실행하지 않는다.
5. 실행 중 폴더 내용이 바뀌지 않는 것을 전제로 한다.

## 7. 속도별 시나리오 선택

차량은 각 trajectory의 시작 목표 속도와 같은 초기 속도로 시작해야 한다. 정지 상태의
시나리오를 로드한 뒤 고속 trajectory를 실행하는 fallback은 사용하지 않는다.

각 CSV에 대해 다음 순서로 시나리오를 결정한다.

1. CSV 첫 행의 `lanelet_id`를 `start_lanelet`으로 읽는다.
2. CSV 첫 행의 `velocity_mps`를 읽는다.
3. `speed_kph = velocity_mps * 3.6`으로 변환한다.
4. 정수 속도와 허용 오차 안에서 일치하는지 확인한다.
5. config의 `scenario.filename_pattern`으로 시나리오 이름을 만든다.

예:

```text
첫 lanelet_id = 200101
첫 velocity_mps = 27.777... m/s
speed_kph = 100
시나리오 = 200101_start_100.json
```

시나리오 JSON을 찾은 뒤 다음 값도 확인한다.

- `mapInfo.mapName`이 config의 `expected_map_name`과 일치
- ego 시작 위치가 해당 start lanelet의 시작 위치
- `egoVehicle.currentVelocity`가 `speed_kph`와 일치
- `egoVehicle.initPosition.vehicleSaveData.velocity`가 `speed_kph`와 일치

일치하는 시나리오가 없거나 속도가 다르면 batch 전체를 시작하지 않는다. 모든 trajectory와
시나리오의 대응 관계를 먼저 확인한 뒤 첫 차량을 움직인다.

속도별 시나리오 파일명 규칙이 다른 시험은 config의 `filename_pattern`만 다르게 지정한다.

## 8. 필수 스택

### 8.1 Batch 시작 시 한 번 실행

| 단축어 기준 | 실제 명령 | 역할 |
| --- | --- | --- |
| `rc` | `roscore` | ROS Master |
| `udp` | `roslaunch sensor_interface morai_udp_interface.launch` | Sensor/Ego UDP 수신 및 `/ctrl_cmd` 송신 |
| `maptf_morai_cor` | `roslaunch bringup maptf_morai_deg_correction.launch` | yaw 보정 MORAI GT localization과 `map -> base_link` |

`udp` 실행 시 config의 파일 경로를 전달한다.

```bash
roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:=<paths.sensor_json> \
  network_config:=<paths.network_json>
```

`udp`와 `maptf_morai_cor`는 trajectory마다 다시 실행하지 않고 batch 동안 유지한다.

현재 `maptf_morai_cor`가 포함하는 localization launch의 기본 Lanelet map은 K-City v45다.
따라서 사용자가 말한 highway가 K-City 내부 고속주로라면 그대로 사용할 수 있다. 별도 MORAI
highway map을 뜻한다면 해당 map의 TF 정렬과 Lanelet map을 사용하는 별도 localization
entrypoint가 필요하며, 그 경로를 확정하기 전에는 Lane Change runner를 구현하지 않는다.

### 8.2 각 trajectory마다 실행

| 단축어 기준 | 실제 명령 | 실행 시점 |
| --- | --- | --- |
| `traj_gen` | `roslaunch trajectory_generator trajectory_publisher.launch trajectory_csv:=<CSV>` | 시나리오 로드 후 |
| `traj_val` | `roslaunch trajectory_validator record.launch` | generator 준비 후 |
| `traj_fol_a` | `roslaunch trajectory_follower pp_pi_autoware.launch` | validator armed 후 |

batch에서는 RViz가 필요하지 않으므로 `traj_val`의 기록 기능인 `record.launch`를 직접 사용한다.
수동 시험에서 RViz도 필요하면 기존 `trajectory_validator.launch`를 별도로 사용할 수 있다.

## 9. MORAI에서 먼저 준비할 항목

현재 저장소 기준으로 Network와 Sensor JSON을 MORAI GUI에 로드하고 Connect하는 동작 자체는
ROS launch가 대신하지 않는다.

현재 기본 센서 파일 `configs/sensors/Sensor.json`은 `Sensor_5.json`과 같은 최신 실행
설정이며 GPS는 30 Hz로 설정되어 있다.

map별 batch 시작 전에 MORAI에서 한 번 수행한다.

1. Lane Change면 highway map, Obstacle Avoidance면 K-City map을 연다.
2. F4에서 config의 `network_json`을 로드하고 Connect한다.
3. F3에서 config의 `sensor_json`을 로드하고 Connect한다.
4. Ego를 `AV-ExternalCtrl`로 전환한다.

그 뒤 runner가 `udp`와 `maptf_morai_cor`를 실행하고 다음 topic을 확인한다.

```text
/Ego_topic
/CollisionData
/localization/kinematic_state
/ctrl_cmd
```

Sensor 전체 연결을 확인할 때는 다음 topic도 확인한다.

```text
/sensor/gps
/sensor/imu
/sensor/lidar1
/sensor/camera/front/compressed
```

## 10. Run 실행 순서

### 10.1 Batch 시작 전

1. config 읽기
2. map, trajectory, scenario, network, sensor 경로 확인
3. trajectory 폴더의 모든 CSV 목록 확정
4. 각 CSV의 시작 lanelet과 시작 속도 확인
5. 모든 CSV에 대응하는 속도별 시나리오 존재 여부 확인
6. `roscore`, `udp`, `maptf_morai_cor` 실행 또는 실행 상태 확인
7. MORAI Network/Sensor topic 확인

### 10.2 각 trajectory

1. 해당 CSV에 대응하는 시나리오 로드
2. `/Ego_topic`에서 초기 속도와 초기 위치 확인
3. `traj_gen` 실행
4. `/control/trajectory` 발행 확인
5. `traj_val` 실행
6. validator가 기록 대기 상태인지 확인
7. `traj_fol_a` 실행
8. validator가 목표 도착으로 기록을 완료할 때까지 대기
9. `traj_fol_a` 종료
10. 차량 정지 확인
11. `traj_val`과 `traj_gen` 종료
12. 다음 trajectory의 속도별 시나리오 로드

시나리오와 초기 속도가 확인되기 전에는 follower를 실행하지 않는다.

## 11. 시나리오 로드 방식

runner는 각 run 전에 선택된 시나리오 이름을 MORAI의 Scenario Load 인터페이스로 전달해야
한다.

현재 저장소에는 `morai_msgs/ScenarioLoad` 메시지와 Scenario Load 예제가 있지만, 현재 사용
중인 MORAI에서 실제로 연결된 topic 또는 service 이름과 응답 방식은 확인이 필요하다.

구현 전에 한 번 확인할 항목:

1. GUI에서 저장된 시나리오를 ROS/UDP 인터페이스로 다시 로드할 수 있는지
2. 파일 전체 경로를 받는지, 확장자를 제외한 시나리오 이름만 받는지
3. load 완료를 알리는 응답이 있는지
4. 시나리오 변경 후 Ego Network와 Sensor Connect가 유지되는지
5. 로드 직후 `AV-ExternalCtrl` 상태가 유지되는지

이 인터페이스가 확인되기 전에는 자동 batch 실행 코드를 작성하지 않는다.

## 12. 기록 위치

새로운 결과 형식을 만들지 않는다. `traj_val`이 저장하는 기존 기록을 그대로 사용한다.

```text
base_ws/src/control/trajectory_validator/history/
└── <trajectory_basename>/
    └── <run_id>/
        ├── tracking_samples.csv
        └── metadata.json
```

runner는 validator 기록이 완료됐는지만 확인한다. lateral error, heading error, velocity error
등의 분석은 이후 기존 `metadata.json`과 `tracking_samples.csv`를 대상으로 진행한다.

## 13. Map별 운영

### Lane Change

- MORAI highway map 사용
- highway 전용 trajectory 폴더 사용
- highway 전용 속도별 시나리오 폴더 사용
- highway lanelet map과 projector 사용
- `config/lane_change.yaml`로 실행

### Obstacle Avoidance

- MORAI K-City map 사용
- K-City 전용 trajectory 폴더 사용
- K-City 전용 속도별 장애물 시나리오 폴더 사용
- `lanelet2_map_v45.osm`과 `map_projector_info_v1.yaml` 사용
- `config/obstacle_avoidance.yaml`로 실행

map을 바꿀 때는 같은 runner 실행 안에서 전환하지 않는다. 현재 batch를 종료하고 MORAI map,
Network, Sensor를 준비한 뒤 다른 config로 새 batch를 시작한다.

## 14. 구현 순서

1. highway map, lanelet map, projector, 시나리오 폴더 경로 확정
2. Lane Change와 Obstacle Avoidance의 시나리오 파일명 규칙 확정
3. MORAI Scenario Load topic/service 확인
4. `config/lane_change.yaml`, `config/obstacle_avoidance.yaml` 작성
5. 하나의 `run_trajectory_batch.py` 작성
6. 하나의 `trajectory_batch_runner.launch` 작성
7. Lane Change CSV 1개로 실행 확인
8. Obstacle Avoidance CSV 1개로 실행 확인
9. 지정 폴더 전체 실행 확인

## 15. 미확정 사항

코드 구현 전에 아래 항목을 확인해야 한다.

1. 실제 K-City Obstacle Avoidance trajectory 전용 폴더
2. K-City Obstacle Avoidance 속도별 시나리오 폴더와 파일명 규칙
3. trajectory의 초기 속도 기준을 CSV 첫 점 `velocity_mps`로 사용하면 되는지
4. scenario의 `currentVelocity`와 `vehicleSaveData.velocity` 단위가 km/h가 맞는지
5. 실제 MORAI Scenario Load topic/service 이름과 성공 응답
6. 시나리오 재로드 후 Network/Sensor/ExternalCtrl 연결 유지 여부
7. trajectory 종료는 validator의 `goal_reached`만 사용하면 되는지

위 값이 정해지면 추가 기능 없이 이 문서의 패키지 구조와 실행 순서만 구현한다.
