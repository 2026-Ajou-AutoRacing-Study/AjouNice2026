# Planning scenario validator

팀원이 만든 MORAI scenario JSON 폴더를 대상으로, 실제 localization → 인지 → tracking/prediction → planning → controller 폐루프 주행을 시나리오별로 검증한다. GUI는 필요 없다. 각 시나리오는 bag, 화면 영상, 실시간 판정 결과로 따로 저장한다.

## 5분 실행

1. 아래 `install.sh`를 한 번 실행한다. 그러면 `ego_network_validator_local.json`이 대회 MR의 `SaveFile/Network/25.S4.MolitComp03/`에도 설치된다. MORAI MR에서 K-City 맵을 열고, YAML에 적힌 Sensor JSON과 이 Ego Network JSON을 각각 로드해 Connect한다. MORAI GUI의 최초 Connect는 사용자가 한 번 한다. 시나리오 전환은 실행기가 자동으로 한다. 평소의 `ego_network_competition_local.json`을 대신 로드하면 ScenarioLoad가 없어 자동 전환이 실패한다.
2. 저장소 루트에서 실행한다.

```bash
bash base_ws/src/planning_addons/planning_scenario_validator/install.sh
mkdir -p base_ws/src/planning_addons/planning_scenario_validator/config/users/sw
cp base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_obstacle_gt.yaml \
   base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/campaign.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/campaign.yaml --dry-run
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/campaign.yaml
```

다른 터미널의 대시보드:

```bash
./planning_validator_dashboard planner_logs/validation/soa_team_example_01
```

Ctrl+C 후 같은 명령을 다시 실행하면 완료된 시나리오를 건너뛰고 남은 시나리오를 진행한다. 실패 시나리오를 다시 보려면 `--rerun-failed`를 붙인다. 개별 오류가 났다면 `error/`에 증거를 남기고 다음 실행에서 재시도한다.

## 팀원이 해야 할 일

시나리오 폴더에 `.json`을 넣는다. 공통 goal을 쓴다면 YAML의 `default_case.goal`을 한 번만 설정한다. 목적지가 다를 때만 같은 이름의 `.case.yaml`을 추가한다.

```text
scenarios/
  soa_narrow_gap.json
  soa_narrow_gap.case.yaml
```

```yaml
description: 좁은 S자 통과
goal: {x: -13.382585757, y: 93.782754520, z: 28.489, yaw_deg: 60.815}
waypoints: []
timeout_sec: 180
arrival_mode: stop
```

`scenario_install_dir: auto`는 `~/MoraiLauncher_Lin/`의 대회 MR을 선택한다. 팀원 PC에서 경로가 다르면 해당 값을 직접 지정한다. 실행기는 원본 JSON을 수정하지 않고 MR SaveFile에 ExternalCtrl/정지 시작 사본과 `_MN.json`을 설치한다.

목적지·경유점은 MORAI scenario JSON의 로컬 좌표가 아닌 planning의 `map` 좌표로 입력한다. 예제 goal은 v85 지도 Route A 끝점에서 가져왔다. 장면마다 다른 지점에서 끝내려면 `.case.yaml`로 지도 좌표 목적지를 지정한다.

## 녹화 및 판정

- `basic.json`: camera/LiDAR 제외, planning/인지/제어/GT 핵심 토픽
- `lightweight.json`: 위치, trajectory, command, collision 중심
- `full.json`: 알려진 센서·planning 토픽 목록. 실제 수신하지 않는 항목은 rosbag에 들어가지 않는다.
- 영상은 X11 전체 화면을 시나리오마다 ffmpeg로 녹화한다. Planviz를 최대화하면 원하는 화면이 저장된다.
- 충돌은 `/CollisionData`의 새 contact ID를 실시간으로 읽는다. goal 반경, 속도 및 유지 시간으로 성공을 판단한다. 지정 시간 미도달은 TIMEOUT이다.
- `result.json`은 개별 증거용, `summary.csv`는 사람에게 보여주기 위한 전체 표, `events.jsonl`은 시간 순서 감사 기록이다.

출력 예:

```text
planner_logs/validation/soa_team_example_01/
  campaign_state.json
  summary.csv
  stack_logs/
  success/soa_.../
    planbag_soa_..._P_20261002_123456.bag
    planbag_soa_..._P_20261002_123456.mp4
    planbag_soa_..._P_20261002_123456.json
    events.jsonl
  failure/...
  error/...
  running/...
```

`upload.enabled: true`와 `upload.rclone_remote`를 설정하면 시나리오 종료 직후 해당 결과 폴더를 `rclone copy`한다. 로컬 기록은 업로드 실패와 무관하게 보존한다. 사용자는 자신의 Google Drive remote를 미리 설정해야 한다.

## 운영 계약

- 실행 전 ROS graph에 기존 `/ctrl_cmd`, `/planning/trajectory`, localization, detector publisher가 있으면 중복 실행을 막는다. 기존 수동 스택을 종료한 뒤 사용한다.
- custom goal은 `/planning/set_waypoint_route`로 보낸다. 대회 Route A/B requester는 실행하지 않는다.
- `pose_source: localization`은 MR `/Competition_topic` + -1.347° 보정을 사용한다. `maptf_morai`는 `/Ego_topic`을 사용한다.
- `detector: od_gt`는 `/Object_topic`이 필요하다. `bevfusion`은 카메라 4대·LiDAR가 있는 Sensor JSON과 기존 BEVFusion image/checkpoint가 필요하다.
- 신호등 off는 신호 인지 노드와 planner 소비 모듈을 함께 끈다.
- 실행기에서 최초 MORAI GUI Sensor/Ego Network Connect는 자동화하지 않는다. topic 준비 검사를 통해 연결 누락을 찾는다.
- `install.sh`는 기존 대회용 Ego UDP 설정의 `PUBSUB_TYPE=783` 슬롯을 검증된 UDP ScenarioLoad(9095)로 바꾼 전용 JSON을 생성한다. 이 슬롯의 기존 `/GetTrafficLightStatus` UDP 요청 기능은 전용 프로필에서 사용하지 않으며, 카메라 기반 신호등 인지 노드와는 별개다. 기존 `ego_network_competition_local.json`은 수정하지 않는다. GUI에서 전용 JSON을 로드해야 한다.
- 현재 영상은 Planviz 창 단독 캡처가 아니라 전체 X11 화면이다. 다른 창이 덮이면 영상에도 찍힌다.
- 새 ROS 패키지로 `roslaunch planning_scenario_validator validator.launch config:=...`를 쓰려면 workspace를 한 번 빌드해야 한다. 저장소 루트 `./planning_validator`는 별도 패키지 빌드 없이 실행할 수 있다.

개발 상태와 남은 실차 검증은 [설계·검증 기록](../../../../docs/planning/scenario_validator_automation.md)을 참고한다.
