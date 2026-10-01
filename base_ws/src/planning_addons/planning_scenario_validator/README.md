# Planning scenario validator

팀원이 만든 MORAI scenario JSON 폴더를 대상으로, 실제 localization → 인지 → tracking/prediction → planning → controller 폐루프 주행을 시나리오별로 검증한다. 검증기 전용 GUI는 없지만 MORAI MR의 최초 네트워크 Connect에는 MORAI GUI가 필요하다. 각 시나리오는 bag, 화면 영상, 실시간 판정 결과로 따로 저장한다.

## 5분 실행

1. 아래 `install.sh`를 한 번 실행한다. 그러면 `ego_network_validator_local.json`이 대회 MR의 `SaveFile/Network/25.S4.MolitComp03/`에도 설치된다. MORAI MR에서 K-City 맵을 열고, YAML에 적힌 Sensor JSON과 이 Ego Network JSON을 각각 로드해 Connect한다. MORAI GUI의 최초 Connect는 사용자가 한 번 한다. 시나리오 전환은 실행기가 자동으로 한다. 평소의 `ego_network_competition_local.json`을 대신 로드하면 ScenarioLoad가 없어 자동 전환이 실패한다.
2. 저장소 루트에서 실행한다.

```bash
bash base_ws/src/planning_addons/planning_scenario_validator/install.sh
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py
mkdir -p base_ws/src/planning_addons/planning_scenario_validator/config/users/sw
cp base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_mission_gt.yaml \
   base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/campaign.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/campaign.yaml --dry-run
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/campaign.yaml
```

다른 터미널의 대시보드:

```bash
./planning_validator_dashboard planner_logs/validation/soa_static_mission_gt_example
```

Ctrl+C 후 같은 명령을 다시 실행하면 완료된 시나리오를 건너뛰고 남은 시나리오를 진행한다. 실패 시나리오를 다시 보려면 `--rerun-failed`를 붙인다. 개별 오류가 났다면 `error/`에 증거를 남기고 다음 실행에서 재시도한다.

BEVFusion까지 켜는 시험은 별도 예제를 복사해서 사용한다. 먼저 무장애물 연결 확인용 `bevfusion_clear_smoke.yaml`을 권장한다. **일반 정적 장애물 구간**은 아래의 `static_mission_bevfusion.yaml`을 사용한다. `static_obstacle_bevfusion.yaml`과 `static_obstacle_gt.yaml`은 이름과 달리 GPS 음영 구간 안의 hidden mission regression이므로 이번 일반 구간과 혼동하지 않는다. MORAI GUI의 Sensor는 예제에 적힌 `Sensor_competition_local.json`, Ego Network는 validator 전용 JSON으로 연결해야 한다. BEVFusion checkpoint·Docker image도 `scripts/runtime/run_bevfusion.sh`의 계약대로 준비해야 한다.

```bash
cp base_ws/src/planning_addons/planning_scenario_validator/config/examples/bevfusion_clear_smoke.yaml \
   base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/bevfusion_campaign.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/bevfusion_campaign.yaml --dry-run
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/bevfusion_campaign.yaml
```

무장애물 예제와 일반 정적 장애물 예제는 원시 센서를 bag에서 제외한다. 음영 구간 regression 예제 `static_obstacle_bevfusion.yaml`만 원시 센서도 녹화하므로 약 3분에 bag이 수 GB일 수 있다. `recording.include_sensors: false`로 바꿔도 BEVFusion **추론 입력은 그대로 유지**된다. `FAIL/TIMEOUT` 같은 실제 주행 실패와 `ERROR/SYSTEM_ERROR` 같은 자동화·환경 오류는 별도로 분류한다.

일반 정적 장애물 구간의 두 CargoBox 변형을 생성·비교하려면 다음을 실행한다. 두 YAML은 **같은 scenario와 sensor/goal**을 사용하고 detector만 다르다. 원본 샘플의 CargoBox scale `2×3×2`와 비교용 native `1×1×1`을 포함하며, GPS 음영과 움직이는 NPC는 제거한다. 시나리오는 Git에 추적하지 않는 `planner_logs/validation_inputs/static_corridor/`에 생성된다.

```bash
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py
cp base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_mission_bevfusion.yaml \
   base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/static_mission_bevfusion.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/static_mission_bevfusion.yaml --dry-run
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/users/sw/static_mission_bevfusion.yaml
# GT 비교도 static_mission_gt.yaml을 같은 방식으로 복사하되 campaign_name을 다르게 둡니다.
```

빌더의 `soa_corridor_*`는 Base3 **OD 벤치마크용 합성 배치**다. 단일 CargoBox도 planning에서 정지하거나 충돌할 수 있으므로 이를 공식 샘플의 회피 가능 장면으로 취급하지 않는다. 일반 정적 장애물 구간의 기본 검증은 `soa_official_*`로 시작한다.

다른 장애물 종류와 배치에 대한 **비공식 스트레스 테스트**는 `static_catalog_gt.yaml`과 `static_catalog_bevfusion.yaml`로 실행한다. 두 설정은 같은 `soa_catalog_*` 8개 장면(PE_Drum, TrafficBarrel, Concrete_Barricade01, PE_Firewall_Orange 각각 2개 위치)을 고른다. `sample_pose`는 배포 샘플 CargoBox의 위치이며, `pathward_080`은 무장애물 주행 경로 쪽으로 x를 0.8m 이동시킨 위치다. 다른 asset을 그 위치에 놓은 것이므로 대회 출제 장면·공식 성공률로 해석하지 않는다. GT 실행에서 배치 자체가 회피 가능한지 먼저 보고, 동일 장면을 BEVFusion으로 비교한다.

```bash
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_catalog_gt.yaml --dry-run
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_catalog_gt.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_catalog_bevfusion.yaml
```

두 장애물을 연속으로 피해 가는 **비공식 지그재그 스트레스 테스트**는 `static_pair_mission_gt.yaml`/`static_pair_mission_bevfusion.yaml` 예제를 사용한다. PE_Drum과 TrafficBarrel을 약 17m 간격으로 배치한 한 장면(`soa_pair_05*`)을 선택하며, 두 물체가 각각 단독으로도 통과한 배치다. GT와 BEVFusion을 같은 장면·goal로 비교한다. 빌더가 함께 만드는 `soa_pair_01`~`04`와 `soa_pair_control_*`은 실패 원인을 찾기 위해 사용한 실험 장면이며 기본 실행 예제에는 포함하지 않았다. 여러 장애물 결과를 단일 물체 성공률에 합산하지 않는다.

```bash
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_pair_mission_gt.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_pair_mission_bevfusion.yaml
```

두 명령은 동시에 실행하지 않는다. 각각 스택을 자체 실행·종료하며, 첫 명령이 끝난 뒤 두 번째를 실행한다.

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

목적지·경유점은 MORAI scenario JSON의 로컬 좌표가 아닌 planning의 `map` 좌표로 입력한다. 일반 정적 구간 예제 goal은 v85 지도에서 장애물 약 25m 뒤의 차로 중심이다. 장면마다 다른 지점에서 끝내려면 `.case.yaml`로 지도 좌표 목적지를 지정한다.

## 녹화 및 판정

- `basic.json`: camera/LiDAR 제외, planning/인지/제어/GT 핵심 토픽
- `lightweight.json`: 위치, trajectory, command, collision 중심
- `full.json`: 알려진 센서·planning 토픽 목록. 실제 수신하지 않는 항목은 rosbag에 들어가지 않는다.
- 영상은 X11 전체 화면을 시나리오마다 ffmpeg로 녹화한다. 기본값 `focus_planviz: true`는 녹화 직전 Planviz를 전면·최대화하며 ROS1 EOL 팝업을 띄우지 않는다. 다른 창에 포커스를 주면 전체 화면 영상에도 그 창이 찍힌다.
- 충돌은 `/CollisionData`의 객체 ID를 실시간으로 읽는다. 한 시나리오에서 같은 ID의 접촉 메시지가 끊겼다 재등장해도 1개 객체로만 센다. goal 반경, 속도 및 유지 시간으로 성공을 판단한다. 지정 시간 미도달은 TIMEOUT이다.
- `result.json`은 개별 증거용, `summary.csv`는 사람에게 보여주기 위한 전체 표, `events.jsonl`은 시간 순서 감사 기록이다.
- 녹화는 ScenarioLoad 전에 시작한다. Route 요청이 거부돼도 bag·MP4·이벤트가 남는다. `elapsed_sec`은 주행 판정 시간이고, 새 결과의 `wall_elapsed_sec`은 준비와 정리를 포함한 전체 실행 시간이다.

출력 예:

```text
planner_logs/validation/soa_static_mission_gt_example/
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
- BEVFusion 실행기에는 YAML의 Sensor JSON을 그대로 전달한다. 모델 준비뿐 아니라 실제 detection 토픽의 새 메시지가 확인돼야 route/controller 단계로 넘어간다.
- 신호등 off는 신호 인지 노드와 planner 소비 모듈을 함께 끈다.
- 실행기에서 최초 MORAI GUI Sensor/Ego Network Connect는 자동화하지 않는다. topic 준비 검사를 통해 연결 누락을 찾는다.
- `install.sh`는 기존 대회용 Ego UDP 설정의 `PUBSUB_TYPE=783` 슬롯을 검증된 UDP ScenarioLoad(9095)로 바꾼 전용 JSON을 생성한다. 이 슬롯의 기존 `/GetTrafficLightStatus` UDP 요청 기능은 전용 프로필에서 사용하지 않으며, 카메라 기반 신호등 인지 노드와는 별개다. 기존 `ego_network_competition_local.json`은 수정하지 않는다. GUI에서 전용 JSON을 로드해야 한다.
- 현재 영상은 Planviz 창 단독 캡처가 아니라 전체 X11 화면이다. 다른 창이 덮이면 영상에도 찍힌다.
- 새 ROS 패키지로 `roslaunch planning_scenario_validator validator.launch config:=...`를 쓰려면 workspace를 한 번 빌드해야 한다. 저장소 루트 `./planning_validator`는 별도 패키지 빌드 없이 실행할 수 있다.

개발 상태와 남은 실차 검증은 [설계·검증 기록](../../../../docs/planning/scenario_validator_automation.md)을 참고한다.
