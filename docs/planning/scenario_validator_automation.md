# Planning 시나리오 자동 검증: 구현 계약과 검증 상태

## 목적

팀원이 만든 임의 MORAI JSON을 같은 풀스택으로 차례로 실행해, 대회에서 대응하지 못하는 상황을 찾는다. 실행 전 과도한 시나리오 생성 기능은 넣지 않는다. 팀원은 JSON 폴더와 캠페인 YAML의 목적지·선택 스택만 관리한다.

## 현재 구현

### 신호등 Python 선택

신호등 on/off와 별개로 GUI에서 `시스템 Python` / `가상환경 Python`을 선택한다. YAML의 `stack.traffic_light_runtime`은 `system|virtualenv`이고, 기존 `traffic_light_python`만 지정한 YAML도 호환한다. 기본 system은 `/usr/bin/python3` 및 기존 사용자 패키지 방식이며, virtualenv에서는 사용자 site-packages를 차단한다. `traffic_light_python`의 `~`는 각 사용자의 홈 폴더로 해석하고, 실행 파일의 venv symlink는 해제하지 않는다. `traffic_light_device`는 GPU 번호 또는 `cpu`다. 기존 YAML을 열고 저장하는 것만으로 환경 키를 추가하거나 resume 해시를 바꾸지 않는다.

공용 설치기는 `scripts/runtime/setup_traffic_light_env.sh`다. 기본 prefix는 `~/.local/share/ajounice/traffic_light_gpu`, Python 3.11 / Torch 2.7.1 cu128 / TorchVision 0.22.1 cu128이며 inference dependencies는 기존 `requirements-runtime.txt`를 재사용한다. 기존 Conda 실행 파일을 사용하거나, 없으면 checksum 검증한 Miniforge를 홈 폴더에 설치한다. 시스템 Python·기존 환경·bashrc는 변경하지 않는다. 설치기가 관리하지 않는 기존 prefix는 덮어쓰지 않는다. `--check`는 설치 없이 공용 버전/모델/GPU/ROS 메시지를 검사한다.

신호등 on이면 캠페인별 실제 실행 전에 `check_traffic_light_env.py`로 선택한 Python의 실제 CUDA 연산·YOLO 추론·ROS import를 검사하고, 실패/120초 timeout은 시스템 오류로 중단한다. 로그는 `stack_logs/traffic_light_preflight.log`에 남긴다. 설정 검사/dry-run은 경로·YAML 검사이며 GPU를 사용하지 않는다. signal decision parameters, 별도 system-Python 지도 매칭, 기본 예제 off는 유지한다.

공용 환경 신규 설치 및 재실행 검사, 실제 ROS CompressedImage → GPU YOLO → TrafficLightGroupArray smoke를 **로컬 5060 Ti**에서 검증했다. CUDA 아키텍처 목록에는 `sm_75/86/90/120`이 있으나 2080 Ti/3090/4090 실기 검증을 완료한 뜻은 아니다. 신호등 색 판단·A/B/C/D 실주행 성공률은 이번 환경 smoke의 검증 범위가 아니다. 최초 설치 및 GUI 사용은 [팀원 가이드](SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md)의 7-1을 참고한다.

### 자동화 스택과 녹화

- `base_ws/src/planning_addons/planning_scenario_validator` ROS 패키지와 루트의 `planning_validator`, `planning_validator_dashboard` 진입점을 만들었다. 이 패키지는 제어기 구현이 아니라 planning 시나리오 검증·스택 orchestration이므로 `control/`에서 `planning_addons/`로 옮겼다.
- 시나리오 원본을 검증하고 MORAI MR SaveFile에 정지 ExternalCtrl 사본, 대응 Ego Network `_MN.json`을 설치한다. 기존 `trajectory_batch_runner`에서 검증한 69-byte ScenarioLoad UDP payload builder를 재사용한다. 일반 주행용 competition local JSON에는 ScenarioLoad 항목이 없다. `PUBSUB_TYPE=783`이 `/GetTrafficLightStatus`로 배정된 슬롯을 `ego_network_local.json`의 UDP 9095 ScenarioLoad 항목으로 교체한 전용 JSON을 설치한다. 이 전용 JSON을 MORAI GUI에서 최초 1회 Connect해야 한다. 두 783 항목을 중복 추가하는 방식은 사용하지 않는다.
- 공통 스택과 개별 시나리오 스택을 분리한다. 시나리오마다 localization, GT adapter, tracking, planning을 재시작해 이전 상태를 제거한다. BEVFusion inference 서버, UDP, sensing, Planviz는 한 캠페인 동안 유지한다.
- `/planning/set_waypoint_route`에 UUID가 있는 goal/waypoints를 요청하고, 같은 UUID의 route와 이후 trajectory가 도착해야 controller를 켠다. 대회 route requester는 꺼서 경로 공급자가 둘이 되지 않도록 한다.
- 실시간 subscriber가 localization, trajectory, collision을 관측한다. goal 반경/hold/speed, 충돌, timeout, topic loss를 판정한다.
- 시나리오별 rosbag과 X11 전체 화면 MP4를 **ScenarioLoad 전부터** 기록하고 `PASS/FAIL/ERROR` 폴더로 분류한다. 경로 거부·스택 시작 실패도 녹화 범위에 포함한다. `result.json`/`events.jsonl`/전체 `summary.csv`/대시보드를 생성한다.
- 종료되거나 Ctrl+C로 중단돼도 완료 결과를 유지한다. 같은 YAML의 resume는 PASS/FAIL을 건너뛰고 ERROR/미완료를 다시 시도한다.
- Google Drive 업로드는 캠페인 YAML의 `upload.enabled`, `upload.profile`(`basic`/`bag`/`video`/`full`), `upload.rclone_remote`로 고른다. 기본 경로는 `ajou_drive:planning_validator`이며 맵 스크립트와 같은 공유 `AjouNice2026` 폴더 ID를 rclone 명령에 전달한다. 따라서 실제 위치는 `AjouNice2026/planning_validator/<campaign_name>/...`이다. 각 결과 이후 선택한 파일과 캠페인 `summary.csv`·설정·상태 JSON을 공유하고, `success/failure/error` 구조를 유지한다. bag·MP4를 제외해도 로컬에는 원본을 보존한다. 실패는 주행 결과와 구분해 기록한다. 현재 기본값은 업로드 꺼짐이며 실제 Drive 전송은 아직 검증하지 않았다. 기존 빈 경로 개인 YAML은 GUI에 제안 경로가 보여도 업로드를 켜기 전까지 원본을 유지한다.

## 검증 게이트

1. YAML과 scenario/map/network/topic profile/goal의 정적 검사 및 dry-run.
2. Python compile과 핵심 순수 함수 검사.
3. 설치된 ROS 메시지·서비스와 launch 인자, ScenarioLoad payload 형식 확인.
4. 실제 MR에서 빈 ROS graph로 한 시나리오 주행, bag index/MP4 재생/종료 사유 검증.
5. 두 시나리오 연속 실행 및 강제 중단 후 resume 검증.

4·5는 MORAI GUI와 폐루프 주행이 필요한 live 게이트이며, 아래 기록으로 통과 범위를 남긴다.

2026-10-02 첫 live smoke는 Ego Network에 ScenarioLoad가 없는 일반 주행용 JSON을 연결해 35초 시나리오 위치 대기에서 실패했다. 실패는 `error/`로 분류되고 재개 상태가 보존됨을 확인했다. 후속 수정에서 전용 네트워크 프로필을 생성하고 YAML 검사 시 UDP 9095 항목 존재를 강제했다. 전용 JSON을 GUI에서 Connect한 뒤 재검증이 필요하다.

전용 프로필을 연결한 다음 live smoke에서는 ScenarioLoad와 localization·planning 노드 실행까지 성공했다. 하지만 예제 YAML의 goal을 MORAI 좌표 `(-47.607,-530.134)`로 적어 planning map 좌표계에서 `DefaultPlanner returned an empty route`가 났다. v85 Route A의 map goal `(-13.383,93.783)`로 수정했다. 이후 녹화 시작을 ScenarioLoad 전으로 옮겨, 경로 거부도 성공률에는 넣지 않으면서 bag·영상 증거는 남긴다.

map goal 수정 후에는 Route와 controller가 실제 준비됐고, 실시간 감시기가 약 7.5초 뒤 vehicle ID 100의 `/CollisionData` 접촉을 검출했다. 첫 기록 마감 검사는 `.bag.active → .bag` rename보다 빨라 일시적으로 `RECORDING_FAILURE`가 됐지만, 생성된 42MB indexed bag과 MP4는 보존됐다. 후속 수정에서 rosbag 완료 파일을 최대 15초 기다리도록 했다.

최종 단일 장면 `soa_live_smoke_recording_261002`: `/CollisionData`에서 ID 100 접촉을 1회 판정해 `FAIL/COLLISION`으로 저장했다. 동일 basename의 indexed bag(18.2초, 17,684 messages, 약 62MB), 1920×1080 MP4(17.75초), JSON·events가 실제 생성됐다. 이 장면의 충돌은 자동화 실패가 아니라 기존 SOA 회피 문제를 드러낸 실제 판정이다.

연속·재개 시험 `soa_two_scene_resume_261002`: 첫 장면 PASS/SUCCESS(40.88초, 목표 최소거리 0.11m), 두 번째 장면 DRIVING 중 Ctrl+C로 MANUAL_ABORT 기록. 같은 YAML을 다시 실행하니 첫 장면은 건너뛰고 두 번째만 attempt 2로 실행되어 PASS/SUCCESS(33.09초, 최소거리 0.176m)했다. 최종 `summary.csv` 2/2 PASS, attempt 1/2, 이전 중단 증거는 `error/`에 보존됐다. 즉 scene 자동 전환·분리 bag·실시간 판정·중단 후 resume를 MR에서 실제 확인했다. 이 결과는 정적 장애물 회피 전체 시나리오의 일반적 성공률을 뜻하지 않는다.

`catkin_make_isolated --install --pkg planning_scenario_validator` scoped build와 설치 후 `rospack find`/`roslaunch --nodes`, 당시 7개 unit test, `git diff --check`가 통과했다. 첫 live 연속·재개 시험의 스택 조합은 `od_gt + localization + 신호등 off`였다.

추가 live 게이트(2026-10-02): 고의로 map 밖 goal을 지정한 `soa_route_failure_evidence_261002`에서 `FAIL/ROUTE_FAILURE`를 재현했다. 수정 후에는 경로가 없어도 indexed bag 17.3초·5,006메시지, MP4 17.1초, `SCENARIO_LOAD_REQUEST → SCENARIO_READY → ROUTE_REQUEST` 이벤트가 남았다.

BEVFusion 포함 `soa_bevfusion_smoke_01`의 첫 시도에서는 이전 장면과 동일한 Ego 시작 XY를 새 ScenarioLoad 성공으로 0.1초 만에 잘못 인정했다. 실제 리로드는 약 1.8초 뒤였고, 그 사이 제어 전 Ego가 움직여 GPS 음영 구간에 진입해 `(0,0)` GPS 때문에 localization이 초기화되지 않았다. 이 원인을 rosbag의 Ego 위치/GPS 시계열로 확인했다. 보강한 검증기는 이전 Ego가 예상 출발점 근처라면 ScenarioLoad 후 Ego 스트림의 재시작 간격을 관측해야 완료로 인정한다. Controller가 켜지기 전까지는 `/ctrl_cmd` brake를 계속 발행한다.

같은 캠페인 재시도는 정상 ScenarioLoad(약 1.5초), 정상 GPS, localization, BEVFusion, tracking/prediction, planning, controller까지 실제 연결됐다. E25 모델의 detection **9.26 Hz**, tracking **30.0 Hz**, planning trajectory **10.03 Hz**가 bag에 남았다. 약 219초, 2.80 GB의 indexed bag(4대 카메라·LiDAR 포함)과 약 219초 MP4가 정상 마감됐다. 이 장면은 180초 주행 제한 내 목적지에 못 가 `FAIL/TIMEOUT`(충돌 0, 최소 목표 거리 116.432 m)이며, **풀스택 실행 성공을 주행 성공으로 해석하지 않는다**. 이는 본 도구의 첫 BEVFusion 검증 실패 사례로 보존한다. 신호등 on, 원격 LAN MORAI, Google Drive 업로드는 여전히 live 미검증이다.

무장애물 연결 확인 `bevfusion_clear_video_probe_261002`은 BEVFusion·tracking/prediction·planning·controller를 켜고 `PASS/SUCCESS`로 목적지에 도착했다(주행 29.12초, 충돌 0, 최소 목표 거리 0.051m). indexed bag 약 93.9MB와 MP4 약 59초가 남았고, detection 525건의 latency는 평균 148.9ms, p50 131.8ms, p95 222.3ms였다. 영상에는 네 카메라와 Planviz 경로가 나왔다. 단, 이 시도의 Planviz 전면 배치는 자동 보정 실패 후 수동 최대화했으므로 영상 자동화의 단독 증거로 삼지 않는다. 이후 `soa_route_failure_evidence_261002` 재실행에서 수동 개입 없이 Planviz 자동 최대화와 MP4 기록을 확인했다. 이 무장애물 PASS는 센서·모델·판단·제어 연결 확인이지 정적 장애물 회피 성공률은 아니다.

ScenarioLoad 패킷은 `trajectory_batch_runner.morai_udp_client.build_scenario_load_payload`를 재사용한다. dataset 수집 PoC의 UDP builder와 같은 요청으로 출력한 패킷이 **69바이트 전체에서 byte-identical**함을 확인했다. dataset 쪽 builder는 대형 실험 스크립트 안에 있고 입력 검사가 적다. 본 도구는 batch runner의 독립된 packet builder를 쓰고, 필요한 3회 재시도와 Ego 위치 확인은 validator에서 수행한다. dataset PoC 전체를 planning 런타임에 의존시키지 않는다.

## 알려진 제한

- 최초 Sensor/Ego Network JSON 선택과 Connect는 MORAI GUI에서 수동이다. 시나리오 전환은 UDP로 자동 수행한다.
- 시나리오별 goal이 다르면 `.case.yaml`을 둔다. JSON 자체에서 임의 목적지를 추측하지 않는다.
- `full.json`은 알려진 토픽 inventory이며 새/비표준 토픽은 수동 추가해야 한다. `rosbag record -a`는 디스크 위험 때문에 사용하지 않는다.
- 영상은 전체 X11 화면이다. 첫 BEVFusion smoke의 MP4를 실제 프레임으로 확인했을 때 RViz EOL 팝업과 미최대화 창이 영상을 가렸다. 후속 수정에서 ROS1 EOL 팝업 비활성화와 Planviz 자동 전면·최대화를 추가했다. 재녹화 프레임에서 팝업 없이 최대화된 Planviz를 확인했다. 다만 화면 잠금·다른 창의 가림은 여전히 피해야 한다.
- 시작 위치가 이전 장면과 같으면 Ego UDP 스트림의 재시작 간격을 ScenarioLoad 확인 신호로 사용한다. 매우 특이한 통신 장애가 동일한 현상을 만들 수 있으므로, 이것을 MORAI의 명시적 성공 ACK로 간주하지 않는다.
- 모든 배포 예제의 녹화 기본값은 `basic.json` + `include_sensors: false` + 화면 영상 켜짐이다. BEVFusion 추론은 센서 제외 녹화에서도 그대로 수행한다. 인지 재현이 필요해 raw 센서를 함께 녹화하면 1장면 약 3분에도 bag이 수 GB가 될 수 있다.
- 성공은 goal 접근과 정차(또는 통과) 및 충돌 없음으로 판단한다. 공식 대회 판정과 동일하다는 뜻은 아니다.
- `rclone`이 없는 PC에서도 검증은 동작하지만 자동 업로드는 실패로 기록된다.

## 일반 정적 장애물 구간 재사용 계약

`config/examples/static_obstacle_{gt,bevfusion}.yaml`은 GPS **음영 구간 안**의 hidden mission이다. 일반 정적 장애물 구간과 구분한다. 일반 구간은 validator의 `scripts/build_static_corridor_cases.py`로 `planner_logs/validation_inputs/static_corridor/`에 재현 장면을 생성하고, `config/examples/static_mission_{gt,bevfusion}.yaml`로 같은 장면·goal·sensor를 비교한다. `soa_official_*`은 배포 샘플 CargoBox의 원래 위치와 scale을 기준으로 하고, 움직이는 NPC와 GPS 음영만 제거한다. `soa_corridor_*`은 Base3 OD 벤치마크의 합성 배치이므로 공식 샘플의 회피 가능 장면으로 간주하지 않는다.

빌더의 `soa_catalog_*`은 배포 샘플과 동일한 정적 구간 출발점·목적지에 Base3 카탈로그 장애물 4종을 한 개씩 배치한 **비공식 스트레스 테스트**다. 각각 샘플 CargoBox 위치와 무장애물 Ego 경로 방향으로 x를 0.8m 이동한 위치를 사용한다. `config/examples/static_catalog_{gt,bevfusion}.yaml`은 두 인지 소스에서 같은 8개 장면을 선택한다. 장애물 유형·배치가 바뀌므로 CargoBox 공식 샘플 결과와 성공률을 섞지 않는다.

`soa_pair_05*`는 같은 일반 구간에 PE_Drum과 TrafficBarrel을 약 17m 간격으로 엇갈리게 놓은 비공식 지그재그 스트레스 장면이다. 두 장애물이 같은 출발점·목적지에서 각각 단독으로도 GT 통과한 배치이며, `config/examples/static_pair_mission_{gt,bevfusion}.yaml`에서 선택한다. GT 입력이 이미 실패한다면 BEVFusion 실패를 모델 문제로 단정하지 않고 planning·배치·경로 조건부터 분리한다.

빌더는 `soa_pair_01`~`04`와 `soa_pair_control_*`도 생성한다. 빠른 연결 확인용 `static_pair_mission_{gt,bevfusion}.yaml`은 통과 배치 `05`만 선택하지만, 실패를 찾는 `static_pair_stress_{gt,bevfusion}.yaml`은 `01`~`05`를 모두 실행한다. 단일 배치도 실패하면 두 장애물의 상호작용 때문이라는 추론은 성립하지 않는다. 개별 실행 근거와 영상은 각 PC의 Git 제외 `planner_logs/`에 남긴다.

실패 분석은 주행 중 구독한 Ego 이동거리·속도, trajectory 앞부분 속도, `/ctrl_cmd`, 목표 거리와 해당 장면의 스택 로그를 사용한다. `TIMEOUT`을 미출발, 주행 후 정지, 목표 반경에 왔으나 정차/유지 실패, 기타 목적지 미도달로 나누고, 결과 JSON과 CSV에 증거를 남긴다. `suspected_stage`는 원인 확정이 아닌 조사 우선순위다. 로그는 캠페인 공용 파일이므로 장면 시작 시 바이트 위치를 기록해 이전 장면의 경고가 섞이지 않게 한다. 실패 분석 자체가 오류를 내더라도 bag/영상/원래 판정을 보존한다.

분류 기준은 의도적으로 보수적이다. `NO_DEPARTURE`는 출발점 기준 최대 이동 ≤0.5m와 최고 속도 ≤0.5m/s가 동시에 충족될 때만 붙인다. `MID_DRIVE_STOP`은 출발점에서 0.5m 이상 이동한 후, 종료 시 속도 ≤0.3m/s·최근 5초 이동 ≤0.5m·마지막 유의미한 속도 이후 ≥3초인 경우다. 목적지 반경에는 왔으나 정차·유지 조건을 채우지 못했다면 `ARRIVAL_CONDITION_UNMET`으로 별도 표시한다. 관측값이 부족하면 `INSUFFICIENT_OBSERVATION`으로 남긴다. planner가 0속도 궤적을 냈다는 관측은 `planning_output` 조사 우선순위를 줄 수 있지만, 그 판단이 잘못된 이유가 perception·map·planning 중 무엇인지는 GT/BEV 대조 및 bag 분석 없이는 확정하지 않는다. 모든 결과의 `root_cause_confirmed`는 false다.

동일 장면을 GT와 BEVFusion으로 비교할 때는 검출 유무뿐 아니라 `trajectory`의 속도·횡방향 경로, `ctrl_cmd`, `/CollisionData`와 실제 Ego 궤적을 함께 확인한다. PASS 한 번은 해당 장면의 폐루프 성공일 뿐 다른 객체·배치의 안전성 보장은 아니다. 개별 bag, 수치, 이미지와 분석 보고서는 Git 제외 경로 `planner_logs/analysis/<run_id>/`에 둔다.

## 2026-10-02 GUI 진입점

### 팀원 사용 계약과 설정 복원

미션별 Git 추적 스타터는 `base_ws/src/planning_addons/planning_scenario_validator/config/missions/`에 있다. `roundabout`, `static_obstacle`, `dynamic_obstacle`, `gps_shadow`, `highway` 각각 `base/`와 `workspace/` 두 폴더로 나눈다. base에는 `example.yaml`과 24개 시나리오·case sidecar·manifest를 제공한다. workspace는 `.gitignore`만 공유하고 YAML·시나리오는 Git 제외한다. GUI에서 base를 선택해 `베이스 → 내 작업 폴더 복사`를 누르면 기존 시나리오를 덮어쓰지 않고 작업 사본과 새 캠페인 YAML을 만든다. 미션 YAML 저장/복사는 workspace를 사용하고 기존 config/users 개인 파일은 그대로 유지한다. 예전 config/examples/missions 경로를 쓰는 기존 YAML은 파일 내용·resume 해시 변경 없이 새 base 위치로 읽기 호환 처리한다.

GUI 예제 목록에 포함하며 기본 탐지기는 BEVFusion이다. 총 120개는 새 live 검증을 완료한 결과가 아니라 **재현 가능한 오프라인 검증 입력**이다. 일반/기준선 16개와 stress 8개를 구분하고, 주최측 답변과 검증용 가정·미검증 배우 동작을 별도 README/manifest에 명시했다. 음영은 끼어들기 없이 정적 회피 중심이다. 기존 샘플과 학습 데이터 생성기는 수정하지 않았다.

목적은 항상 성공하는 시나리오 생성이 아니라, 팀원이 준비한 다양한 시나리오에서 시스템이 대응하지 못하는 상황을 찾고 실패 근거를 보존하는 것이다. 자동 실행·녹화·실시간 판정·resume 구조는 이 목적에 맞지만, 첫 사용자가 네트워크 연결과 목적지를 준비하는 과정은 여전히 수동이다.

- 배포자는 미션별 캠페인 YAML과 네트워크 설정을 제공하고, 팀원은 시나리오 JSON 폴더와 목적지·제한시간을 준비한다. 풀스택 YAML을 처음부터 작성하도록 요구하지 않는다.
- 같은 구간에서 객체 배치만 바뀌면 공통 목적지·시간을 사용한다. 다른 장면만 같은 이름의 `.case.yaml`을 둔다. Sidecar 값은 캠페인 기본값보다 우선한다.
- 예제는 개인 YAML로 복사하며, 미실행 개인 YAML은 같은 이름으로 수정한다. 실행 이력이 있는 캠페인은 설정 변경 시 새 이름으로 복사한다. 설정 검사·실행 버튼도 먼저 저장한다.
- 마지막으로 열거나 저장한 YAML 경로만 `~/.config/ajounice/planning_scenario_validator/gui_settings.json`에 기억한다 (`XDG_CONFIG_HOME` 설정 시 해당 경로 사용). 재시작 시 해당 YAML을 불러오고, 삭제되었거나 읽을 수 없으면 기본 예제로 돌아간다. 미저장 입력값·창 위치·모니터 모드·실패 재실행 체크는 복원하지 않는다. 개인 YAML과 GUI 설정은 Git에 포함하지 않는다.
- 미션 템플릿과 시나리오 폴더를 선택하는 ‘새 캠페인 만들기’ 마법사는 후속 후보이며 아직 구현하지 않았다. 팀원 1명이 안내만으로 실행해 보면서 준비 과정의 막힘을 확인하는 것이 다음 사용성 검증이다.

### 짧은 제한시간과 전환 대기

배포 예제의 기본 주행 제한시간은 catalog 35초, 일반 정적 장애물·pair·clear 50초, 음영 구간 75초로 줄였다. 생성기의 장면별 override는 clear 40초, pair 45~50초, 복잡한 corridor 65~90초 등이다. 생성된 sidecar를 쓰면 생성기를 다시 실행해야 새 값이 적용된다. 개인 YAML도 유형별 상한으로 낮추되 더 짧은 값은 유지했다. 기존 결과는 보존하고 변경된 설정은 새 캠페인으로 실행하며, 과거 결과의 제한시간을 소급 변경하지 않는다.

현재 대기 1(주행 종료→다음 ScenarioLoad, 약 7~8초)과 대기 2(ScenarioLoad→주행 시작, GT 약 25초·BEVFusion 약 27초)는 유지한다. 대기 1에는 녹화 마감·스택 정리·결과 저장이, 대기 2에는 새 스택과 해당 장면 route·비어 있지 않은 궤적 준비 확인이 포함된다. 임의의 고정 sleep만 있는 것이 아니며 readiness timeout을 낮춰도 정상 준비가 빨라지지는 않는다. 모든 시간이 필수라고 확정한 것은 아니다. 특히 대기 2의 서비스 준비·route 계산·궤적 생성 시간을 분리 측정한 뒤 최적화하며, 이전 장면 궤적 재사용이나 녹화 마감 생략으로 줄이지 않는다.

후속 화면 구성 변경: 모니터 모드의 GUI 폭은 작업 영역의 절반, 즉 왼쪽 RViz 창과 동일하게 설정한다. 하단 바 높이는 `bottom_bar_px: auto`이면 작업 영역 높이의 약 17%(170~275 px)이고, 100~400 px 정수 또는 `"18%"`처럼 10~30%로 직접 지정할 수도 있다. GUI와 RViz가 같은 설정을 해석하며, GUI는 창 관리자의 실제 바깥 창 크기(테두리 포함)로 맞춘다. `전체 설정 보기` 버튼을 상태 문구보다 위에 배치하고 긴 상태 문구를 한 줄로 잘라 고정 높이 안에 남도록 했다. 최대화된 GUI에서 전환할 때는 X11 창 관리자의 최대화 해제 완료를 기다린 뒤 위치·크기를 적용한다. `manual` 영상 설정은 장면마다 RViz를 다시 최대화하므로, GUI 시작 시 이를 안내하고 `split` 변경 여부를 묻는다. 아래의 실제 X11 배치 수치는 이 변경 **이전**의 전체 폭 모니터 바 검증 결과다.

GUI 최소 높이를 위해 비율·고정 px 입력의 실제 예약 높이는 최소 140 px다. 실제 X11에서 임시 Tk 창에 바깥 크기 450×150 px을 지정하여 창 관리자가 보고하는 크기가 정확히 450×150 px인지 확인했다. Tk의 `_NET_WM_PID`는 이 환경에서 0이므로, 복수 GUI 창이 있을 때도 정확한 창을 찾도록 제목에 프로세스 ID를 포함한다. 이 검증은 GUI 창 자체의 배치이며 RViz·MORAI를 포함한 전체 split 영상 재검증은 별도다.

추가 실주행 영상에서 `window_layout: split`, `focus_planviz: false`인데도 첫 장면 녹화 시작 후 7초까지 RViz가 저장된 1251px 폭으로 돌아간 것을 확인했다. `focus_planviz` 최대화 경로가 아니라 RViz 초기화 중 저장된 `QMainWindow` 크기 복원과 외부 배치의 순서 경쟁으로 판단한다. 첫 장면의 배치를 RViz 창 발견 후 6초 늦추고, 이후 크기 유지 여부를 검사·재적용하도록 변경했다. 첫 장면 녹화 시작은 그만큼 늦어지며 이후 장면에는 초기화 대기를 적용하지 않는다. 실제 X11에서 수정된 배치 helper가 RViz 924×906px, MORAI 924×1053px 목표를 유지함을 확인했지만, 새 코드를 적용한 전체 캠페인 MP4는 아직 재검증하지 않았다.

현재 `split` 배치는 왼쪽 위 RViz와 왼쪽 아래 GUI만 세로 공간을 나눠 사용한다. 오른쪽 MORAI에는 더 이상 하단 여백을 남기지 않고 작업 영역 전체 높이를 준다. 과거의 양쪽 903px 배치 수치는 아래에 이전 검증 기록으로 남긴다.

`./planning_validator_gui` (alias `pvgui`)는 Tkinter로 예제 YAML을 개인 설정에 복사·편집하고, 동일한 CLI 검증기 프로세스를 실행/중단/resume한다. ROS graph 제어·분석·녹화 로직은 GUI로 이중화하지 않았다. GUI는 `campaign_state.json`을 주기적으로 읽어 완료 결과와 현재 단계·시간·목적지 거리·속도·충돌 횟수를 표시한다. `static_pair_mission_gt_split.yaml` 예제로 시작하면 실행 중 GUI를 하단의 얇은 모니터 바로 접고, RViz 왼쪽·MORAI 오른쪽 자동 배치를 시도한다. 영상은 세 창이 놓인 X11 화면 전체를 녹화한다. 창 자동 배치가 안 되면 경고만 남기고 주행·녹화는 계속한다. `전체 설정 보기`로 GUI를 펼칠 수 있지만 영상에도 찍힌다. GUI 버튼과 실행 터미널 Ctrl+C 모두 안전 중단 입력이다. MORAI GUI에서 Sensor/Ego Network Load·Connect는 여전히 수동이다. 개인 YAML은 Git에서 제외되며, 이미 실행된 캠페인의 설정을 바꿀 때는 새 캠페인 이름으로 복사할지 묻는다. 사용법은 패키지 README를 우선 참조한다.

후순위 영상 실험: 현행 ScenarioLoad 전 녹화 시작을 유지한다. 센서 제외 성공 녹화에서는 bag 약 50 MiB/분, MP4 약 10 MiB/분이므로 영상 트리밍은 용량 최적화의 우선 대상이 아니다. 영상 on/off의 CPU·MORAI·BEVFusion 주기 A/B를 먼저 측정하고, 필요할 경우에만 새 궤적 준비 직후 영상 시작을 시험한다. 궤적이 생성되지 않는 실패에는 스크린샷을 남겨야 한다. 종료 결과 문구와 약 1초 후행 영상을 원한다면 GUI에 결과가 표시된 것을 확인한 뒤 recorder를 닫도록 순서를 바꿔야 한다. 현재는 결과 저장·GUI 갱신이 recorder 종료 뒤에 일어나므로 1초 대기만 넣어서는 문구가 영상에 남지 않는다. 어느 개선도 아직 적용하지 않았다.

후속 GUI 견고화: 진행표는 행을 지우고 다시 만들지 않고 제자리에서 갱신해 선택된 실패 장면의 상세 보기 상태를 보존한다. CLI 종료 코드 0이어도 FAIL 결과가 있으면 GUI에서 주행 실패 건수를 명확히 표시한다. `기존 FAIL도 다시 실행`은 기존 CLI의 `--rerun-failed`에만 연결된다. 실행·설정 검사 중에는 다른 YAML을 열거나 저장해 resume 해시를 바꾸지 못하도록 막는다. 탐지 토픽의 메시지 수와 객체가 실제 포함된 메시지 수를 따로 기록한다. GUI 가짜 실행기 화면 시험과 MORAI MR local GT 성공·실패 장면, BEVFusion+Planviz+MP4 조합의 실제 GUI 폐루프 및 indexed bag을 확인했다. 동일 조합을 반복했을 때 PASS와 FAIL이 모두 나와, 실패를 화면 녹화 부하만으로 설명할 수 없다. 전체 화면 녹화 중 다른 앱이 앞에 뜨면 영상도 그 앱을 기록하는 제약이 있다. 일회성 수치와 bag 경로는 `planner_logs/analysis/scenario_validator_gui_261002/report.md`에만 둔다.

실제 X11 창 배치 검증: MORAI `Simulator`와 Planviz RViz 창을 동시에 띄우고 `arrange_split_windows.py 150`을 실행했다. 이 PC의 1920×1080 화면에는 왼쪽 Dock·상단 Panel이 있어 작업 영역이 `(72,27,1848×1053)`이다. 작업 영역 기준으로 배치한 뒤 RViz `(72,27,924×903)`, MORAI `(996,27,924×903)`, GUI 모니터 바 `(72,931,1848×149)`로 서로 겹치지 않았다. MORAI 창은 세로축만 최대화된 상태였고, 이를 해제하지 않으면 크기 변경이 거부됐다. 자동 배치는 축별 최대화 해제와 GDK 작업 영역 사용을 포함한다. 이 검증은 **창 배치만** 확인한 것으로, split 화면 MP4를 포함한 새 폐루프 캠페인 재실행까지 확인한 것은 아니다.
