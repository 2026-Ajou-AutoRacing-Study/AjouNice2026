# Planning 시나리오 자동 검증: 구현 계약과 검증 상태

## 목적

팀원이 만든 임의 MORAI JSON을 같은 풀스택으로 차례로 실행해, 대회에서 대응하지 못하는 상황을 찾는다. 실행 전 과도한 시나리오 생성 기능은 넣지 않는다. 팀원은 JSON 폴더와 캠페인 YAML의 목적지·선택 스택만 관리한다.

## 현재 구현

- `base_ws/src/planning_addons/planning_scenario_validator` ROS 패키지와 루트의 `planning_validator`, `planning_validator_dashboard` 진입점을 만들었다. 이 패키지는 제어기 구현이 아니라 planning 시나리오 검증·스택 orchestration이므로 `control/`에서 `planning_addons/`로 옮겼다.
- 시나리오 원본을 검증하고 MORAI MR SaveFile에 정지 ExternalCtrl 사본, 대응 Ego Network `_MN.json`을 설치한다. 기존 `trajectory_batch_runner`에서 검증한 69-byte ScenarioLoad UDP payload builder를 재사용한다. 일반 주행용 competition local JSON에는 ScenarioLoad 항목이 없다. `PUBSUB_TYPE=783`이 `/GetTrafficLightStatus`로 배정된 슬롯을 `ego_network_local.json`의 UDP 9095 ScenarioLoad 항목으로 교체한 전용 JSON을 설치한다. 이 전용 JSON을 MORAI GUI에서 최초 1회 Connect해야 한다. 두 783 항목을 중복 추가하는 방식은 사용하지 않는다.
- 공통 스택과 개별 시나리오 스택을 분리한다. 시나리오마다 localization, GT adapter, tracking, planning을 재시작해 이전 상태를 제거한다. BEVFusion inference 서버, UDP, sensing, Planviz는 한 캠페인 동안 유지한다.
- `/planning/set_waypoint_route`에 UUID가 있는 goal/waypoints를 요청하고, 같은 UUID의 route와 이후 trajectory가 도착해야 controller를 켠다. 대회 route requester는 꺼서 경로 공급자가 둘이 되지 않도록 한다.
- 실시간 subscriber가 localization, trajectory, collision을 관측한다. goal 반경/hold/speed, 충돌, timeout, topic loss를 판정한다.
- 시나리오별 rosbag과 X11 전체 화면 MP4를 **ScenarioLoad 전부터** 기록하고 `PASS/FAIL/ERROR` 폴더로 분류한다. 경로 거부·스택 시작 실패도 녹화 범위에 포함한다. `result.json`/`events.jsonl`/전체 `summary.csv`/대시보드를 생성한다.
- 종료되거나 Ctrl+C로 중단돼도 완료 결과를 유지한다. 같은 YAML의 resume는 PASS/FAIL을 건너뛰고 ERROR/미완료를 다시 시도한다.
- Google Drive 업로드는 `rclone` remote를 설정했을 때 각 결과 이후 수행한다. 실패해도 로컬 기록을 보존한다.

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

`catkin_make_isolated --install --pkg planning_scenario_validator` scoped build와 설치 후 `rospack find`/`roslaunch --nodes`, 현재 7개 unit test, `git diff --check`가 통과했다. 첫 live 연속·재개 시험의 스택 조합은 `od_gt + localization + 신호등 off`였다.

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
- BEVFusion 예제에서 raw 센서를 함께 녹화하면 1장면 약 3분에도 bag이 수 GB가 될 수 있다. 인지 재현이 불필요한 캠페인은 `include_sensors: false`로 설정한다.
- 성공은 goal 접근과 정차(또는 통과) 및 충돌 없음으로 판단한다. 공식 대회 판정과 동일하다는 뜻은 아니다.
- `rclone`이 없는 PC에서도 검증은 동작하지만 자동 업로드는 실패로 기록된다.

## 일반 정적 장애물 구간 재사용 계약

`config/examples/static_obstacle_{gt,bevfusion}.yaml`은 GPS **음영 구간 안**의 hidden mission이다. 일반 정적 장애물 구간과 구분한다. 일반 구간은 validator의 `scripts/build_static_corridor_cases.py`로 `planner_logs/validation_inputs/static_corridor/`에 재현 장면을 생성하고, `config/examples/static_mission_{gt,bevfusion}.yaml`로 같은 장면·goal·sensor를 비교한다. `soa_official_*`은 배포 샘플 CargoBox의 원래 위치와 scale을 기준으로 하고, 움직이는 NPC와 GPS 음영만 제거한다. `soa_corridor_*`은 Base3 OD 벤치마크의 합성 배치이므로 공식 샘플의 회피 가능 장면으로 간주하지 않는다.

동일 장면을 GT와 BEVFusion으로 비교할 때는 검출 유무뿐 아니라 `trajectory`의 속도·횡방향 경로, `ctrl_cmd`, `/CollisionData`와 실제 Ego 궤적을 함께 확인한다. PASS 한 번은 해당 장면의 폐루프 성공일 뿐 다른 객체·배치의 안전성 보장은 아니다. 개별 bag, 수치, 이미지와 분석 보고서는 Git 제외 경로 `planner_logs/analysis/<run_id>/`에 둔다.
