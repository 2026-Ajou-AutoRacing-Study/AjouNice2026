# Planning 시나리오 자동 검증: 구현 계약과 검증 상태

## 목적

팀원이 만든 임의 MORAI JSON을 같은 풀스택으로 차례로 실행해, 대회에서 대응하지 못하는 상황을 찾는다. 실행 전 과도한 시나리오 생성 기능은 넣지 않는다. 팀원은 JSON 폴더와 캠페인 YAML의 목적지·선택 스택만 관리한다.

## 현재 구현

- `base_ws/src/control/planning_scenario_validator` ROS 패키지와 루트의 `planning_validator`, `planning_validator_dashboard` 진입점을 만들었다.
- 시나리오 원본을 검증하고 MORAI MR SaveFile에 정지 ExternalCtrl 사본, 대응 Ego Network `_MN.json`을 설치한다. 기존 `trajectory_batch_runner`에서 검증한 69-byte ScenarioLoad UDP payload builder를 재사용한다. 일반 주행용 competition local JSON에는 ScenarioLoad 항목이 없다. `PUBSUB_TYPE=783`이 `/GetTrafficLightStatus`로 배정된 슬롯을 `ego_network_local.json`의 UDP 9095 ScenarioLoad 항목으로 교체한 전용 JSON을 설치한다. 이 전용 JSON을 MORAI GUI에서 최초 1회 Connect해야 한다. 두 783 항목을 중복 추가하는 방식은 사용하지 않는다.
- 공통 스택과 개별 시나리오 스택을 분리한다. 시나리오마다 localization, GT adapter, tracking, planning을 재시작해 이전 상태를 제거한다. BEVFusion inference 서버, UDP, sensing, Planviz는 한 캠페인 동안 유지한다.
- `/planning/set_waypoint_route`에 UUID가 있는 goal/waypoints를 요청하고, 같은 UUID의 route와 이후 trajectory가 도착해야 controller를 켠다. 대회 route requester는 꺼서 경로 공급자가 둘이 되지 않도록 한다.
- 실시간 subscriber가 localization, trajectory, collision을 관측한다. goal 반경/hold/speed, 충돌, timeout, topic loss를 판정한다.
- 시나리오별 rosbag과 X11 전체 화면 MP4를 기록하고 `PASS/FAIL/ERROR` 폴더로 분류한다. `result.json`/`events.jsonl`/전체 `summary.csv`/대시보드를 생성한다.
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

전용 프로필을 연결한 다음 live smoke에서는 ScenarioLoad와 localization·planning 노드 실행까지 성공했다. 하지만 예제 YAML의 goal을 MORAI 좌표 `(-47.607,-530.134)`로 적어 planning map 좌표계에서 `DefaultPlanner returned an empty route`가 났다. v85 Route A의 map goal `(-13.383,93.783)`로 수정했다. 로드 후 controller 준비 전 차량 움직임을 막기 위한 brake 송신도 추가했다. 이 실패는 `ROUTE_FAILURE`로 기록됐고 주행·녹화 이전에 끝났으므로 성공률에 섞지 않는다.

map goal 수정 후에는 Route와 controller가 실제 준비됐고, 실시간 감시기가 약 7.5초 뒤 vehicle ID 100의 `/CollisionData` 접촉을 검출했다. 첫 기록 마감 검사는 `.bag.active → .bag` rename보다 빨라 일시적으로 `RECORDING_FAILURE`가 됐지만, 생성된 42MB indexed bag과 MP4는 보존됐다. 후속 수정에서 rosbag 완료 파일을 최대 15초 기다리도록 했다.

최종 단일 장면 `soa_live_smoke_recording_261002`: `/CollisionData`에서 ID 100 접촉을 1회 판정해 `FAIL/COLLISION`으로 저장했다. 동일 basename의 indexed bag(18.2초, 17,684 messages, 약 62MB), 1920×1080 MP4(17.75초), JSON·events가 실제 생성됐다. 이 장면의 충돌은 자동화 실패가 아니라 기존 SOA 회피 문제를 드러낸 실제 판정이다.

연속·재개 시험 `soa_two_scene_resume_261002`: 첫 장면 PASS/SUCCESS(40.88초, 목표 최소거리 0.11m), 두 번째 장면 DRIVING 중 Ctrl+C로 MANUAL_ABORT 기록. 같은 YAML을 다시 실행하니 첫 장면은 건너뛰고 두 번째만 attempt 2로 실행되어 PASS/SUCCESS(33.09초, 최소거리 0.176m)했다. 최종 `summary.csv` 2/2 PASS, attempt 1/2, 이전 중단 증거는 `error/`에 보존됐다. 즉 scene 자동 전환·분리 bag·실시간 판정·중단 후 resume를 MR에서 실제 확인했다. 이 결과는 정적 장애물 회피 전체 시나리오의 일반적 성공률을 뜻하지 않는다.

`catkin_make_isolated --install --pkg planning_scenario_validator` scoped build와 설치 후 `rospack find`/`roslaunch --nodes`, 4개 unit test, `git diff --check`가 통과했다. 이 테스트들은 `od_gt + localization + 신호등 off` 조합의 검증이다. BEVFusion, 신호등 on, 원격 LAN MORAI, Google Drive 업로드는 구성만 제공하며 아직 live 검증하지 않았다.

## 알려진 제한

- 최초 Sensor/Ego Network JSON 선택과 Connect는 MORAI GUI에서 수동이다. 시나리오 전환은 UDP로 자동 수행한다.
- 시나리오별 goal이 다르면 `.case.yaml`을 둔다. JSON 자체에서 임의 목적지를 추측하지 않는다.
- `full.json`은 알려진 토픽 inventory이며 새/비표준 토픽은 수동 추가해야 한다. `rosbag record -a`는 디스크 위험 때문에 사용하지 않는다.
- 영상은 전체 X11 화면이다. Planviz를 최대화하고 화면 잠금을 끄는 운용이 필요하다.
- 성공은 goal 접근과 정차(또는 통과) 및 충돌 없음으로 판단한다. 공식 대회 판정과 동일하다는 뜻은 아니다.
- `rclone`이 없는 PC에서도 검증은 동작하지만 자동 업로드는 실패로 기록된다.
