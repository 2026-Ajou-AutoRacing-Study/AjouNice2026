# AW149 아주나이스 통합 인계서

대상 브랜치: `planning/autoware-universe-to-ros1`. `develop`과 다른 브랜치는 건드리지 않는다. 상세 topic·frame·기능 gate는 [BASE-WS-INTEGRATION-GUIDE.md](BASE-WS-INTEGRATION-GUIDE.md)를 함께 따른다.

PointPainting PP1을 planning과 함께 실행하거나 통합 문제를 진단할 때는
[POINTPAINTING-PLANNING-CODEX-HANDOFF.md](POINTPAINTING-PLANNING-CODEX-HANDOFF.md)를
먼저 읽는다. 이 문서에는 detector/downstream Python 환경 분리, production launch,
v62 지도·모델 bundle·AW149 install 결과물의 구분과 중복 실행 방지 계약이 정리되어
있다.

## 지금 바로 가능한 것

- `planning_ws/src`에는 ROS1 포팅 경계 149개와 지원 5개가 있으며 provider 소스도 저장소 안에 있다.
- 중복 Autoware 메시지 14개는 base에서 제거했고 planning install을 단일 메시지 소유자로 쓴다.
- base의 새 메시지 소비 코드(Duration/Time), tracker 설치, traffic-light helper/weights, v62 경로, follower 입력을 수정했다.
- `scripts/workspace/setup_ws.sh`가 submodule 초기화 후 base의 과거 build/devel/install을 정리하고, planning underlay, pin된 Livox SDK, base 25개를 순서대로 빌드한다. 증분 빌드는 `build_all.sh`, 재현용 clean 빌드는 `build_all.sh --clean`이다.
- 공식 runtime overlay는 `base_ws/install_aw149` 하나다. `base_ws/devel` 또는 과거 addon install을 source하지 않는다.
- 저장소 내부 runtime 경로는 `source_aw149.sh`가 설정하는 `AJOUNICE_ROOT`를 기준으로 하므로 checkout이 `$HOME/AjouNice2026`가 아니어도 된다.
- `ajounice_planning_integration/morai_planning_stack.launch`가 localization,
  sensor TF, traffic light, v62 binary map, Mission→BPP→BVP→MVP graph를 묶는다.
  이 full-stack launch가 object tracking/prediction을 기본 소유한다.
  `tracking_prediction.launch`는 standalone 진단용이며 동시에 실행하지 않는다.
- detector checkpoint는 Git에 없으므로 detector와 controller는 안전상 기본 `false`다. route requester 노드는 기본 기동하지만 `auto_request_initial_route=false`라 RViz의 명시적 Route A 요청 전에는 route 서비스를 호출하지 않는다.

현재 로컬 증거는 planning source audit PASS, planning 154/154와 base 25/25 clean install, planning 설치 ELF 230개 링크 감사, install runtime 자산 전수 감사, 현재 지원하는 설치본 launch 19/19 파싱, object pipeline 21 tests, tracker 14 tests, tracking/prediction evaluation 29 tests, trajectory-validator 9 tests, traffic contract 2 tests, requester policy 8 tests와 ROS transport 11 tests다. base clean 결과물의 전체 회귀는 25/25 target, JUnit 114 tests가 오류·실패·스킵 없이 통과했다. MORAI GT localization은 실제 주행 bag의 이동 구간 250개 표본에서 odometry/acceleration 동일 stamp와 acceleration 값 보존을 확인했다. operation-mode `/system`→`/api` relay도 ROS transport 검사를 통과했다. v61 통합 graph는 24개 노드가 외부 입력 없이 25초 동안 process death/FATAL 없이 기동했고 map loader와 traffic model load까지 통과했다. 최신 clean build와 launch 결과는 [machine receipt](evidence/planning-clean-integration-r002.json)에 고정했다.

추가로 설치된 AjouNice `morai_planning_stack.launch`를 대상으로 한 output-only probe가 direct service 1/1과 production `competition_route_requester` 2/2를 통과했다. requester가 실제 Mission service로 Route A를 설정하고 47개 route lanelet과 연속된 최종 trajectory/validator 출력을 확인해 `VERIFIED(route_a)`로 전이했다. 각 실행의 최신 trajectory는 111점이고 validator 20개 flag가 모두 유효했으며 process death는 0이었다. full graph 기동부터 requester 검증까지 약 24.5~25.5초가 걸려 기존 5초 trajectory 관찰 창을 30초로 조정했으며, 연속 3개 fresh validated output 조건은 그대로 유지했다. 이는 통합 launch와 운영 requester의 입력부터 trajectory까지 배선을 확인한 결과이며 실제 MORAI 입력 의미나 controller 폐루프 결과는 아니다. [machine receipt](evidence/integrated-output-probe-r002.json)에 세 raw result SHA와 판정 경계를 기록했다.

그 뒤 v61 순환 경로의 실제 Mission 안전검사까지 포함한 합성 이동 A→B 검사를 2/2로 추가 통과했다. Route A 검증 후 odometry를 `wp_200178`, 2 m/s로 공급했을 때 requester가 Route B 57 lanelets를 수락하고 약 0.73초 뒤 161점 최종 trajectory와 20/20 validator flag를 확인해 `VERIFIED(route_b)`로 끝났으며 process death는 0이었다. 기존 original-first 공통구간 탐색은 Route B 후반의 재방문 lanelet 200889를 먼저 골라 공통 길이를 0 m로 오판했으므로, 원본 주석의 target-front 계약에 맞춰 target-first 탐색으로 최소 수정했다. requester는 독립 callback의 최신값 race를 피하도록 trajectory와 그 이후 validator stamp를 일회성으로 짝지으며 기존 post-route fence·freshness·전 flag·연속 3개 조건을 유지한다. 관찰 창은 부하 편차를 수용하도록 60초이며 유효성 기준을 낮추지 않았다. [r003 machine receipt](evidence/integrated-output-probe-r003.json)에 수정 전후 선택 index, A→B/fallback 네 raw SHA와 판정 경계를 기록했다. 이 검사는 순간이동한 합성 pose가 2 m/s를 보고한 결과이지 시작점부터 연속 주행한 폐루프 증거는 아니다.

같은 현재 artifact에서 합성 A→fallback도 2/2 통과했다. 종전 `200360 s=8 m` cutoff는 2 m/s에서도 공통 길이 26.751 m가 Mission 최소 안전거리 30 m보다 짧아 구조적으로 거절됐다. 안전기준을 낮추지 않고 cutoff와 explicit suffix를 한 lanelet 앞의 `201446 s=8 m`/28 lanelets로 옮겼다. 결과는 약 0.52~0.62초 뒤 151점 trajectory, validator 20/20, `VERIFIED(fallback_route)`, process death 0이다. requester source-age는 canonical Planning Validator latency 1.0초와 일치시키되 controller 안전정지 0.5초는 그대로 유지했다.

실제 운영 설정 `route_a_to_b_with_fallback`의 중재도 두 갈래로 확인했다. 정상 trigger에서는 status가 A 검증→B dispatch/검증으로만 진행해 fallback 호출이 0이었고, B가 채택되지 않은 채 cutoff에 도달한 상태에서는 A 검증→fallback dispatch/검증으로만 진행해 B 호출이 0이었다. 각각 161점/151점 trajectory와 validator 20/20, process death 0이다. [r004 machine receipt](evidence/production-route-arbitration-r004.json)에 두 상태 이력과 raw SHA를 고정했다. 이는 ambiguous/timeout B RPC 직후 fallback 복구 증거는 아니다.

명시적 B 안전거절 뒤 fallback 복구는 별도 fault injection으로 2/2 통과했다. B trigger에서 기존 Mission 안전 공식을 확실히 발동시키기 위해 합성 속도 1000 m/s를 사용했으며 제품 안전거리·route·requester 설정은 바꾸지 않았다. 두 번 모두 `VERIFIED A → DISPATCHED B → REJECTED B → DISPATCHED fallback → VERIFIED fallback`, 거절 직후 `active=route_a`, `uncertain=false`, 최종 151점과 validator 20/20, process death 0이었다. [r005 machine receipt](evidence/production-route-rejection-recovery-r005.json)는 이 값이 실제 차량 조건이 아니라 분기 fault injection임을 명시한다. RPC timeout/ambiguous 결과는 설계상 자동 fallback하지 않고 operator reconciliation을 요구하므로 이 검사 범위가 아니다.

최신 하네스 SHA 하나로 정상 B, B 미채택 cutoff fallback, B 명시적 안전거절 후 fallback의 운영 3분기를 다시 묶은 최종 로컬 output-only 결과는 [r006 release matrix](evidence/production-route-release-matrix-r006.json)다. 모든 분기에서 target route와 fresh trajectory, validator 20/20, process death 0을 확인했다. 이 matrix가 친구 컴퓨터의 실제 MORAI 입력·차량 설정·controller 폐루프 승인 단계를 대신하지는 않는다.

같은 checkpoint와 v61 fixture의 최종 P7.1 standalone graph는 11/11을 통과했다. Route A/B, BVP 양 모드, 안전/점유 차선 LC, 교차로, red→green, 정적 장애물 정지, moving ego, 2초 입력 복구, 안전 규약을 유지한 reroute를 Mission→validator 동일 graph에서 검증했다. r002 시점 source·map·projector·route hash 일치도 다시 확인했다. 상세 범위와 제외 항목은 [P7-STANDALONE-ACCEPTANCE.md](P7-STANDALONE-ACCEPTANCE.md)와 [machine receipt](evidence/p7-standalone-reconciliation.json)에 있다.

현재 source audit은 loop reroute 수정까지 포함한 checkpoint r011을 기준으로 3,999개 planning source와 6,589개 provider source의 해시·누락·ignore·중복을 통과했다. 이어 r011 현재 소스에서 planning 154개 `run_tests`를 패키지별 fresh ROS master/ROS_HOME과 `-j1`로 전면 재실행했다. 154/154 타깃과 실제 JUnit 253개·1,288 testcase가 errors/failures/skips/malformed 0이었다. [r003 all-package receipt](evidence/planning-test-audit-r003.json)에 현재 checkpoint와 재현 스크립트 SHA를 고정했다. packages without declared tests 67개는 타깃 완료일 뿐 testcase coverage로 확대하지 않는다.

실제 객체가 든 `sensor_cam_raw_obstacle1.bag`으로 oracle/evaluation adapter→map-frame tracking→prediction도 실행했다. v61 map-based 경로는 15초 동안 tracking 699 frame/5,396 object, prediction 511 frame/3,733 object를 처리했고 모든 prediction object가 finite 위치와 현재 pose 포함 31-point/3.0초 horizon path를 가졌다. 기본 CTRV도 3,124 object 전부 같은 길이의 path를 생성했다. 다만 이것은 생산 detector/tracker 대신 MORAI GT를 쓴 transport/runtime 증거이며, map-based는 50 Hz 원본의 모든 frame을 보존하지 않았다.

Controller 입력 freshness는 receipt time뿐 아니라 trajectory/state `header.stamp`도 검사하도록 보강했다. ROS transport 시험은 fresh 전진, 과거·미래·0 stamp trajectory의 반복 수신 중 full-brake stop, fresh 입력 복구를 검증한다. 이는 planner 내부가 cached trajectory를 계속 내는 현상을 제거하는 것이 아니라 controller handoff에서 오래된 source를 거부하는 동등 안전 계약이다. 실제 MORAI clock pause/reset과 0.5초 임계값 승인은 여전히 폐루프 gate다.

이 증거는 **실제 입력으로 trajectory가 나온다거나 폐루프 주행이 승인됐다는 뜻은 아니다.** 외부 입력이 없던 기동에서는 route, ego/TF, objects, acceleration, operation mode를 기다리는 것이 정상 상태였다.

## 새 컴퓨터에서 첫 순서

```bash
git checkout planning/autoware-universe-to-ros1
git submodule update --init --recursive
sudo apt install ros-noetic-jsk-recognition-msgs ros-noetic-robot-localization \
  python3-nose python3-pytest

# CUDA/CPU에 맞는 PyTorch 2.4.1을 먼저 설치한 뒤
python3 -m pip install -r \
  base_ws/src/perception/traffic_light_detection/requirements-runtime.txt

bash scripts/workspace/setup_ws.sh
source scripts/workspace/source_aw149.sh
```

`source_aw149.sh`가 실패하면 낡은 devel로 우회하지 말고 build/install 실패부터 해결한다. `rospack find ajounice_planning_integration`, `rospack find ekf_multi_object_tracking`, Python message `__file__`이 모두 `install_aw149` 또는 planning `install_isolated` 아래를 가리켜야 한다.

차량 치수는 upstream test YAML을 운영값으로 복사하지 않는다. 기본값은 `configs/planning/vehicle_info.yaml`이며, 저장소의 IONIQ5 mesh 측정 기록과 controller 설정을 대조한 flat YAML이다:

```text
wheel_base, wheel_tread, front_overhang, rear_overhang,
left_overhang, right_overhang, vehicle_height,
wheel_radius, wheel_width, max_steer_angle
```

rear-axle `base_link`에 맞춰 rear track을 쓰고 body width에서 좌우 overhang을 계산했다. 설정 provenance 단위검사와 실제 launch parameter load는 통과했지만, competition 차량의 track/tire/최대 tire angle은 폐루프 전에 담당자가 재확인한다. 필요하면 `vehicle_param_path:=...`로 승인본을 덮어쓴다.

정적 launch 확인 예:

```bash
roslaunch ajounice_planning_integration morai_planning_stack.launch \
  start_controller:=false --nodes
```

clean 설치본의 24개 필수 노드 생존과 v61 binary map 수신을 격리된 master에서 재검사하려면 `bash scripts/runtime/smoke_planning_stack.sh`를 실행한다. 25초 재검사 결과는 [machine receipt](evidence/clean-install-runtime-smoke-r001.json)에 있다. 이는 외부 입력을 주입하지 않는 기동 검사이므로 trajectory 생성이나 폐루프 합격으로 승격하지 않는다.

설치된 통합 launch에서 합성 입력을 넣고 Route A부터 최종 trajectory와 validator까지 재검사하려면 direct 기준선은 `bash scripts/runtime/verify_integrated_planning_output.sh`, 현재 제한 인지 기준선은 `AW149_OUTPUT_PROBE_LIMITED_PERCEPTION=1 bash scripts/runtime/verify_integrated_planning_output.sh`, production requester까지 포함한 검사는 `AW149_OUTPUT_PROBE_USE_REQUESTER=1 bash scripts/runtime/verify_integrated_planning_output.sh`를 실행한다. production 정상 B 분기는 `AW149_OUTPUT_PROBE_USE_REQUESTER=1 AW149_OUTPUT_PROBE_ROUTE_CASE=route_a_to_b_with_fallback AW149_OUTPUT_PROBE_TRANSITION_SPEED_MPS=2.0 bash scripts/runtime/verify_integrated_planning_output.sh`, cutoff fallback 분기는 같은 명령에 `AW149_OUTPUT_PROBE_FORCE_FALLBACK=1`을 추가한다. requester 검사는 각 route의 `VERIFIED`와 연속 3개 fresh validated output을 요구한다. 모두 output-only 배선 검사이며 실제 MORAI 센서 의미나 controller를 대신하지 않는다.

명시적 B 거절 뒤 fallback 회귀는 production 명령에 `AW149_OUTPUT_PROBE_FORCE_B_REJECTION=1`을 추가한다. 기본 1000 m/s는 안전거절 분기만 결정적으로 발동시키는 비물리적 테스트 입력이다. 하네스는 B의 `REJECTED`, `active=route_a`, `uncertain=false`를 먼저 확인한 뒤 2 m/s cutoff fallback과 최종 validator를 요구한다.

base 전체 회귀는 `bash scripts/workspace/test_base.sh`로 실행한다. 현재 clean build에서 25/25 package test target, catkin 집계 114 tests가 모두 통과했다. [machine receipt](evidence/base-clean-test-r001.json)는 이 검사가 planning 149 자체의 전체 테스트 수가 아니라 base overlay 회귀임을 구분한다.

FastBEV 또는 PointPainting을 켜려면 checkpoint 파일을 별도로 배포하고 SHA256을 release 문서와 대조한 뒤 `start_detector:=true`와 해당 경로 인자를 넘긴다. 현재 Git clone만으로 detector까지 켤 수 있다고 가정하지 않는다.

planning 입력 topic의 publisher/type만 먼저 보려면 `scripts/runtime/check_topics.sh --types-only`, 실제 메시지 흐름까지 보려면 `scripts/runtime/check_topics.sh`를 실행한다. Route 이후 출력까지는 `--full`, BoundaryDeparturePrevention에 필요한 C2 입력까지는 `--controller`를 추가한다. 옵션은 함께 쓸 수 있다. 하나라도 없거나 타입이 다르거나 기본 5초 안에 메시지가 없으면 nonzero로 종료한다. 이 검사는 축·단위·내용 의미까지 승인하지 않으므로 아래 담당별 동적 확인도 계속 필요하다.

현재 clean graph에서 `--types-only --controller`를 실행한 결과는 13개 중 6개 publisher/type 확인, 7개 `NOPUB`다. 빠진 것은 occupancy, ground-free cloud, system mode와 source가 없어 생성되지 않은 API relay 출력, C2 3개다. [machine receipt](evidence/integration-topic-contract-r001.json)의 `PASS` 6개도 메시지 흐름을 확인한 결과가 아님에 주의한다.

## base_ws 담당자에게 받을 입력

| 담당 | topic/type | 현재 판단과 필요한 조치 |
|---|---|---|
| Localization | `/localization/kinematic_state` `nav_msgs/Odometry` | 이름은 맞음. map pose/base_link twist, stamp, m/s·rad/s 검증 |
| Localization | `/localization/acceleration` `geometry_msgs/AccelWithCovarianceStamped` | MORAI GT launch가 `/Ego_topic.acceleration`을 동일 stamp/base_link frame으로 직접 공급. 실 MORAI에서 축·단위·중력 포함 여부는 동적 확인 필요 |
| Perception | `/perception/object_recognition/objects` `autoware_perception_msgs/PredictedObjects` | 변환/직렬화는 수정됨. 실제 detector→tracker→predictor frame, age, rate 확인 |
| Perception | `/perception/obstacle_segmentation/pointcloud` `sensor_msgs/PointCloud2` | **AjouNice 제한 프로파일에서 명시적 미사용.** empty cloud를 발행하지 않으며 cloud 기반 DetectionArea/MVP obstacle stop·slowdown·limiter와 validator collision checker를 실행하지 않음 |
| Perception | `/perception/occupancy_grid_map/map` `nav_msgs/OccupancyGrid` | **AjouNice 제한 프로파일에서 명시적 미사용.** empty/free grid를 발행하지 않으며 parking/freespace와 occupancy occlusion module을 실행하지 않음 |
| System | `/system/operation_mode/state` `autoware_adapi_v1_msgs/OperationModeState` | 실제 제어권 상태와 연결. 통합 relay가 `/api`에도 동일 메시지를 전달. 상시 autonomous fixture 금지 |
| Vehicle/Control | predicted trajectory, `autoware_control_msgs/Control`, `autoware_vehicle_msgs/SteeringReport` | BoundaryDeparture MVP를 켤 때 실제 controller rollout과 조향 피드백 필요 |
| Vehicle | flat vehicle-info YAML | 실제 IONIQ5 치수와 base_link 기준점 승인 필요 |

신호 출력은 `/perception/traffic_light_recognition/traffic_signals`, 최종 planning 출력은 `/planning/trajectory`, follower 입력도 `/planning/trajectory`, MORAI 명령은 `/ctrl_cmd`다. Lanelet map은 integration binary loader만 `/map/vector_map`을 발행해야 하며 base visualizer의 raw OSM payload와 동시에 켜지 않는다.

## 실행 승인 순서

1. Controller와 requester를 끈 상태에서 actual sensing/localization/perception/system topic의 type, MD5, frame, timestamp, age, rate, TF를 기록한다.
2. requester를 켜고 차량을 공식 시작 구간에 배치한 뒤 RViz `Route Control` 버튼으로 Route A를 요청한다. 기존 VERIFIED A를 갱신할 때도 공식 start pose 3 m 이내, 속도 0.1 m/s 이하 조건을 지키고 새 route stamp와 trajectory/validator 검증 완료를 확인한다. Route A 검증 뒤 Route B/fallback은 자동 전환되며 Mission→BPP→BVP→MVP 각 stage와 `/planning/trajectory`, `/planning/validation_status` 전체 이력을 기록한다.
3. 신호 red/green/unknown, **FastBEV PredictedObjects 기반** obstacle stop/cruise, 안전한 LC와 막힌 LC 거부, 실제 이동 중 A→B 및 fallback을 개별 검증한다. LiDAR pointcloud 기반 stop/slowdown과 occupancy 기반 occlusion·parking/freespace는 이 프로파일의 합격 범위가 아니다.
4. 위 단계가 통과한 뒤에만 controller를 켜 `/ctrl_cmd` 폐루프를 검증한다.
5. input loss, TF loss, stale data, clock reset, route rejection에서 안전 정지와 복구를 확인한다.

각 run에는 route UUID, map/vehicle-config hash, 준비 시간, trajectory Hz와 최대 gap, source age/latency, validator invalid 전 구간을 남긴다. 마지막 한 sample이 valid이거나 노드가 살아 있다는 사실만으로 PASS 처리하지 않는다.

현재 인계 상태는 **아주나이스 저장소 내부에서 build/install 및 output-only 통합 시험을 시작할 수 있는 상태**다. 실제 MORAI 입력과 검토 차량값이 필요한 기능·폐루프 gate는 이 컴퓨터에서 임의 fixture로 완료 처리하지 않았다.

추가 raw-test 전면 보강은 `checkpoint-r010.json`, loop reroute 수정 포함 현재 경계는 `checkpoint-r011.json`으로 분리했다. r011 최종 패키지별
격리 재실행은 154/154 타깃과 실제 JUnit 1,288/1,288 testcase를 통과했다.
이 결과는 package fixture/algorithm oracle을 닫지만 P7 런타임 증거를
확대하지 않는다. 상세 수치는
`docs/planning/evidence/planning-test-audit-r003.json`에 있다. 재현 명령은
`AW149_PACKAGE_TEST_TIMEOUT_S=600 scripts/workspace/test_planning.sh`다.
