# AjouNice AW149 통합 담당자 인계서

검토 기준: `planning/autoware-universe-to-ros1`의 AW149 통합 작업 트리 (2026-09-08).
저장소 루트 기준 상대 경로를 사용한다. 이 문서는 구현과 clean install 검증을 반영한 현재 인계 기준이다.

## 1. 현재 무엇이 되고, 무엇을 해야 하는가

포팅본은 **planning_ws 경계 149개 + 지원 5개와 base_ws 24개를 분리된 install 공간에 빌드·설치했다.** 공식 base 설치 경로는 `base_ws/install_aw149`다. 실제 v61 map을 읽는 full planning graph 24개 노드가 외부 MORAI 입력 없이 25초 동안 process death/FATAL 없이 기동했고 binary map을 수신했다. requester는 정책 8개·모의 RouteSelector ROS 통신 11개 검사를 통과했다. base 설치 뒤 source의 launch/config/RViz 자산이 install에 모두 존재하는지도 빌드 스크립트가 전수 검사한다. 2026-09-08 재감사에서는 삭제된 구형 planning 패키지를 참조하던 `kcity_interactive_trajectory_rviz.launch`와 낡은 안내 문서를 제거했다. 그 뒤 planning 154/154와 base 24/24를 각각 clean build/install하고, 현재 지원하는 설치 launch 19/19를 파싱했다. 최신 상세 영수증은 [planning-clean-integration-r002.json](evidence/planning-clean-integration-r002.json)이다.

설치된 `morai_planning_stack.launch` 자체에 정확한 ROS1 타입의 합성 localization, perception, operation-mode, TF를 공급하는 출력 probe도 통과했다. direct Mission service 기준선은 1/1, production `competition_route_requester` 경로는 서로 다른 private master에서 2/2 통과했다. requester 실행 모두 실제 Mission service로 Route A 47 lanelets를 설정하고 연속된 trajectory/validator 증거를 확인해 `VERIFIED(route_a)`로 끝났다. 최신 trajectory 111점, validation flag 20개 true, process death 0이었다. full graph에서 검증 완료까지 약 24.5~25.5초가 필요해 trajectory 관찰 창은 5초에서 30초로 조정했지만 연속 3개 fresh validated output 조건은 낮추지 않았다. 실행 명령과 세 raw result SHA는 [integrated-output-probe-r002.json](evidence/integrated-output-probe-r002.json)에 있다. 이 결과는 base launch 배선과 planning 출력 경로를 증명하지만 실제 MORAI 데이터 의미나 폐루프 제어를 증명하지 않는다.

추가 r011 검사는 Route A 검증 후 합성 odometry를 2 m/s로 바꾸어 production requester→Mission 안전검사→full BPP/BVP/MVP→validator의 A→B와 A→fallback을 현재 artifact에서 각각 2/2 통과했다. Route B는 57 lanelets/161점 trajectory, fallback은 28 lanelets/151점 trajectory였고 모두 validator 20/20 true, process death 0이었다. 순환 Route B 후반의 재방문 lanelet이 현재 공통구간을 가리던 Mission 검색 순서를 target-first로 수정했고 exact primitive·안전거리 검사는 유지했다. 종전 fallback cutoff `200360 s=8 m`는 공통 길이 26.751 m로 30 m 안전기준을 만족할 수 없어 `201446 s=8 m`로 앞당기고 안전기준은 유지했다. 상세는 [integrated-output-probe-r003.json](evidence/integrated-output-probe-r003.json)에 있다. 합성 pose 전환이므로 실제 차가 시작점부터 trigger까지 연속 이동한 결과로 확대하지 않는다.

운영 case `route_a_to_b_with_fallback`에서도 정상 trigger의 B 우선 분기와 B 미채택 cutoff의 fallback 분기를 각각 통과했다. status 이력상 정상 분기는 fallback dispatch 0, cutoff 분기는 Route B dispatch 0이며 두 분기 모두 target route의 fresh trajectory/validator까지 도달했다. [production-route-arbitration-r004.json](evidence/production-route-arbitration-r004.json)은 실제 B RPC timeout 뒤 복구나 연속 주행 증거로 확대하지 않는다.

추가로 B trigger에서 비물리적 1000 m/s fault injection을 사용해 기존 Mission 안전검사에 의한 명시적 B 거절을 만든 뒤 cutoff fallback 복구를 2/2 통과했다. 거절 직후 Route A가 active로 유지되고 uncertainty가 false인 것을 확인한 뒤 fallback route와 151점 trajectory, validator 20/20까지 검증했다. 제품 안전 파라미터는 수정하지 않았다. [production-route-rejection-recovery-r005.json](evidence/production-route-rejection-recovery-r005.json)은 RPC timeout/ambiguous 자동복구 증거가 아니다. 그 경우는 의도적으로 operator reconciliation 대상이다.

**인지 입력만 연결하면 정상 trajectory가 보장된다는 뜻은 아니다.** 현재 AjouNice localization/perception/control의 실데이터를 함께 연결한 end-to-end 검사는 완료되지 않았다. 새 graph의 기동 검사는 route/ego/TF/objects/acceleration/mode를 공급하지 않았고, 필요한 입력을 기다리는 상태였다. 지금 계속 주행 graph를 켜두었다는 뜻도 아니다.

동일한 frozen planning source와 v61 fixture를 쓴 최종 P7.1 독립 시험은 Mission→BPP→BVP→MVP→validator graph의 11/11 matrix를 통과했다. Route A/B, normal/experimental BVP, 안전 LC, 점유 차선 geometry, 교차로, red→green, 정적 장애물, moving ego, 입력 복구, 정지 후 reroute가 포함된다. 세부 수치와 해시 계보는 [P7-STANDALONE-ACCEPTANCE.md](P7-STANDALONE-ACCEPTANCE.md)에 있다. 이는 AjouNice 실제 입력, 이동 중 A→B 성공, stale 중 자동 안전 정지, controller 폐루프 승인은 아니다. `149`는 패키지 경계 수이며 동시에 실행할 독립 노드 149개라는 뜻도 아니다.

후속 raw-test 전면 보강은 checkpoint r010에 봉인했고, loop reroute 수정을 반영한 현재 source 경계는 r011이다. r011 현재 소스에서도 planning 154개 `run_tests`를 fresh master/ROS_HOME의 패키지별 격리로 전면 재실행해 154/154 타깃, JUnit 253개·1,288 testcase, errors/failures/skips/malformed 0을 확인했다. 재현 스크립트는 `scripts/workspace/test_planning.sh`, 상세는 [planning-test-audit-r003.json](evidence/planning-test-audit-r003.json)이다. 테스트가 없는 67개 패키지는 빌드된 target 완료만 증명한다.

실제 lane-driving 경로:

```text
competition_route_requester → RouteSelector / MissionPlanner → LaneletRoute
  → full Behavior Path → full Behavior Velocity → Path Smoother → Path Optimizer
  → Motion Velocity → Scenario Selector → Velocity Smoother → Planning Validator
  → /planning/trajectory → AjouNice follower → /ctrl_cmd → MORAI/차량
```

Scenario Selector에는 parking 분기도 연결돼 있다. lane-driving scenario는 원래 selector가 만들도록 하고, 초기 출력이 없다는 이유로 scenario를 상수로 위조하지 않는다.

## 2. 우선순위와 담당

| ID | 담당 | 작업 | 왜 필요한가 |
|---|---|---|---|
| B0 | 통합/저장소 | **완료:** provider 2개 CRLF blob 및 `.gitattributes` 복구 | 새 clone의 provider 해시 보존 |
| B1 | 통합/빌드 | **완료:** submodule/고정 Livox SDK 준비, planning→base 24/24 clean install | 낡은 devel 혼입 차단 |
| P1 | 객체 예측 | **완료:** Duration 필드와 소비자/테스트 변환 | Python 메시지 직렬화 통과 |
| P2 | 신호 인식 | **완료:** Time 변환, helper/weights 설치, v61 설정, route state 관리 | clean install 기동·계약 검사 통과 |
| L1 | Localization | **정적 계약 완료:** MORAI acceleration 전달 및 odometry와 동일 stamp ROS transport. 실 MORAI 축·단위·TF/clock 동적 검증은 남음 | BPP/BVP/MVP/validator 필수 입력 |
| P3 | 인지/LiDAR | 실제 ground-free cloud와 occupancy 공급 | 대상 plugin의 판단 입력, occupancy는 BPP cyclic readiness에도 사용 |
| S1 | System/차량 인터페이스 | **토픽 정합 완료:** `/system` 단일 원천을 `/api`로 relay. 실제 mode/engage 상태 공급은 남음 | 초기 경로/재경로/주행 상태 판단 |
| C1 | 제어 | **코드 연결 완료:** follower 입력 `/planning/trajectory`; 실데이터 source age·validator 승인은 남음 | output-only 후 폐루프 순서 유지 |
| C2 | 제어/차량 | 제어 예측 trajectory·Control·SteeringReport 공급 | MVP 경계 이탈 방지 모듈의 추가 입력 |
| M1 | Map/bringup | **구성 완료:** 통합 launch에서 binary map 단일 발행, v61/projector 고정 | 실운영 중 publisher 유일성 재확인 |
| V1 | 차량 설정 | **구성 완료:** mesh 측정 원천 기반 flat YAML과 provenance 검사. 실차 track/tire/steer 재승인은 남음 | 시험차 YAML 혼입 차단 |
| T1 | 도구/제어 | **완료:** CSV/history native Duration 변환 | base 24/24 빌드 및 단위검사 통과 |
| R1 | 통합/bringup | **주 경로 완료:** 통합 entrypoint와 runtime source를 install_aw149로 통일 | 과거 실험 launch는 운영 entrypoint가 아님 |
| A1 | 통합/검증 | **standalone P7.1 11/11 완료:** 동일 소스·v61 fixture graph 승인. 실제 AjouNice route→trajectory→plugin→폐루프 승인은 남음 | fixture PASS를 production E2E PASS로 승격하지 않음 |

## 3. B0 — 새 컴퓨터 빌드 전 반드시 처리할 푸시본 문제

로컬 작업 파일뿐 아니라 Git blob을 원본 manifest SHA256와 대조해 아래 2개의 CRLF 변환 문제를 찾았고 원본 바이트로 복구했다.

- `planning_ws/external/osqp/src/docs/make.bat`
- `planning_ws/external/acados/src/external/osqp/docs/make.bat`

Git의 `text=auto` 적용으로 CRLF가 변환됐다. 로컬 원본 작업 파일의 해시는 일치해 기존 빌드가 성공했지만, clone으로 받은 내용은 다르다. `planning_ws/tools/prepare_osqp.py`와 `prepare_acados.py`는 provider 전체 파일의 해시를 검사하므로 문서용 BAT 파일이어도 검사를 통과하지 못한다.

두 provider의 가장 가까운 `docs/.gitattributes`에 `make.bat -text`를 추가하고 파일 blob을 원본 SHA256에 맞췄다. manifest를 완화하거나 파일을 검사에서 제외하지 않았다. 최종 커밋 뒤 Git blob 기준 전수 대조를 다시 실행해야 한다.

## 4. B1 — 메시지 소유권·의존성·빌드 순서

### 메시지는 planning_ws가 소유한다

기존 base 중복 메시지 14개를 이미 제거했다. 새 `autoware_*_msgs`, `tier4_*_msgs`, `builtin_interfaces`, `unique_identifier_msgs`는 planning install에서 찾도록 한다. 이름이 같아도 이전 base 생성물과 MD5가 다를 수 있으므로 C++ 노드와 별도 Python/Conda 프로세스까지 모두 같은 메시지를 로드해야 한다.

헤더 `std_msgs/Header.stamp`는 native ROS1 Time이다. 반면 `PredictedPath.time_step`, `TrajectoryPoint.time_from_start`, `TrafficLightGroupArray.stamp`, mode/control/steering payload stamp는 동결된 `builtin_interfaces` 타입이다. 모든 stamp를 일괄 치환하지 않는다.

### 외부 코드와 런타임

- `base_ws/src/ekf-multi-object-tracking`은 초기화했고 ROS1 install 규칙을 보완한 revision `94819735`를 사용한다. clean install의 binary/config/launch 및 14개 C++ 검사를 확인했다.
- `robot_localization`과 `jsk_recognition_msgs`는 Noetic apt 패키지로 설치했다. 친구 컴퓨터에서는 rosdep/apt 단계가 필요하다.
- tracking/prediction 평가 테스트는 Ubuntu의 일치하는 NumPy/SciPy/Pytest를 사용하도록 user-site를 격리한 catkin wrapper로 29/29를 통과했다. `python3-nose`, `python3-pytest`를 rosdep/apt로 준비한다.
- Livox SDK는 `third_party2/livox_sdk`의 Gitlink revision `9306596a`를 `build_external`에 먼저 빌드한다. 드라이버 CMake가 최신 upstream HEAD를 임의 clone하지 않도록 `build_all.sh`가 이 정적 라이브러리를 강제한다.
- traffic light runtime은 package의 `requirements-runtime.txt`에 검증 버전을 기록했다. PyTorch wheel은 대상 CUDA에 맞게 먼저 선택해야 한다.
- FastBEV/PointPainting checkpoint는 Git에 포함되지 않는다. 통합 launch의 detector 기본값은 비활성이고, 별도 배포한 모델의 SHA256 확인 후 명시적으로 켠다.
- 전체 catkin 검색 결과는 planning 154(경계 149 + 지원 5), base 24이며 중복 package name은 0이다.

### 빌드 경로

`setup_ws.sh`는 submodule을 재귀 초기화한 뒤 `build_all.sh --clean`을 호출한다. 빌드 순서는 Noetic → planning provider/154 package install → pinned Livox SDK → fast_lio 이외 base 23개 → fast_lio 메시지 선생성/본체 install이다. pin된 FAST_LIO는 실행 파일이 생성 메시지 헤더에 의존한다는 선언이 없어 clean 병렬 빌드에서 `Pose6D.h`보다 먼저 컴파일될 수 있다. 하네스가 upstream submodule을 수정하지 않고 이 순서를 명시한다. `--clean`은 생성된 `base_ws/{build,devel,install}_aw149`만 지우고 다시 만들어 과거 install 잔재가 launch 탐색을 오염시키지 않게 한다. planning이 base를 underlay로 사용하지 않으므로 순환 의존이 없다. 개발 중 증분 빌드는 인자 없는 `build_all.sh`를 쓸 수 있으며, 자산 감사에서 legacy flat-install 중복을 발견하면 clean 재빌드를 요구한다.

아래 명령을 실제로 실행해 base 24/24 install 성공을 확인했다.

```bash
# repository root에서, 기존 ROS/Conda overlay가 섞이지 않은 새 셸
bash scripts/workspace/setup_ws.sh
source scripts/workspace/source_aw149.sh
```

clean 설치본의 통합 entrypoint 생존과 v61 binary map transport는 별도 ROS master를 쓰는 아래 foreground smoke로 재현한다. 스크립트는 controller/requester/detector를 안전 기본값으로 끄고 24개 필수 노드를 정확히 대조하며, 종료 시 자신이 띄운 master와 launch를 정리한다. 25초 실행 결과는 [clean-install-runtime-smoke-r001.json](evidence/clean-install-runtime-smoke-r001.json)에 기록했다.

```bash
bash scripts/runtime/smoke_planning_stack.sh
```

base 24개 전체 catkin test target은 `bash scripts/workspace/test_base.sh`로 실행한다. clean 결과물 기준 24/24 target과 catkin 집계 114 tests가 오류·실패·skip 없이 통과했으며 [base-clean-test-r001.json](evidence/base-clean-test-r001.json)에 기록했다.

운영 환경은 새 planning install 후 `base_ws/install_aw149/setup.bash`를 source하도록 확정한다. 기존 `base_ws/devel`, 다른 Autoware2ros1 install, addon-only install을 뒤섞지 않는다. `rospack find`와 Python module `__file__`, 실제 연결 MD5로 각 프로세스의 경로를 확인한다.

## 5. P1 — 객체 검출·추적·예측

입력 흐름은 `DetectedObjects → tracker의 TrackedObjects → PredictedObjects`다. planning 최종 인지 입력은 `/perception/object_recognition/objects`의 `autoware_perception_msgs/PredictedObjects`이며, boxes/MarkerArray나 DetectedObjects를 직접 remap하지 않는다.

| 실제 수정 파일 | 현재 문제 | 변경 |
|---|---|---|
| `base_ws/src/perception_object_pipeline/scripts/object_prediction_node.py:51` | `path.time_step = rospy.Duration(horizons[0])` | builtin Duration 생성/대입 |
| 같은 scripts의 `map_based_prediction_node.py:55,95` | 동일한 native Duration 대입 2곳 | 동일 helper 적용 |
| 같은 scripts의 `object_pipeline_visualization_node.py:265` | `time_step.to_sec()` | `sec + nanosec * 1e-9` 사용 |
| `base_ws/src/perception_object_pipeline/test/test_object_prediction.py:45` | native `.to_sec()` 가정 | 새 payload 타입으로 기대값 및 serialize 검사 |
| 같은 test의 `test_standard_pipeline.py:118` | 동일 | 실제 ROS transport 포함해 확인 |
| `package.xml`, helper를 공유할 경우 CMake/setup.py | builtin 타입 직접 사용 의존성 | `builtin_interfaces` 직접 의존성 및 helper 설치 명시 |

Python 변환 형태(구현 지침):

```python
from builtin_interfaces.msg import Duration as BuiltinDuration
from builtin_interfaces.msg import Time as BuiltinTime

d = rospy.Duration.from_sec(seconds)
path.time_step = BuiltinDuration(sec=d.secs, nanosec=d.nsecs)
group_array.stamp = BuiltinTime(sec=source_stamp.secs, nanosec=source_stamp.nsecs)
seconds = path.time_step.sec + path.time_step.nanosec * 1e-9
```

native `header.stamp`는 그대로 유지한다. 음수 duration도 nsec 범위가 정상화되도록 변환한다. 정상 객체 1개·객체 없는 관측·다중 객체를 각각 직렬화해서 받아야 한다. 빈 객체 메시지는 센서가 실제 관측한 empty scene일 때만 의미가 있다.

현재 `object_prediction.launch`의 map 기본값은 v61이며, 상위 `object_pipeline.launch`도 `map_file`과 `projector_file`을 하위 prediction launch로 전달한다. 운영에서는 기본값에 의존하지 말고 통합 entrypoint에서 승인한 v61 경로를 명시적으로 전달한다. CTRV 기본 운용에는 map 파일이 직접 쓰이지 않는다.

기본 prediction horizon은 3.0s, step은 0.1s다. 필요한 lane change·장애물 예측 시간보다 짧은지 검토한다. 무조건 15s로 늘리기보다는 plugin 요구 시간, 예측 신뢰도, stale 제한, 처리 지연을 함께 결정한다. pose/shape/heading/velocity는 map 및 객체 좌표계 정의에 맞고 velocity는 m/s여야 한다. map_based 노드는 `header.frame_id != map` 입력을 버린다.

실제 `sensor_cam_raw_obstacle1.bag` 15초 구간을 oracle/evaluation adapter로 map-frame `TrackedObjects`로 변환해 v61 map-based predictor에 입력했다. tracking 699 frame/5,396 object와 prediction 511 frame/3,733 object를 기록했고, prediction object 전부에 현재 pose를 포함한 31-point/3.0초 horizon path가 있었으며 비정상 위치는 0이었다. 같은 bag의 기본 CTRV 경로도 3,124 prediction object 전부에 같은 길이의 path를 만들었다. map-based 처리 로그는 입력에 따라 약 1.9–37 ms였고 50 Hz 원본의 모든 frame을 보존하지는 않았다. 이는 실제 객체 메시지·좌표계·예측 연결 증거이며 oracle GT 입력이므로 production detector/tracker 정확도 증거는 아니다. `competition_route_full_highspeed.bag`의 `/Object_topic` 7,551 frame은 전부 empty list라 그 bag만으로 장애물 기능을 승인할 수 없다.

## 6. P2 — 신호 인식은 이름 외에도 수정할 부분이 있다

정식 출력은 `/perception/traffic_light_recognition/traffic_signals`, 타입은 `autoware_perception_msgs/TrafficLightGroupArray`로 이미 맞는다.

1. **완료:** `traffic_light_node.py`는 native image stamp를 builtin Time payload로 변환하고 실제 영상 관측 시각을 보존한다. 0 timestamp fallback은 별도 동적 확인 대상이다.
2. **완료:** 통합 launch가 v61 OSM/projector와 `/planning/mission_planning/route`, `/planning/route_state`를 명시적으로 전달한다.
3. **완료:** latched route는 30초 receipt timeout으로 폐기하지 않고 `RouteState.UNSET` 또는 route 변경으로 무효화하며, odometry freshness는 계속 검사한다.
4. 같은 파일 `get_group_id`: XY 최단거리로 현재 route index를 고른다. 루프·인접 차로에서 방향과 진행 이력이 맞는지 검증하고 필요하면 heading/진행 index 연속성을 사용한다. 신호 table의 regulatory group ID와 v61 실지도 연결을 대조한다. 소스 주석의 특정 map ID 오류 주장은 이번에 지도 결함으로 재확정한 것이 아니므로 맵을 임의 수정하지 않는다.
5. 거리 fallback은 다른 진입로의 신호 선택 가능성이 있으므로 실제 fallback 발동 이유를 진단에 남긴다. 매칭 불가/unknown/검출 없음의 상태가 BVP에 어떻게 전달되는지 시험한다. 임의 green 또는 group_id 0으로 기능 합격을 만들지 않는다.
6. **완료:** helper scripts, group-ID table, weights 디렉터리, runtime requirements를 install share/bin에 배치하고 clean-install 기동에서 확인했다.
7. **완료:** 직접 사용하는 MORAI/nav/planning/builtin/Lanelet/PyYAML 의존성을 `package.xml`과 CMake에 선언했다. 실제 모델 실행 환경은 대상 CUDA에 맞는 PyTorch와 별도 checkpoint 검증이 여전히 필요하다.

합격 기준: clean install에서 helper import/weights 로딩 성공, route가 30초 이상 유지돼도 올바른 신호 매칭, 실제 red/green/arrow/unknown·영상 지연 상황에서 올바른 stop line 정지/해제.

## 7. L1 — Localization, TF, clock

| 항목 | 현재 | 필요한 조치 |
|---|---|---|
| Odometry | `localization_manager/scripts/morai_gt_kinematic_state_publisher.py:173–183`이 `/localization/kinematic_state` 발행 | topic/type는 유지. map pose, base_link twist, m/s·rad/s, 측정 stamp 확인 |
| GT acceleration | `morai_gt_kinematic_state_publisher.py`가 `/Ego_topic.acceleration`을 `/localization/acceleration`으로 직접 전달 | 동일 stamp/base_link frame의 AccelWithCovarianceStamped. 실 MORAI에서 축·단위·중력 포함 여부 확인 |
| CTRA acceleration | `localization_manager/launch/gnss_imu_gt_ctra.launch:145`, `configs/localization/robot_localization_gnss_imu_gt_ctra_global.yaml:5`에 설정 존재 | 이 모드의 실제 실행 시 `/localization/acceleration` 발행과 covariance 확인 |
| 일반 GNSS/IMU | `robot_localization_gnss_imu_{local,global}.yaml`의 publish_acceleration=false | 선택 모드에 맞게 estimator 출력과 remap을 추가 |
| TF | GT는 `map → morai_map → base_link` | chain 유지 가능. 센서 TF 포함, 같은 edge에 복수 publisher 금지 |
| 시간 | 여러 bringup launch가 `/use_sim_time`을 설정 | 하나의 운영 clock 정책. true이면 실제 `/clock` 필요, 여러 launch가 서로 덮지 않도록 통일 |

GT 노드는 `EgoVehicleStatus.velocity`를 base_link twist로 복사한다. 해당 MORAI 설정에서 실제 body frame 속도인지 회전 주행으로 확인해야 한다. `sensor_interface/config/lidar_preprocess.yaml`도 `morai_velocity_frame=base_link`를 가정한다. 이름만 맞춰도 두 가정이 틀리면 deskew와 planning 양쪽이 틀어진다.

GT covariance는 unavailable로 설명돼 있다. 이를 실제 추정 covariance 검증까지 완료했다고 취급하지 않는다. acceleration은 `geometry_msgs/AccelWithCovarianceStamped`, `header.frame_id=base_link`로 MORAI Ego 측정값을 보존해 공급한다. 차체 roll/pitch·중력·yaw 회전 때의 축·부호·단위는 실제 입력으로 검증한다.

## 8. P3 — pointcloud와 occupancy 공급

현재 `sensor_interface/launch/lidar_preprocess.launch`는 `/sensor/lidar1 → /sensor/lidar1/deskewed`를 만든다. 지면제거/occupancy production 공급은 검토한 코드에서 확인되지 않았다.

- ground-free: `/perception/obstacle_segmentation/pointcloud`, `sensor_msgs/PointCloud2`. 실제 ground segmentation을 적용하고 stamp 시점 TF, finite xyz, frame, point field를 확인한다. deskew만 된 cloud나 object marker를 이름만 바꿔 넣지 않는다.
- occupancy: `/perception/occupancy_grid_map/map`, `nav_msgs/OccupancyGrid`. resolution·width/height·origin pose·frame·data 길이와 unknown(-1)/free/occupied 의미를 정의한다. 살아 있는 관측에서 갱신되는지, 지도 밖 및 관측 불가 영역을 free로 잘못 처리하지 않는지 확인한다.
- BPP full node `behavior_path_planner_node.cpp:281`의 `isDataReady`는 objects/odom/occupancy cyclic 수신을 검사한다. occupancy가 없으면 인지 객체를 공급해도 경로 출력이 막힐 수 있다.
- BVP/MVP/parking/validator도 cloud·grid를 각 기능에서 사용한다. MVP는 활성 module의 required subscription을 기준으로 입력 준비를 검사하므로 실제 선택한 plugin 구성을 기준으로 확인한다.

합격 기준: 비어 있는 실제 장면과 실제 장애물이 있는 장면, 센서 끊김·TF 누락·stale·unknown 상황을 구분해 동작한다. 더미 empty/free 입력을 production launch에 넣지 않는다.

2026-09-08 clean 통합 graph의 publisher/type 감사에서는 기본 10개 중 localization 2개, objects, traffic signal, map 2개가 광고됐고 occupancy·ground-free cloud·실제 system mode는 공급자가 없었다. `/api` mode도 source가 없어서 lazy relay 출력이 생기지 않았다. C2 3개까지 포함하면 총 13개 중 7개가 `NOPUB`였다. 이는 [integration-topic-contract-r001.json](evidence/integration-topic-contract-r001.json)에 고정했으며, `PASS` 6개도 `--types-only` 판정이라 실제 message flow 합격으로 세지 않는다.

## 9. S1 및 C2 — operation mode와 제어 피드백: 기존 목록에서 빠진 부분

System은 `/system/operation_mode/state`의 `autoware_adapi_v1_msgs/OperationModeState`를 실제 모드/제어권과 연결해야 한다. 현재 production publisher는 확인되지 않았다. `mode`, `is_autoware_control_enabled`, `is_in_transition`, stamp를 사실대로 갱신한다. requester는 재경로 시 autonomous/control-enabled, 전환 중 아님, 최신 reroute availability의 hold 조건 등을 확인한다.

**현재 full MVP launch에는 BoundaryDeparturePrevention module이 포함된다.** 이 module은 다른 경로 `/api/operation_mode/state`를 구독한다. 통합 launch의 `operation_mode_state_to_api` relay가 `/system/operation_mode/state`를 `/api/operation_mode/state`로 그대로 전달하며 transport 검사를 통과했다. 따라서 실제 상태 발행기는 `/system` 한 곳에만 둔다. 외부 스택이 이미 두 topic을 모두 발행한다면 `relay_operation_mode_to_api:=false`로 중복 발행을 막는다.

추가 피드백 3개:

| planner가 요구하는 topic/type | 현재 base 제공 | 변경 설계 |
|---|---|---|
| `/control/trajectory_follower/lateral/predicted_trajectory` — `autoware_planning_msgs/Trajectory` | 없음 | 실제 controller 모델이 예측한 ego rollout을 map pose·시간·속도로 발행. planner 참조 trajectory를 그대로 복사하는 것은 다른 의미 |
| `/control/command/control_cmd` — `autoware_control_msgs/Control` | `/control/command/steering` Float64와 `/ctrl_cmd` morai_msgs/CtrlCmd | controller의 실제 결정에서 lateral/longitudinal command를 정식 물리 단위로 발행하는 adapter/출력 확장 |
| `/vehicle/status/steering_status` — `autoware_vehicle_msgs/SteeringReport` | 정식 topic 공급 미확인; EgoVehicleStatus에 wheel_angle 필드는 있음 | wheel_angle 단위·부호·tire/steering-wheel 의미를 확인 후 실제 조향 피드백으로 변환 |

`Control.lateral.steering_tire_angle`는 rad, longitudinal velocity는 m/s, acceleration은 m/s²다. `morai_msgs/CtrlCmd.accel/brake`의 0..1 pedal 값을 acceleration에 그대로 넣으면 안 된다. `SteeringReport`는 명령 복사값을 실제 측정값처럼 발행하지 않는다. native ROS1 header와 달리 여기 payload stamp들은 builtin Time이다.

근거: `planning_ws/src/autoware_motion_velocity_boundary_departure_prevention_module/src/boundary_departure_prevention_module.cpp:308`의 구독, `:424`의 `is_data_invalid`, `:457`의 timeout. 입력이 없으면 `plan()`이 오류 설명과 빈 planning result를 반환해 해당 module의 velocity 개입이 없을 수 있다. **다른 모듈이 trajectory를 계속 내보내도 이 기능까지 실행됐다고 판정하면 안 된다.** timeout은 odom/예측 trajectory/control/steering에 1s 기준을 적용한다.

이 피드백을 아직 제공할 수 없으면 기능 제한을 명시하고 해당 plugin의 검증을 미완료로 둔다. 통과 수를 맞추기 위한 더미 피드백이나 무기록 plugin 해제는 통합 완료가 아니다. controller가 첫 trajectory를 받기 전/받은 후의 startup 순환 의존도 실제 graph에서 시험한다.

## 10. C1 및 T1 — follower와 기존 trajectory 도구

### Follower

`base_ws/src/control/trajectory_follower/config/trajectory_follower_autoware.yaml:2`의 `topics/trajectory`를 최종 `/planning/trajectory`로 바꾼다. 선택 launch는 `pp_pi_autoware.launch`. 다른 `pp_pi.launch`나 CSV trajectory source와 중복 실행하지 않는다.

현재 흐름: `purepursuit_autoware.cpp:166`에서 Float64 steering·validity·debug를 발행하고 `longi_pi_autoware.cpp:256`이 MORAI type-1 CtrlCmd를 만든다. 이름에 autoware가 있어도 위 C2의 정식 control/예측 trajectory/steering report를 모두 제공하는 구현은 아니다.

`purepursuit_autoware`와 `longi_pi_autoware`는 기존 수신 시각 timeout에 더해 trajectory/state의 원본 `header.stamp` age를 검사한다. 0·0.5초 초과·0.1초보다 미래인 stamp를 거부한다. 실제 ROS transport 시험에서 fresh 입력으로 전진 명령, 오래된 trajectory를 30 Hz로 계속 재발행하는 동안에도 `accel=0`, `brake=1`, `steering=0`, 이후 fresh 입력 복구를 확인했다. 이 controller-side 방어는 planner 자체의 cached output 지속을 고친다는 뜻이 아니다.

- 실제 주행 clock에서 source-age 임계값 0.5초와 future tolerance 0.1초가 적합한지 확인하고 sim-time pause/reset도 별도 시험한다.
- `/planning/validation_status`의 20개 `is_valid_*` 및 해당 trajectory와 시간상 대응. status topic은 `autoware_planning_validator/PlanningValidatorStatus`다.
- perception/localization 끊김 또는 오래된 trajectory 재발행 시 실제 정지 동작.
- planner 최종 출력이 있다는 이유로 무조건 valid로 보지 않는다. validator가 invalid trajectory를 어떻게 처리하는지도 실로그와 함께 검사한다.

`/ctrl_cmd` 발행과 simulator command 송신을 분리해 최초 output-only 검사를 한다. 실제 MORAI 연결 시 `sensor_interface`의 sender/bridge와 제어권 계약을 정하고 동시에 두 송신 경로를 켜지 않는다.

### 도구의 컴파일/직렬화 수정

| 파일 | 위치/내용 | 변경 |
|---|---|---|
| `base_ws/src/control/trajectory_generator/src/trajectory_publisher_node.cpp` | 251, `ros::Duration`을 time_from_start에 대입 | builtin Duration 필드에 sec/nanosec 변환 |
| `base_ws/src/control/trajectory_validator/src/trajectory_validator/history_publisher.py` | 57, 204, native Duration 대입 | P1과 같은 helper |
| 같은 디렉터리 `recorder.py` | 630, native Duration 대입 | 동일 |

C++ 예시: `ros::Duration d(seconds); point.time_from_start.sec = d.sec; point.time_from_start.nanosec = d.nsec;`. 필요 dependency/header를 추가하고 전체 base를 재컴파일한다. CSV/history 도구를 운영 launch에서 빼더라도 source를 전체 빌드하면 generator의 컴파일 문제는 남는다.

## 11. M1 및 V1 — 지도·차량 설정

binary 지도는 `ajounice_planning_integration/launch/map.launch`가 실행하는 full loader 한 곳이 `/map/vector_map`을 발행한다. `lanelet_map_visualizer/scripts/lanelet2_osm_marker_publisher.py:736`은 raw OSM을 LaneletMapBin에 넣으므로 함께 발행하면 안 된다. 타입/MD5가 같아도 payload가 다르다.

`bringup/launch/localization_morai_gt.launch`는 `start_lanelet_map=true`가 기본이며 visualizer에 `publish_vector_map=false`를 전달하지 않는다. 통합 launch에서 `start_lanelet_map=false`로 시작하고 필요하면 별도로 visualizer를 `publish_vector_map=false`로 실행하거나, 해당 arg를 상위 launch까지 전달하도록 수정한다. 다른 localization launch도 visualizer include를 확인한다.

v61 OSM SHA256: `c8ee07fb70334a13373f8cbd54fabe95980ac1e9589bbc05382337074f089dc2`.
projector: `kcity_map/lanelet_map/map_projector_info_v1.yaml`.
기존 map projector addon은 생성 상수 `MapProjectorInfo.LOCAL_CARTESIAN_UTM` / `WGS84`를 사용하도록 수정돼 있다. 이 포팅 메시지는 ROS1에서 문자열 상수에 따옴표가 남으므로 YAML literal과 wire constant를 혼동하지 않는다. map·projector hash를 바꾸면 requester의 frozen route 정의도 다시 검증해야 한다.

planner 기본값은 `configs/planning/vehicle_info.yaml`이다. nested `configs/vehicle/ioniq5.yaml`을 그대로 로드하지 않고, `base_ws/src/vehicle/meshes/ioniq5/ioniq5_mesh_measurements.json`의 target dimension과 controller의 steer 계약을 대조해 flat schema로 만들었다.

| 원천 값 | planner flat key | 처리 |
|---|---|---|
| wheelbase=3.000 / radius=0.3705 | wheel_base / wheel_radius | mesh target와 일치 |
| front=0.845 / rear=0.790 / height=1.600 | 해당 flat key | rear-axle frame 길이 합 4.635 m 검사 |
| rear_track=1.651 / tire_width=0.235 | wheel_tread / wheel_width | rear-axle `base_link` 기준 |
| body_width=1.890 | left_overhang=right_overhang=0.1195 | tread와 합해 body width, 미러 폭 2.134는 제외 |
| max_wheel_angle_deg=40 | max_steer_angle=0.6981317008 | controller 계약과 rad 변환 검사, 실차 tire angle 재확인 |

URDF와 sensor TF 모두 `base_link=rear axle center`로 문서화돼 있다. integration launch는 위 flat YAML을 기본 로드하며 10개 값의 실제 launch parameter 주입과 원천 일치 단위검사를 통과했다. follower는 기존 nested YAML을 쓰되 공통 물리값을 유지한다. competition 차량의 track, tire width, 40도 값이 tire angle인지 여부는 폐루프 승인 전 실물/공식 제원으로 재확인한다.

## 12. R1 — 실행 entrypoint·설치 경로 정리

- **완료:** `scripts/runtime/launch_sensing.sh`, `morai_status.sh`, `morai_reset.sh`는 모두 `scripts/workspace/source_aw149.sh`를 통해 공식 install overlay만 source하고 reset 대상도 install 경로를 사용한다.
- **완료:** follower, sensing, sensor TF, MORAI UDP, FastBEV, PointPainting, tracking/prediction evaluation의 저장소 내부 config/map 기본 경로를 `AJOUNICE_ROOT` 또는 상위 `repo_root` 인자로 통일했다. 설치 공간에서 standalone launch 6개를 기본 인자로 파싱하고 실제 저장소 config 경로 주입을 확인했다.
- 외부 OpenPCDet 위치는 저장소 내부 경로가 아니므로 `openpcdet_root`로 별도 지정한다. 과거 실험용 localization/시각화 launch의 `$HOME/AjouNice2026` 기본값은 운영 entrypoint와 구분하며, 사용할 경우 상위에서 명시 경로를 넘기거나 후속 정리한다.
- 설치 공간에서 실행하던 중 찾은 `tracking_prediction_evaluation`, `lanelet_map_visualizer`, `morai_ros_bridge`, PointPainting RViz, `ros_tutorial`의 launch/config/RViz 누락은 CMake install 규칙으로 수정했다. upstream FAST_LIO submodule은 install 규칙이 없으므로 submodule을 변조하지 않고 `build_all.sh`가 빌드된 binary와 launch/config/RViz 자산을 공식 install overlay에 복사한 뒤 전수 감사를 수행한다.
- 기존 검증 잔재 정리 완료: 존재하지 않는 local runner/package를 참조하던 launch 3개는 제거했다. `bringup/validation`의 v44 수치와 시각화 자료는 historical evidence로만 보존하며 현재 v61 실행 명령이나 승인으로 취급하지 않는다.
- top-level 통합 launch는 sensing/localization/perception/system/map/planning/requester/controller의 순서·활성화를 한 곳에서 제어하도록 구성한다. 기존 `ajounice_planning_integration/planning.launch`는 map+planning만 구성하고 requester는 기본 false, controller는 포함하지 않는다.
- Virtual traffic light/V2X, lateral offset, 외부 speed limit 같은 선택 입력은 모든 차로에서 무조건 필요한 주기 입력으로 취급하지 않는다. 사용 지도 regulatory element와 활성 module 요구에 따라 구현·시험한다. 실제 도로 신호 인식 topic과 virtual traffic light state는 별개다.
- 방향지시등은 사용자 운용 요구에서 제외됐다. 이는 C2의 실제 **조향각** 피드백과 다른 항목이므로 steering report까지 제외하면 안 된다.

## 13. Requester 설정과 승인

코드: `base_ws/src/planning_addons/competition_route_requester`.
설정: `config/competition_route_requester.param.yaml`, 경로: `config/kcity_v61_competition_route.yaml`.

- 초기에는 YAML case `route_a_only`, 공식 start pose/허용 lanelet을 사용한다. `route_b_only`는 200178 s≥8m 시작 조건이므로 공식 start에서 바로 쓰는 case가 아니다.
- main 서비스는 `/planning/set_waypoint_route` (`autoware_internal_planning_msgs/SetWaypointRoute`)와 `/planning/set_lanelet_route` (`autoware_planning_msgs/SetLaneletRoute`)다. 직접 LaneletRoute를 발행하거나 Mission 뒤에 route mux를 넣지 않는다.
- 정상 A→B는 200178 s≥8m부터, 200346 진입 전에 확인돼야 한다. fallback은 Mission의 30 m 안전거리를 보존하도록 201446 s≥8m에서 시작하는 28-lanelet suffix다. 각 case는 SPEC를 따른다.
- 서비스 timeout·수락 후 UUID/route/trajectory 확인 실패는 `RECONCILIATION_REQUIRED`. 이때 실제 route가 이미 바뀌었을 수 있어 “A 유지”로 가정하거나 맹목적 retry/clear/restart하지 않는다.
- 현재 trajectory 검증 관찰 창은 60초다. full graph cold-start가 과거 실행에서 30초를 넘긴 부하 편차를 수용하지만, post-route stamp/freshness·finite/shape/frame·validator 20개 flag·연속 3개 출력 조건은 유지한다. requester source-age는 canonical Planning Validator latency checker와 같은 1.0초이고, controller의 0.5초 안전정지는 그대로다. timeout 자체를 성능 승인으로 취급하지 않는다.
- 새 trajectory에는 route UUID가 없어서 응답·경로 확인 이후 timestamp와 validator 대응은 시간상 연관 증거다. 실제 수락된 경로를 따른 궤적인지 geometry/주행으로 검증해야 한다.

## 14. 순서대로 확인할 합격 기준

### 운영 토픽 대조표

아래는 base와 planning 사이의 계약이다. module 선택에 따라 필요 여부가 다른 피드백/선택 입력은 본문의 해당 항목을 따른다.

| topic | ROS1 type | 소유/작업 |
|---|---|---|
| `/localization/kinematic_state` | nav_msgs/Odometry | Localization, L1 |
| `/localization/acceleration` | geometry_msgs/AccelWithCovarianceStamped | Localization, L1 |
| `/perception/object_recognition/objects` | autoware_perception_msgs/PredictedObjects | Prediction, P1 |
| `/perception/traffic_light_recognition/traffic_signals` | autoware_perception_msgs/TrafficLightGroupArray | Traffic light, P2 |
| `/perception/occupancy_grid_map/map` | nav_msgs/OccupancyGrid | Occupancy, P3 |
| `/perception/obstacle_segmentation/pointcloud` | sensor_msgs/PointCloud2 | Ground removal, P3 |
| `/map/vector_map` | autoware_map_msgs/LaneletMapBin | full map loader 단일 발행, M1 |
| `/map/map_projector_info` | autoware_map_msgs/MapProjectorInfo | integration projector, M1 |
| `/system/operation_mode/state` | autoware_adapi_v1_msgs/OperationModeState | System, S1 |
| `/api/operation_mode/state` | autoware_adapi_v1_msgs/OperationModeState | integration relay가 `/system` 원천을 보존해 전달, S1/C2 |
| `/control/trajectory_follower/lateral/predicted_trajectory` | autoware_planning_msgs/Trajectory | Controller rollout, C2 |
| `/control/command/control_cmd` | autoware_control_msgs/Control | Controller 정식 명령 출력, C2 |
| `/vehicle/status/steering_status` | autoware_vehicle_msgs/SteeringReport | 차량 실제 조향 피드백, C2 |
| `/planning/mission_planning/route` | autoware_planning_msgs/LaneletRoute | RouteSelector 출력; requester/신호 인식 구독 |
| `/planning/route_state` | autoware_planning_msgs/RouteState | RouteSelector 출력 |
| `/planning/trajectory` | autoware_planning_msgs/Trajectory | 최종 planning 출력, follower 구독 C1 |
| `/planning/validation_status` | autoware_planning_validator/PlanningValidatorStatus | Planning validator 출력, C1/A1 |
| `/ctrl_cmd` | morai_msgs/CtrlCmd | AjouNice longitudinal/follower 출력, simulator interface 구독 |

### 단계별 검사

1. **Git checkout/빌드**: B0 수정 후 Git blob 해시 일치. 필요한 submodule 확보. planning 154 빌드 및 base 전체 빌드 성공. 새 환경에서 import·package 경로가 올바르고 같은 이름의 중복 메시지 없음.
2. **입력-only**: 실제 sensing/localization/perception/system을 실행. topic 이름·타입·MD5뿐 아니라 frame/단위/원본 timestamp/age/주기/TF 확인. 선택한 모드의 acceleration, occupancy, ground-free cloud 존재 확인. latched map/route와 cyclic 상태 메시지의 유효성 정책을 구분.
3. **Route A → 최종 trajectory**: controller 송신을 분리한 상태에서 requester A 실행. map, route state, scenario를 거쳐 각 stage의 출력이 연결되는지 확인. `/planning/trajectory`의 finite pose/speed/acceleration·차량 footprint·속도제한·정지점과 `/planning/validation_status` 전체 이력을 기록.
4. **전체 plugin 적용 여부**: 단순 library load와 실제 기능 개입을 구분. 특히 C2 피드백 없이 BoundaryDeparturePrevention을 완료로 세지 않는다. obstacle stop/slow/cruise, signal stop/release, 관련 intersection/crosswalk 등은 실제 해당 입력으로 확인.
5. **A/B/fallback 및 LC**: 합성 odometry가 2 m/s인 A→B와 A→fallback, 각 변경 뒤 새 trajectory는 r011에서 통과했다. 실제 연속 이동의 deadline, 잘못된/거절된 요청, 안전한 차선변경과 정말 막힌 차선변경 거부는 실제 입력으로 별도 검사한다. 합성 pose 전환을 폐루프 moving 승인으로 세지 않는다.
6. **폐루프/복구**: 위 통과 후 follower와 MORAI/차량 연결. 목표 속도 추종·정지·장애물·교차로·신호를 확인하고 input loss/TF loss/stale/clock reset/재경로 실패에서 기대한 정지·복구를 확인한다.

각 run에 route UUID, 지도/차량 config hash, 시작 준비 시간, 안정 구간 output Hz, 최대 gap, source-age/latency 분포, 전체 validator invalid 건수·연속 지속시간, 경로/장애물/차선변경 판정을 남긴다. 고정 정답 수치를 임의로 정하지 말고 현재 제어 timeout 및 실제 주행 조건으로 합격 임계값을 합의한다. 테스트의 마지막 validator 1개가 true였다는 사실만으로 전체 run을 PASS로 처리하지 않는다.

입력 연결 직후 `scripts/runtime/check_topics.sh`로 10개 planning 입력의 type과 실제 message flow를 선검사한다. `--types-only`는 publisher/type만, `--full`은 route/trajectory/validator 출력까지, `--controller`는 C2의 controller rollout·정식 control·steering report 3개를 포함한다. 옵션은 함께 사용할 수 있다. 이 스크립트의 PASS는 frame·단위·semantic 승인 대신 사용할 수 없다.

## 15. 친구에게 전달할 작업 완료 정의

“통합 완료”는 compile을 넘어 **실제 AjouNice 입력 → 유효한 route → 실제 필요한 plugin이 적용된 최종 trajectory → follower 폐루프**를 검사한 상태다. 현재는 소스·메시지·설치·통합 graph 기동, L1 가속도 전달, S1 mode topic relay, V1 기본 차량 설정까지 완료했다. 실제 데이터의 축·단위와 P3/S1/V1/C2/A1 동적 승인은 완료로 세지 않는다.

친구 컴퓨터에서는 먼저 `setup_ws.sh`로 같은 install을 재현한다. 그 다음 L1/P3/S1/V1 입력 계약을 맞추고 controller를 끈 output-only로 Route A/B와 trajectory 전체 이력을 승인한 후 C2/폐루프를 켠다. 기존 포팅 알고리즘 오류가 나오면 원본 코드/변환 규약/재현 입력을 함께 기록하고 최소 수정 후 해당 검사와 관련 회귀 검사를 다시 수행한다.
