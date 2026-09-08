# AW149 아주나이스 통합 인계서

대상 브랜치: `planning/autoware-universe-to-ros1`. `develop`과 다른 브랜치는 건드리지 않는다. 상세 topic·frame·기능 gate는 [BASE-WS-INTEGRATION-GUIDE.md](BASE-WS-INTEGRATION-GUIDE.md)를 함께 따른다.

## 지금 바로 가능한 것

- `planning_ws/src`에는 ROS1 포팅 경계 149개와 지원 5개가 있으며 provider 소스도 저장소 안에 있다.
- 중복 Autoware 메시지 14개는 base에서 제거했고 planning install을 단일 메시지 소유자로 쓴다.
- base의 새 메시지 소비 코드(Duration/Time), tracker 설치, traffic-light helper/weights, v61 경로, follower 입력을 수정했다.
- `scripts/workspace/setup_ws.sh`가 submodule 초기화, planning underlay, pin된 Livox SDK, base 24개를 순서대로 빌드한다.
- 공식 runtime overlay는 `base_ws/install_aw149` 하나다. `base_ws/devel` 또는 과거 addon install을 source하지 않는다.
- 저장소 내부 runtime 경로는 `source_aw149.sh`가 설정하는 `AJOUNICE_ROOT`를 기준으로 하므로 checkout이 `$HOME/AjouNice2026`가 아니어도 된다.
- `ajounice_planning_integration/morai_planning_stack.launch`가 localization, sensor TF, object tracking/prediction, traffic light, v61 binary map, Mission→BPP→BVP→MVP graph를 묶는다.
- detector checkpoint는 Git에 없으므로 detector, controller, route requester는 안전상 기본 `false`다.

현재 로컬 증거는 planning source audit PASS, base 24/24 install, install runtime 자산 전수 감사, object pipeline 21 tests, tracker 14 tests, tracking/prediction evaluation 29 tests, trajectory-validator 9 tests, traffic contract 2 tests, requester policy 8 tests와 ROS transport 10 tests다. MORAI GT localization은 실제 주행 bag의 이동 구간 250개 표본에서 odometry/acceleration 동일 stamp와 acceleration 값 보존을 확인했다. operation-mode `/system`→`/api` relay도 ROS transport 검사를 통과했다. v61 통합 graph는 24개 노드가 외부 입력 없이 25초 동안 process death/FATAL 없이 기동했고 map loader와 traffic model load까지 통과했다.

같은 checkpoint와 v61 fixture의 최종 P7.1 standalone graph는 11/11을 통과했다. Route A/B, BVP 양 모드, 안전/점유 차선 LC, 교차로, red→green, 정적 장애물 정지, moving ego, 2초 입력 복구, 안전 규약을 유지한 reroute를 Mission→validator 동일 graph에서 검증했다. 현재 checkout의 source·map·projector·route hash 일치도 다시 확인했다. 상세 범위와 제외 항목은 [P7-STANDALONE-ACCEPTANCE.md](P7-STANDALONE-ACCEPTANCE.md)와 [machine receipt](evidence/p7-standalone-reconciliation.json)에 있다.

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

FastBEV 또는 PointPainting을 켜려면 checkpoint 파일을 별도로 배포하고 SHA256을 release 문서와 대조한 뒤 `start_detector:=true`와 해당 경로 인자를 넘긴다. 현재 Git clone만으로 detector까지 켤 수 있다고 가정하지 않는다.

planning 입력 topic의 publisher/type만 먼저 보려면 `scripts/runtime/check_topics.sh --types-only`, 실제 메시지 흐름까지 보려면 `scripts/runtime/check_topics.sh`를 실행한다. Route 이후 출력까지는 `--full`을 추가한다. 하나라도 없거나 타입이 다르거나 기본 5초 안에 메시지가 없으면 nonzero로 종료한다. 이 검사는 축·단위·내용 의미까지 승인하지 않으므로 아래 담당별 동적 확인도 계속 필요하다.

## base_ws 담당자에게 받을 입력

| 담당 | topic/type | 현재 판단과 필요한 조치 |
|---|---|---|
| Localization | `/localization/kinematic_state` `nav_msgs/Odometry` | 이름은 맞음. map pose/base_link twist, stamp, m/s·rad/s 검증 |
| Localization | `/localization/acceleration` `geometry_msgs/AccelWithCovarianceStamped` | MORAI GT launch가 `/Ego_topic.acceleration`을 동일 stamp/base_link frame으로 직접 공급. 실 MORAI에서 축·단위·중력 포함 여부는 동적 확인 필요 |
| Perception | `/perception/object_recognition/objects` `autoware_perception_msgs/PredictedObjects` | 변환/직렬화는 수정됨. 실제 detector→tracker→predictor frame, age, rate 확인 |
| Perception | `/perception/obstacle_segmentation/pointcloud` `sensor_msgs/PointCloud2` | production ground-removal 출력 미확인. deskewed cloud 단순 remap 금지 |
| Perception | `/perception/occupancy_grid_map/map` `nav_msgs/OccupancyGrid` | production publisher 미확인. 빈 free grid로 합격시키지 않음 |
| System | `/system/operation_mode/state` `autoware_adapi_v1_msgs/OperationModeState` | 실제 제어권 상태와 연결. 통합 relay가 `/api`에도 동일 메시지를 전달. 상시 autonomous fixture 금지 |
| Vehicle/Control | predicted trajectory, `autoware_control_msgs/Control`, `autoware_vehicle_msgs/SteeringReport` | BoundaryDeparture MVP를 켤 때 실제 controller rollout과 조향 피드백 필요 |
| Vehicle | flat vehicle-info YAML | 실제 IONIQ5 치수와 base_link 기준점 승인 필요 |

신호 출력은 `/perception/traffic_light_recognition/traffic_signals`, 최종 planning 출력은 `/planning/trajectory`, follower 입력도 `/planning/trajectory`, MORAI 명령은 `/ctrl_cmd`다. Lanelet map은 integration binary loader만 `/map/vector_map`을 발행해야 하며 base visualizer의 raw OSM payload와 동시에 켜지 않는다.

## 실행 승인 순서

1. Controller와 requester를 끈 상태에서 actual sensing/localization/perception/system topic의 type, MD5, frame, timestamp, age, rate, TF를 기록한다.
2. requester를 켜 Route A를 요청하고 Mission→BPP→BVP→MVP 각 stage와 `/planning/trajectory`, `/planning/validation_status` 전체 이력을 기록한다.
3. 신호 red/green/unknown, obstacle stop/slow/cruise, 안전한 LC와 막힌 LC 거부, 실제 이동 중 A→B 및 fallback을 개별 검증한다.
4. 위 단계가 통과한 뒤에만 controller를 켜 `/ctrl_cmd` 폐루프를 검증한다.
5. input loss, TF loss, stale data, clock reset, route rejection에서 안전 정지와 복구를 확인한다.

각 run에는 route UUID, map/vehicle-config hash, 준비 시간, trajectory Hz와 최대 gap, source age/latency, validator invalid 전 구간을 남긴다. 마지막 한 sample이 valid이거나 노드가 살아 있다는 사실만으로 PASS 처리하지 않는다.

현재 인계 상태는 **아주나이스 저장소 내부에서 build/install 및 output-only 통합 시험을 시작할 수 있는 상태**다. 실제 MORAI 입력과 검토 차량값이 필요한 기능·폐루프 gate는 이 컴퓨터에서 임의 fixture로 완료 처리하지 않았다.
