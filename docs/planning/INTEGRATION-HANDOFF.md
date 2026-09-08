# AW149 통합 인계 — 개발자 확인 전 주행 연결 금지

## 이번 변경 범위

작업 브랜치: `planning/autoware-universe-to-ros1`, 시작 HEAD `db16770`.
`develop` 및 다른 브랜치는 변경하지 않았다. 커밋/푸시는 아직 하지 않았다.

- `planning_ws/src`: r003-source-002의 149개 경계 패키지 + 지원 5개.
- provider 소스/lock/renderer와 라이선스 포함, 새 경로에서 재빌드. 이전 install 복사 아님.
- `base_ws/src/msgs`의 중복 14개 제거. planning_ws 메시지만 사용.
- 빈 `base_ws/src/planning`, 기존 competition/planning validation launch 8개와 legacy fixture script 4개 정리.
- 위 제거물은 `/home/jihun/Autoware2ros1/porting/backups/ajounice-pre-aw149-db16770/`에 복구 가능하게 보관했다. Git 추적 파일은 시작 HEAD에도 남아 있다.
- requester 3개 노드 + map/projector/full planning launch addon 추가.
- 다른 스택 개발자의 localization/perception/control 구현은 수정하지 않았다.

**현재 base_ws의 기존 devel/install을 사용하면 안 된다.** 소스 메시지를 통합했지만 해당 메시지를 사용하는 base 코드의 아래 변경은 의도적으로 보류했다. 전체 base 재빌드/주행 승인이 아니다.

## 개발자별 전달 사항

| 담당 스택 | 현재 코드/형식 | 필요한 변경 및 확인 | 완료 기준 |
|---|---|---|---|
| 객체 예측 | `perception_object_pipeline/scripts/object_prediction_node.py:51`, `map_based_prediction_node.py:55,95`: `PredictedPath.time_step = rospy.Duration(...)` | `builtin_interfaces/Duration`의 `sec/nanosec`로 생성. `object_pipeline_visualization_node.py:265`의 `.to_sec()`도 필드 기반 변환 | 실제 PredictedObjects serialize/deserialize + planning 연결, prediction horizon/shape/좌표계 검증 |
| 신호 인식 | `perception/traffic_light_detection/scripts/traffic_light_node.py`: TrafficLightGroupArray payload stamp에 native rospy.Time | payload만 builtin Time으로 변환. `/perception/traffic_light_recognition/traffic_signals` 이름은 유지, v61 regulatory group ID 매칭 | 실제 신호 입력에 따른 BVP stop/release, stale/unknown 정책 확인 |
| 제어/trajectory 도구 | trajectory_generator C++의 `ros::Duration` 대입, trajectory_validator Python의 native Duration 대입 | `TrajectoryPoint.time_from_start.sec/nanosec`로 변환. 기존 CSV trajectory publisher와 planner 동시 발행 금지 | 새 메시지로 compile + 실제 serialization, 단일 trajectory 공급 |
| Follower/제어 | 입력 `/control/trajectory`; planner 출력 `/planning/trajectory` | 검토 후 follower 입력 설정 변경. 기존 receive-time timeout 외 원본 입력/trajectory freshness 확인 | input loss/stale/invalid 시 안전 정지. controller 연결 전 output-only 시험 |
| Localization | `/localization/kinematic_state` nav_msgs/Odometry 일치. CTRA acceleration 있음, MORAI GT launch에는 없음 | 사용 모드에서 실제 `/localization/acceleration` AccelWithCovarianceStamped 제공. pose map/twist base_link, covariance/단위/시간 검증 | 연속 fresh ego+accel, TF 단일 소유자 |
| 지면제거/LiDAR | deskewed cloud는 있으나 canonical ground-free cloud 없음 | `/perception/obstacle_segmentation/pointcloud` sensor_msgs/PointCloud2 제공. deskewed→ground-free 단순 remap 금지 | 실제 지면제거 결과와 frame/stamp 검증 |
| Occupancy | `/perception/occupancy_grid_map/map` production publisher 미발견 | 실제 nav_msgs/OccupancyGrid 제공. origin/resolution/frame/unknown 의미 합의 | 지도 밖/unknown을 안전하게 처리. 빈 free grid를 production 대체로 넣지 않음 |
| System/engage | operation mode 발행은 fixture에서만 확인 | `/system/operation_mode/state` autoware_adapi_v1_msgs/OperationModeState를 실제 제어권/모드에 연결 | 실제 engagement/transition 반영, 가짜 autonomous 상시 발행 금지 |
| Map/bringup | base visualizer가 LaneletMapBin.data에 raw OSM 넣음 | 기존 visualizer 실행 시 `publish_vector_map=false`; 새 binary loader만 `/map/vector_map` 소유 | 동일 타입/MD5 외 Boost binary payload 및 유일 publisher 확인 |
| 차량 설정 담당 | `configs/vehicle/ioniq5.yaml`은 예시 nested YAML, tread 없음 | 검토한 flat planner YAML: wheel_base/wheel_tread/front_overhang/rear_overhang/left_overhang/right_overhang/vehicle_height/wheel_radius/wheel_width/max_steer_angle 제공 | 실제 차량 치수와 기준점 합의. test fixture 치수 자동 채택 금지 |
| Map-based prediction | standalone launch 일부 v60 기본값 | v61 map/projector 명시, 현재 기본 horizon 약3s의 LC 충분성 검토 | 충분한 예측 범위로 주요 plugin 재시험 |

`std_msgs/Header.stamp`는 여전히 native ROS1 time이다. 모든 stamp를 일괄 변환하면 오히려 잘못된다. `builtin_interfaces` payload만 규약대로 변환하고 음수 duration 정규화도 지켜야 한다.

## 확인된 메시지 차이

| 타입 | 이전 base MD5 | planning MD5 |
|---|---|---|
| Trajectory | f564849920833d9ceb52033399660bfb | cfd4a446faa4bee5ac422db560c39c52 |
| PredictedObjects | 3919d7e4660f8152a1e9c2bc1bf49976 | 6102a226bd69cc37071944272eaba096 |
| TrafficLightGroupArray | 163f5b4abaacf767c5397c9a4ba18a8e | f4f413546ec069fbb4dae90ca5482b60 |

LaneletMapBin MD5는 같아도 raw XML과 binary payload는 호환되지 않는다.

MapProjectorInfo는 r003에 동결된 ROS2-origin string 상수를 ROS1 genmsg가 따옴표 포함 문자열로 생성한다. 새 projector publisher는 YAML 문자열을 그대로 대입하지 않고 `MapProjectorInfo.LOCAL_CARTESIAN_UTM`, `MapProjectorInfo.WGS84` 생성 상수를 사용한다. C++ loader도 이 상수와 비교한다. 이 통합 단계에서 메시지 정의/MD5를 임의로 변경하지 않았다.

## 빌드·실행 순서

1. `planning_ws/tools/build.sh --providers` — Noetic만 들어간 깨끗한 환경에서 154개 설치.
2. `planning_ws/tools/build_addons.sh` — 지금은 planning addon 2개만 별도 출력에 빌드. 다른 base 코드는 수정 대기.
3. 위 담당자 수정 완료 후 planning install을 underlay로 전체 base_ws를 **새 build/devel/install 공간**에 빌드. 오래된 base_ws/devel source 금지.
4. `planning_ws/tools/env.sh`는 새 planning + addon 설치만 source한다. 실제 base 전체 설치 검증 후 운영 환경 순서를 확정한다.
5. `ajounice_planning_integration/planning.launch`: reviewed `vehicle_param_path` 필수, requester 기본 비활성, controller 없음. 실제 필요한 입력이 없으면 기다리는 것이 정상이며 빈 메시지로 통과시키지 않는다.
6. output-only로 입력/TF/MD5/semantic 검사 → trajectory 품질·age·연속성 → requester A/B/fallback → controller 연결 후 폐루프.

## 아직 주행 승인을 막는 planning 자체 검증

- 기존 P7.1 PASS는 전 시간대 전 sample validator PASS가 아니다. Route A rosout에 invalid trajectory/latency warning이 있다.
- 실제 이동 중 A→B 완료는 미검증. 이전 시험의 standstill 성공을 moving 성공으로 대체하지 않는다.
- 새 route 수락 이후 trajectory의 시간상 연관과 안정 출력, 전체 sample latency/gap/invalid 기록이 필요하다.
- occupied-target 시험은 separated merge를 보였지만 진짜 collision-blocked LC rejection 전체 oracle은 남아 있다.
- requester의 서비스 mock/정책 검사는 Mission/BPP/BVP/MVP 및 MORAI 폐루프 검사의 대체가 아니다.

따라서 이번 단계의 목표는 **코드 물리 통합·새 경로 빌드·요청기 구현·인계 계약 명확화**다. 다른 스택을 수정하지 않기로 한 범위 때문에 실제 AjouNice input→trajectory 및 주행 통합 완료로 보고하지 않는다.
