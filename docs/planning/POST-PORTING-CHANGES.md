# 포팅 후 변경사항

이 문서는 ROS1 포팅 이후 실제로 변경한 항목과 변경 이유만 기록한다. Autoware 원본이 있는 `planning_ws`는 수정하지 않고 `base_ws`의 통합 launch와 보조 노드에서 적용했다.

## 1. Ground pointcloud 및 occupancy grid 비활성화

변경 내용:

- `use_ground_pointcloud=false`
- `use_occupancy_grid=false`
- `start_parking=false`
- ground pointcloud, occupancy grid, freespace parking 입력에 의존하는 모듈을 활성 모듈 목록에서 제외
- 사용하지 않는 입력을 가짜 빈 메시지로 발행하지 않도록 구성

변경 이유:

- 대회 구성에서 ground removal과 occupancy grid map을 사용하지 않기로 결정했다.
- 존재하지 않는 입력을 기다리거나 빈 입력으로 정상 동작처럼 보이게 하지 않고, 사용하지 않는 기능을 명시적으로 끄기 위해 변경했다.

적용 파일:

- `base_ws/src/planning_addons/ajounice_planning_integration/launch/planning.launch`
- `base_ws/src/planning_addons/ajounice_planning_integration/launch/morai_planning_stack.launch`

## 2. 자율주행 모드 상태 고정 publisher 추가

변경 내용:

- `/system/operation_mode/state`에 `autoware_adapi_v1_msgs/OperationModeState`를 10 Hz로 발행하는 노드 추가
- 발행값을 `mode=AUTONOMOUS`, `is_autoware_control_enabled=true`, `is_in_transition=false`로 고정
- `planning.launch`와 `morai_planning_stack.launch`에 `publish_fixed_autonomous_mode` 인자 추가
- 기본값을 `true`로 설정하고, 실제 controller 상태를 사용할 때 `false`로 끌 수 있도록 구성

변경 이유:

- 현재 운용 방침은 stack 실행 시 항상 자율주행 모드로 사용하는 것이며, 실제 controller 연결 전에도 route/planning의 operation-mode gate를 통과시킬 필요가 있었다.
- 실제 제어권과 controller 상태를 확인하지 않고 고정값을 발행하는 계약 위반 사항은 `FIXED-AUTONOMOUS-MODE-CONTRACT-DEVIATION.md`에 별도로 기록했다.

적용 파일:

- `base_ws/src/planning_addons/ajounice_planning_integration/scripts/fixed_autonomous_operation_mode_publisher`
- `base_ws/src/planning_addons/ajounice_planning_integration/launch/fixed_autonomous_operation_mode.launch`
- `base_ws/src/planning_addons/ajounice_planning_integration/launch/planning.launch`
- `base_ws/src/planning_addons/ajounice_planning_integration/launch/morai_planning_stack.launch`
- `base_ws/src/planning_addons/ajounice_planning_integration/CMakeLists.txt`
- `base_ws/src/planning_addons/ajounice_planning_integration/package.xml`

## 3. Behavior Path Planner 전방 경로 길이 축소

변경 내용:

- BPP `forward_path_length`를 300 m에서 150 m로 변경
- `behavior_path_forward_length` launch 인자를 추가하고 기본값을 `150.0`으로 설정
- Autoware 원본 설정을 수정하지 않고 통합 launch에서 절대 ROS parameter로 덮어쓰도록 구성

변경 이유:

- BPP가 만든 긴 경로 전체가 BVP 입력으로 전달되어 BVP 플러그인의 처리량과 최종 trajectory latency가 증가했다.
- 실제 운용에 필요한 전방 범위는 유지하면서 BVP에 전달되는 point 수와 계산량을 줄이기 위해 변경했다.

적용 파일:

- `base_ws/src/planning_addons/ajounice_planning_integration/launch/planning.launch`
- `base_ws/src/planning_addons/ajounice_planning_integration/launch/morai_planning_stack.launch`

## 4. Intersection attention area 길이 축소

변경 내용:

- Intersection `common.attention_area_length`를 200 m에서 100 m로 변경
- `intersection_attention_area_length` launch 인자를 추가하고 기본값을 `100.0`으로 설정
- Autoware 원본 설정을 수정하지 않고 통합 launch에서 절대 ROS parameter로 덮어쓰도록 구성

변경 이유:

- v61 지도에서 200 m 범위로 충돌 차선과 선행 차선을 탐색할 때 특정 구간의 Intersection 계산량이 크게 증가했다.
- 현재 150 m planning horizon 안에서 교차로 탐색 범위를 100 m로 제한하여 불필요한 선행 차선 탐색과 BVP 병목을 줄이기 위해 변경했다.

적용 파일:

- `base_ws/src/planning_addons/ajounice_planning_integration/launch/planning.launch`
- `base_ws/src/planning_addons/ajounice_planning_integration/launch/morai_planning_stack.launch`

## 5. Planning RViz 설정 추가

변경 내용:

- 기존 base planning 시각화 구성을 바탕으로 AW149 planning용 RViz 설정과 launch 추가
- v61 lanelet map, mission route, 최종 trajectory, 현재 차량 위치, Route A 시작점을 표시하도록 topic 구성
- Fixed Frame을 `map`, TopDownOrtho Target Frame을 `base_link`로 설정

변경 이유:

- 현재 차량 위치와 Route A, 최종 trajectory를 한 화면에서 확인하고 RViz 화면이 이동 차량을 따라가도록 하기 위해 변경했다.
- planning 계산 코드와 `planning_ws`를 수정하지 않고 시각화만 추가하기 위해 `base_ws`에 분리했다.

적용 파일:

- `base_ws/src/bringup/launch/aw149_planning_rviz.launch`
- `base_ws/src/bringup/rviz/kcity_aw149_planning.rviz`

### GNSS+IMU 2D localization용 지도 높이 표시

- production localization의 `two_d_mode: true`와 `map -> base_link z=0`은 그대로 유지한다.
- planning이 사용하는 `/map/vector_map` 원본과 `map -> morai_map` TF도 변경하지 않는다.
- AW149 planning RViz launch가 발행하는 `/map/lanelet_map` MarkerArray만
  `lanelet_ground` 프레임에서 평탄화한다.
- `map -> lanelet_ground`의 기본 z는 IONIQ 5 wheel radius와 같은
  `-0.3705 m`다. 따라서 `base_link`가 rear axle 중심에 있어도 차량 바퀴와
  시각화용 노면이 자연스럽게 맞는다.
- launch 인자 `flatten_visualization_map:=false`로 원본 고도 표시를 복원할
  수 있으며, `visualization_map_z_offset`으로 표시 높이만 조정할 수 있다.

이 변경은 RViz 전용이다. localization pose, planning trajectory, binary map
payload 및 주행 계산 결과에는 영향을 주지 않는다.

## 6. 초기 Route A 자동 요청을 RViz 수동 요청으로 변경

변경 내용:

- `auto_request_initial_route`를 `true`에서 `false`로 변경하여 planner 시작 직후의 Route A 자동 요청을 비활성화
- RViz에 `REQUEST ROUTE A` 버튼과 requester 상태 표시를 추가
- 버튼을 누르면 기존 `/competition_route_requester/request` 서비스에 `route_key=route_a`를 전달하도록 연결
- 실제 `/planning/mission_planning/route` 발행은 새 수동 publisher를 만들지 않고 기존 RouteSelector와 Mission Planner가 담당하도록 유지
- 운영 case를 `route_a_to_b_with_fallback`으로 변경
- Route A 검증 이후 Route B와 fallback 자동 전환은 유지
- planner 기동 시 requester 노드는 기본 실행하되 버튼 입력 전에는 route 요청을 하지 않도록 구성
- 비대화형 통합 probe는 private 설정에서만 초기 Route A 자동 요청을 켜도록 분리
- 이미 검증된 Route A뿐 아니라 Route B 또는 fallback이 활성화된 상태에서도 공식 start pose 3 m 이내이고 차량 속도가 0.1 m/s 이하일 때 같은 버튼으로 Route A를 다시 요청하도록 변경
- Route A 재요청 시 기존 route를 먼저 지우지 않고 요청 시각 이후의 route와 새 trajectory/validator를 검증하도록 변경
- Route B 또는 fallback에서 Route A로 돌아가는 동작은 자동화하지 않고 RViz 버튼 요청에만 허용
- 새 Route A 요청을 수락할 때 이전 주행에서 대기 중이던 fallback 요청을 폐기하도록 변경

변경 이유:

- planner 기동 시점과 MORAI 차량 배치 완료 시점이 달라, 준비되지 않은 차량 위치를 기준으로 Route A가 자동 생성되는 것을 막기 위해 변경했다.
- 운전자가 차량을 원하는 공식 시작 위치에 배치하고 정차를 확인한 뒤 Route A 생성 시점을 직접 결정하기 위해 RViz 수동 요청으로 변경했다.
- MORAI에서 차량을 공식 시작점으로 다시 옮겼는데 Route B/fallback 상태와 이전 궤적이 유지되는 문제를 해결하고, 운전자가 정차 후 버튼을 눌러 새 Route A 주행을 명시적으로 시작할 수 있도록 변경했다.
- 주행 중 오조작은 공식 시작점과 정차 조건으로 차단하고, Route A를 수락하면 이전 세션의 fallback이 새 경로를 덮어쓰지 못하도록 변경했다.

적용 파일:

- `base_ws/src/bringup/include/bringup/route_control_panel.hpp`
- `base_ws/src/bringup/src/route_control_panel.cpp`
- `base_ws/src/bringup/plugin_description.xml`
- `base_ws/src/bringup/rviz/kcity_aw149_planning.rviz`
- `base_ws/src/bringup/launch/aw149_planning_rviz.launch`
- `base_ws/src/planning_addons/competition_route_requester/config/competition_route_requester.param.yaml`
- `base_ws/src/planning_addons/competition_route_requester/src/competition_route_requester/runtime.py`
- `base_ws/src/planning_addons/competition_route_requester/src/competition_route_requester/policy.py`
- `base_ws/src/planning_addons/competition_route_requester/test/test_gateway_ros.py`
- `base_ws/src/planning_addons/competition_route_requester/test/test_policy.py`
- `base_ws/src/planning_addons/ajounice_planning_integration/launch/planning.launch`
- `base_ws/src/planning_addons/ajounice_planning_integration/launch/morai_planning_stack.launch`
- `scripts/runtime/verify_integrated_planning_output.py`
