# competition_route_requester

K-City v68 대회 경로를 요청하기 위한 AjouNice 전용 planning addon이다.

ROS1 실행 노드 3개(`route_requester`, `route_supervisor`, `finish_guard`)와 서비스 gateway를 구현했다. **구현·단위검사와 실차/전체 파이프라인 운용 승인은 다르다.** 현재 운용 기본값은 `route_a_to_b_with_fallback`이며 초기 Route A는 자동 요청하지 않고 RViz의 명시적 요청을 사용한다. Route A 검증 뒤 Route B/fallback 상태 기계는 자동으로 동작한다. 검증 범위와 미완료 사항은 [통합 인계서](../../../../../docs/planning/INTEGRATION-HANDOFF.md)를 참고한다.

패키지 전체는 competition route input manager를 담지만, 향후 실행 node의 책임은 분리한다. `route_requester`는 RouteSelector main 요청 gateway이고 `route_supervisor`가 Route A/B 정책과 정상 재경로 상태 기계를 소유한다. finish guard와 fallback request builder는 독립 boolean으로 켜지 않고 선택한 operation case에 따라 필요한 경우에만 동작한다.

이 addon은 Mission Planner가 발행한 route 뒤에 별도 mux를 붙이지 않는다. 정상 Route A/B는 `SetWaypointRoute`로 고주로의 병렬 segment를 포함한 경로를 만든 뒤 `SetPreferredPrimitive`로 각 segment의 선호 차선을 확정한다. fallback은 명시적인 lanelet segment를 담은 `SetLaneletRoute`로 RouteSelector main 입력에 전달한다. 실제 `LaneletRoute` 생성과 BPP 전달은 기존 RouteSelector/MissionPlanner 흐름을 그대로 사용한다.

YAML의 `route_operation_case`는 `route_a_only`, `route_b_only`, `route_a_to_b`, `route_a_to_fallback`, `route_a_to_b_with_fallback` 중 하나만 허용한다. launch 파일은 case 인자를 받지 않고 공통 YAML을 로드한다. case는 주행 중 변경하지 않으며 YAML 수정 후 전체 launch를 다시 시작한다. 각 case의 동작은 [`docs/SPEC.md`](docs/SPEC.md#41-yaml-only-operation-case-원칙)에 정의한다.

## 현재 포함 범위

- [`docs/SPEC.md`](docs/SPEC.md): requester 동작 명세와 구현 시작 조건
- [`docs/assets/kcity_v68_route_a_b.svg`](docs/assets/kcity_v68_route_a_b.svg): v68 OSM 중심선에서 생성한 Route A/B 및 재경로 deadline 도식
- [`docs/assets/kcity_v68_operation_cases.svg`](docs/assets/kcity_v68_operation_cases.svg): 5개 operation case와 RouteSelector 입력 관리자 구조
- [`docs/assets/kcity_v68_route_a_detail.svg`](docs/assets/kcity_v68_route_a_detail.svg): Route A 요청 waypoint와 검증 lanelet 전체 순서
- [`docs/assets/kcity_v68_route_b_detail.svg`](docs/assets/kcity_v68_route_b_detail.svg): `200178`에서 시작하는 Route B 재경로 요청과 검증 lanelet 전체 순서
- [`docs/assets/kcity_v68_reroute_process.svg`](docs/assets/kcity_v68_reroute_process.svg): trigger, guard, 요청·검증, deadline 절차
- [`docs/assets/kcity_v68_highway_lane_strategy.svg`](docs/assets/kcity_v68_highway_lane_strategy.svg): 고속주로 우측·중간·최상위 차선 topology와 선택 경로 비교
- [`config/kcity_v68_competition_route.yaml`](config/kcity_v68_competition_route.yaml): K-City v68 Route A/B 정의
- [`../../../../kcity_map/docs/assets/kcity_v67_competition_intersections_detail.svg`](../../../../kcity_map/docs/assets/kcity_v67_competition_intersections_detail.svg): 7개 교차로 polygon과 30 km/h 적용 lanelet 상세
- [`config/competition_route_requester.param.yaml`](config/competition_route_requester.param.yaml): 기존 동작 파라미터 초안

`competition_route_requester.param.yaml`은 현재 ROS1 flat YAML이며 `/planning/mission_planning/route`를 사용한다. 잘못된 case/지도 SHA는 구독·서비스 생성 전 거부한다. 서비스 응답 불명확 또는 서비스 성공 후 검증 실패는 `RECONCILIATION_REQUIRED`로 고정하고 자동 재요청하지 않는다. 이 상태에서 Route A가 그대로라고 가정하거나 프로세스 재시작만으로 해결하지 않는다. 운영자가 실제 RouteSelector 상태와 UUID를 먼저 확인해야 한다.

`/planning/mission_planning/route_marker`는 선호 lanelet만이 아니라 각 route segment의 주행 가능 후보인 `primitives` 전체를 표시한다. K-City v68 Mission Planner 설정은 `200130` segment를 `[200127, 200130, 200134]`로 구성하고, 다음 합류 segment를 `[201439, 201441, 201446]`으로 이어 세 차선이 끊기지 않고 한 차선으로 모이게 한다. `201447`과 `201401` segment에서는 각각 `[201447]`, `[201401]`만 남긴다. 이는 marker만 거르는 설정이 아니라 발행되는 실제 route를 수정하며 선호 lanelet 순서는 유지한다.

운영 RViz의 `Route Control` 패널은 `/competition_route_requester/request`에 `route_key=route_a`를 호출한다. 이는 LaneletRoute 직접 publisher가 아니며 기존 gateway의 map, localization, 공식 시작 구간, RouteSelector 응답 및 trajectory/validator 검증을 그대로 거친다. 최초 요청과 gateway가 검증한 Route A/B/fallback의 재시작은 모두 공식 start pose 6 m 이내에서만 허용되며, 재시작은 속도 0.1 m/s 이하 정차를 추가로 요구한다. 기존 route를 먼저 지우지 않고 새 route stamp와 새 trajectory/validator 증거가 확인된 뒤 Route A의 새 generation으로 검증한다. Route B 또는 fallback 활성화 뒤 Route A 복귀는 자동 전환이 아니라 운전자가 버튼을 누른 경우에만 허용하며, 이전 주행에서 대기 중이던 fallback 요청은 폐기한다.

Route B 이동 중 재시도는 `200346` 이전 현재 lanelet부터의 **정확한 canonical suffix**만 수신 경로로 허용한다. 출발 lanelet이 달라졌다는 이유로 경유 구간 누락이나 임의 새 경로를 허용하지 않는다. trajectory에는 route UUID가 없으므로 요청 응답·경로 확인 이후 source timestamp와 20개 validator flag, 3개 새 출력은 시간상 연관 증거이지 완전한 route 인과성/폐루프 증명이 아니다.

## 디렉터리 소유권

- `planning_ws/src`: upstream Autoware Universe ROS1 포팅 코드(149 + 지원 5개)
- `base_ws/src/planning_addons`: AjouNice 대회 전용 추가 기능

이 패키지는 upstream 포팅 코드를 수정하지 않고 RouteSelector의 main route 입력을 이용하는 addon으로 구현한다.

## 경로 시각화

### 고주로 선호 차선

![K-City v68 고주로 선호 차선](docs/assets/kcity_v68_highway_lane_strategy.svg)

`200137`은 기존 `200128` 다음에 빠진 직렬 구간이 아니라 그와 나란한 상위 차선이다. 적용 경로는 고주로 진입 시 현재 `200101`에서 같은 route segment의 `201434`를 선호하도록 만든 뒤 `201435 → 200137 → 200187 → 201443 → 200130 → 201441 → 200360`으로 이어진다. `201443`은 중간 차선이고 같은 위치의 최상위는 `200188`이므로, `201436`까지 올라갔다가 다시 내려오는 경로는 사용하지 않는다.

![Competition route operation cases](docs/assets/kcity_v68_operation_cases.svg)

다섯 operation case 모두 RouteSelector의 main 입력을 사용한다. 정상 Route A/B는 `SetWaypointRoute`, fallback은 `SetLaneletRoute` 요청이며 Mission Planner 출력 이후에는 별도 mux를 두지 않는다. fallback은 변경된 중간 차선의 `201441` s 8 m에서 시작해 Mission의 최소 30 m reroute 안전거리를 보존하며 goal `200210`까지 이어지는 28개 lanelet suffix로 고정한다. `200360` s 8 m는 2 m/s에서도 공통 길이가 26.751 m뿐이라 안전검사에서 거절된다.

![K-City v68 Route A/B와 재경로 구간](docs/assets/kcity_v68_route_a_b.svg)

도식은 `config/kcity_v68_competition_route.yaml`의 `expected_path_lanelet_ids`를 순서대로 읽고, SHA-256가 고정된 `lanelet2_map_v68.osm`의 실제 중심선을 투영해 만든다. 공식 START/END는 `200892` 끝과 `200889` 시작의 경계다. 파란색은 Route A의 출발 전용 구간, 청록색은 `200178`부터 `200181`까지 Route A/B 공통 구간, 주황색은 Route B가 `200892`로 재진입해 최종 goal `200210`까지 진행하는 연장 구간이다. 확대도에서 trigger는 `200178`의 s 8 m, deadline은 `200346` 진입 전이다.

### Route A 상세

![K-City v68 Route A 상세](docs/assets/kcity_v68_route_a_detail.svg)

지도 위 숫자는 `route_a.waypoint_refs`의 요청 순서다. 오른쪽 위 표는 실제 요청에 들어가는 waypoint와 goal을, 아래 표는 선호 차선 적용 뒤 수신 경로 검증에 사용하는 47개 canonical `expected_path_lanelet_ids`를 나타낸다. 시작점은 공식 `200892`→`200889` 경계의 현재 odometry이고, Route A는 loop 방지를 위해 `200892` 재진입 전 `200181`의 0.75 지점에서 끝난다.

### Route B 재경로 상세

![K-City v68 Route B 재경로 상세](docs/assets/kcity_v68_route_b_detail.svg)

지도 위 숫자는 `route_b.waypoint_refs`의 요청 순서다. Route B 요청 시 차량이 lanelet `200178`의 s 8 m 이상에 있으므로 `wp_200178`은 waypoint 요청에서 의도적으로 제외한다. 결과 검증 순서는 현재 start인 `200178`부터 시작하며 최종 goal `200210`까지 57개 lanelet을 확인한다. `route_b_only`도 공식 START/END가 아니라 이 시작 조건에서 시험한다.

### 재경로 지점과 절차

![K-City v68 Route A에서 Route B로의 재경로 절차](docs/assets/kcity_v68_reroute_process.svg)

재경로는 `200178`의 s 8 m 이상에서 준비하고, 최신 odometry와 reroute availability의 age 및 0.5초 hold 조건을 통과한 뒤 요청한다. `ClearRoute`는 호출하지 않는다. 서비스 성공 후 Route B UUID, goal, lanelet 순서와 새 trajectory 검증이 끝나야 전환 완료로 처리한다. 이 사이 실제 경로는 이미 바뀔 수 있으므로 Route A 유지로 단정하지 않는다. 적용 deadline은 lanelet `200346` 진입 전이고, `wp_200346`의 s 8 m pose는 deadline 감지 anchor이므로 둘을 같은 위치로 해석하지 않는다.

OSM과 projector 파일이 있는 저장소 루트에서 다음 명령으로 SVG 6개를 다시 생성하고 입력 hash·경로 연속성·설정 pose·fallback suffix·operation case를 검증할 수 있다.

```bash
python3 base_ws/src/planning_addons/competition_route_requester/tools/generate_route_visualization.py
```

생성 도구는 문서 전용이며 ROS를 import하거나 경로를 요청하지 않는다. `PyYAML`과 `pyproj`가 필요하다.
