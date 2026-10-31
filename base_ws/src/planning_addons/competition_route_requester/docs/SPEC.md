# Competition Route Requester 명세

## 1. 상태

- 문서 상태: 정책 원본 + 아래 ROS1 구현 계약 revision 1
- 패키지 상태: gateway/supervisor/finish_guard 실행 코드 및 launch 구현
- 실제 주행 승인: 미완료. 이동 중 A→B/fallback 및 base 입력 계약 검증 필요

### ROS1 구현 계약 revision 1

이 단락은 아래의 구현 전 임시 인터페이스 표현보다 우선한다.

- gateway: `/planning/set_waypoint_route` (`autoware_internal_planning_msgs/SetWaypointRoute`), `/planning/mission_planning/mission_planner/set_preferred_primitive` (`autoware_planning_msgs/SetPreferredPrimitive`), `/planning/set_lanelet_route` (`autoware_planning_msgs/SetLaneletRoute`).
- 출력 관찰: `/planning/mission_planning/route` (`autoware_planning_msgs/LaneletRoute`), `/planning/route_state` (`autoware_planning_msgs/RouteState`).
- YAML은 ROS1 flat 구조이며 5개 case 중 하나만 startup에 고정한다.
- 초기 A는 canonical 48개 또는 incoming 200892를 포함한 49개만 허용한다.
- 이동 중 B 재시도는 설정된 deadline 이전 현재 시작 lanelet에 해당하는 canonical B의 정확한 suffix를 허용한다. 이미 지난 lanelet을 첫 segment로 강제하지 않으며 중간 생략/순서 변경은 허용하지 않는다.
- 서비스 거부와 결과 불명확을 구별한다. transport timeout, 응답 성공 후 route/trajectory 검증 실패는 `RECONCILIATION_REQUIRED`. 이때 Route A 유지로 단정하거나 자동 clear/retry하지 않는다.
- 성공 증거는 route UUID/goal/순서, 응답 및 route 확인 이후 timestamp의 3개 새 trajectory와 validator 20개 flag다. Route UUID가 없는 trajectory의 완전한 인과성 또는 주행 안전 인증은 아니다.
- 자동 운용 기본값은 `route_a_to_b_with_fallback`이며, 실제 주행 검증 여부는 별도로 확인한다.

이 명세는 requester의 동작과 안전 조건을 먼저 고정한다. ROS1 서비스명, 메시지 타입, 토픽명 및 launch 구성은 upstream 포팅 결과를 확인한 뒤 확정한다.

## 2. 목적과 범위

`competition_route_requester`는 K-City v87 대회 주행을 위해 Route A/B 요청, 정상 전환 정책, 독립 감시 및 결승선 fallback을 단계적으로 검증하는 AjouNice 전용 addon이다.

패키지 이름은 requester이지만 실행 책임은 하나의 거대한 requester node에 모으지 않는다. `route_requester`는 현재 차량 위치와 sparse waypoint, goal을 `SetWaypointRoute` 요청으로 전달하는 저수준 실행기이고, `route_supervisor`가 Route A/B 상태와 정상 전환 정책을 소유한다. fallback component는 별도로 격리한다.

정상 동작과 fallback 모두 addon이 `LaneletRoute`를 직접 발행하지 않는다. addon은 RouteSelector의 main 입력에 넣을 `SetWaypointRoute` 또는 `SetLaneletRoute` 요청을 관리하고, 실제 `LaneletRoute` 생성과 발행은 항상 Mission Planner에 맡긴다. 정확한 ROS1 message/service 이름은 포팅본 계약을 확인한 뒤 고정한다.

다음 기능은 범위 밖이다.

- Mission Planner 알고리즘 변경
- RouteSelector 재구현
- Behavior Path Planner 변경
- Lanelet map 적재 및 projector/TF 구성
- upstream ROS2 알고리즘의 임의 수정

## 3. upstream 기준과 의존성

포팅 기준은 Autoware Universe 0.51.0의 고정 소스 경계다.

- Autoware Universe commit: `d4d260983d357e1b2b34291d91933f9f4b53bf94`
- 대상 package: `autoware_mission_planner_universe`
- 대상 component: `RouteSelector`, `MissionPlanner`, `DefaultPlanner`

ROS2 upstream에서 `RouteSelector`는 별도 패키지가 아니라 `autoware_mission_planner_universe` 내부 component다. 정상 경로와 MRM 경로 각각에 Mission Planner와 같은 요청 인터페이스를 제공하고, 현재 MRM 상태에 따라 요청과 결과를 분배한다.

참고용 ROS2 정상 경로 서비스는 다음과 같다.

```text
/planning/mission_planning/route_selector/main/set_waypoint_route
```

이 이름과 ROS2 서비스 타입은 ROS1 런타임 계약이 아니다. 포팅본에서 namespace, remap 및 메시지 변환 결과를 확인한 뒤 확정한다.

## 4. 논리 데이터 흐름

```text
route_supervisor --정상 A/B 요청-------------------┐
                                                  v
finish_guard -> fallback_request_builder -> route_requester
                                                  |
                         한 번에 하나의 요청 직렬화/우선순위 선택
                                                  |
                                                  v
            RouteSelector main SetWaypointRoute/SetLaneletRoute 입력
                                                  |
                                                  v
                                           MissionPlanner
                                                  |
                                          실제 LaneletRoute
                                                  |
                                                  v
                                      Behavior Path Planner
```

각 component의 책임은 다음과 같다.

- `route_requester`: supervisor/guard가 만든 요청 중 한 번에 하나만 선택해 RouteSelector main 서비스로 전달하고 응답을 돌려주는 단일 gateway
- `route_supervisor`: 초기 Route A 요청, 현재 진행도와 Route A/B 적용 상태 판단, 정상 Route B 전환 및 재시도 정책
- `finish_guard`: supervisor와 독립적으로 결승선 통과 가능성을 감시하고 fallback 요청 생성을 지시
- `fallback_request_builder`: 현재 위치 이후의 사전 검증 lanelet segment와 결승선 이후 goal을 가진 `SetLaneletRoute` 요청 준비

따라서 `route_requester`는 route supervisor가 아니다. supervisor와 guard가 정책을 결정하고 requester는 요청을 실행한다. 여기서 선택하는 것은 Mission Planner 출력 route가 아니라 Mission Planner로 들어갈 route request다. requester는 RouteSelector의 main 입력만 사용하며 MRM 입력을 호출하지 않는다.

RouteSelector는 main/MRM 요청을 하나의 Mission Planner로 전달하고 Mission Planner의 route/state를 현재 선택된 인터페이스로 돌려주는 upstream component다. competition addon은 이 구조 뒤에 별도 route mux를 추가하지 않는다. Behavior Path Planner와 controller 사이에도 addon을 삽입하지 않는다.

### 4.1 YAML-only operation case 원칙

주행 동작의 단일 기준은 `config/competition_route_requester.param.yaml`의 `route_operation_case`다. launch 파일은 동작 선택 인자를 받지 않는다. launch는 이 YAML을 그대로 로드하고 고정된 전체 node graph를 실행하며, 각 node는 선택된 case에서 자신에게 할당된 역할만 수행한다.

case 변경 절차는 `YAML 수정 -> 전체 launch 재시작`으로 고정한다. 주행 중 rosparam 변경이나 별도 launch argument로 case를 바꾸지 않는다.

운영 기본 case는 `route_a_to_b_with_fallback`이고 `auto_request_initial_route=true`다. planner session이 시작되면 map, route state, operation mode와 최신 odometry를 기다린 뒤 현재 lanelet/arc로 초기 요청을 선택한다. 공식 S부터 `200258` 진입 전까지는 Route A, `200258` 진입부터 `200892` 내부까지는 Route B를 요청한다. 허용 구간 밖에서는 route를 요청하지 않는다. 이전 session의 route가 RouteSelector에 남아 있어도 이를 채택하지 않고 새 `SetWaypointRoute` 요청으로 교체하며 `ClearRoute`는 호출하지 않는다. 새 route stamp와 새 trajectory/validator를 확인해야 startup generation이 검증된다.

RViz `Route Status` 패널은 gateway 상태의 `verified_active`를 읽는 전용 표시기다. Route A 또는 Route B가 검증되면 해당 표시등만 켜지고, fallback이 활성화됐거나 새 route를 아직 검증 중이면 두 표시등 모두 꺼진다. RViz에서는 route 요청을 보내지 않는다.

독립적인 `enable_*` boolean은 두지 않는다. 기능 간 의존성이 있는데 boolean을 독립적으로 두면 32개 조합이 생기고 의미 없거나 안전하지 않은 조합을 모두 검증해야 하기 때문이다. 허용되는 동작은 아래 5개 case로 제한한다.

| `route_operation_case` | 초기 route | 정상 A→B | 결승선 fallback | 시험 목적 |
|---|---|---:|---:|---|
| `route_a_only` | S–1 구간 A | off | off | Route A 단독 요청·주행 검증 |
| `route_b_only` | 1–deadline 구간 B | off | off | `startup_route_b_trigger` 구간에서 Route B 단독 검증 |
| `route_a_to_b` | 위치 기반 A/B | on | off | 정상 A→B 재경로만 검증 |
| `route_a_to_fallback` | S–1 구간 A | off | on | 정상 Route B 요청 없이 fallback만 검증 |
| `route_a_to_b_with_fallback` | 위치 기반 A/B | on | on | 정상 A→B를 시도하고 실패 시 fallback 인계 검증 |

`route_a_to_fallback`에서는 hard cutoff 전까지 Route A를 유지하고, cutoff에서 명시적 lanelet segment를 담은 fallback `SetLaneletRoute`를 RouteSelector main 입력으로 보낸다. `route_a_to_b_with_fallback`에서는 먼저 정상 Route B `SetWaypointRoute`를 시도하며, 결승선 통과 가능한 Route B가 확인되지 않은 경우에만 같은 fallback 요청으로 승격한다.

목록에 없는 case는 시작 전에 거부한다. 각 case에서 필요하지 않은 finish guard 또는 fallback builder 로직은 서비스 호출이나 요청 생성을 하지 않는다.

#### case 입력 검증 계약

1. YAML을 읽은 직후 ROS subscriber, timer 및 route service 요청을 활성화하기 전에 `route_operation_case`를 검증한다.
2. 값은 대소문자를 구분한 exact string match로 비교한다.
3. 허용값은 표에 기록된 5개뿐이며 별칭, 빈 문자열 및 자동 기본값 대체를 허용하지 않는다.
4. 허용되지 않은 값이면 fatal 오류에 입력값과 허용값 목록을 남기고 route 서비스를 한 번도 호출하지 않은 채 시작을 중단한다.
5. 실행 중 parameter 변경을 감지해 case를 전환하지 않는다. 변경하려면 YAML을 수정하고 전체 launch를 다시 시작한다.

## 5. 경로 정의

경로 데이터의 단일 기준은 `config/kcity_v87_competition_route.yaml`이다.
이전 버전 설정과 설명 그림은 현재 배포에 포함하지 않는다.

- map version: 87
- map SHA-256: `70f2cdf7ee9dee3333d6553787525890c0a73c4fbcdfdcc9fc95a5c92c687575`
- 공식 START/END: `200892` 끝과 `200889` 시작의 경계
- Route A: waypoint 16개, canonical 경로 48개 lanelet, goal `200181`
- Route B: waypoint 15개, canonical 경로 58개 lanelet, goal `200210`
- fallback: `201439`부터 goal `200210`까지 30개 lanelet

UUID, waypoint pose와 전체 lanelet 순서는 YAML에서 읽는다. `waypoint_refs`는
서비스 요청 지점이며 `expected_path_lanelet_ids`는 응답 경로의 순서를 검증하는
목록이다. 두 목록을 같은 의미로 해석하지 않는다. 공식 start에서 incoming
`200892`가 포함되는 경우는 `accepted_start_prefixes`로 검증한다.

### 5.1 초기 경로와 재경로 조건

초기 Route A/B 선택은 `startup_route_b_trigger`와 현재 odometry로 판단한다.
현재 v101의 초기 선택·Route A→B 재경로 trigger는 Spawn 5번에서 주행 방향 뒤쪽 5 m인
`200310`의 s=3.149898 m (map x=71.238562, y=540.694727)이고, deadline은
`200315` 진입 전이다. 감지 anchor의 pose와 차로 진입 경계를
혼동하지 않는다. 요청 start pose는 Mission Planner가 현재 odometry에서 취득한다.

고주로에서는 waypoint planner가 만든 병렬 route segment를 확인한 뒤
`SetPreferredPrimitive`로 YAML의 선호 lanelet 순서를 적용한다. 실제 LC 시작과
주변 차량에 대한 안전 판단은 BPP가 수행한다. 지도 병합 전 ID 목록을 재사용하지 않는다.

### 5.2 fallback cutoff

cutoff는 `fallback_route.hard_cutoff_pose_ref`가 가리키는 pose다. v87에서
`201439`의 local s는 43.191875 m이다. lanelet 병합으로
시작점이 달라질 수 있으므로 이전 버전의 local s를 사용하지 않는다.

요청을 보내기 전 availability와 최신 입력을 확인하고, 성공 응답 뒤에도 UUID,
goal, 선호 lanelet 순서와 새로운 trajectory/validator 출력을 검증해야 한다.
검증이 끝나기 전에는 전환 완료로 판단하지 않는다. 응답이 불명확하거나
성공 응답 후 검증이 실패하면 `RECONCILIATION_REQUIRED`로 처리한다.

## 6. 상태 기계

```text
VALIDATE_OPERATION_CASE
  -> WAIT_FOR_STACK
  -> WAIT_FOR_FRESH_ODOMETRY
  -> case 분기

route_a_only:
  REQUEST_ROUTE_A -> VERIFY_ROUTE_A -> ROUTE_A_ACTIVE

route_b_only:
  REQUEST_ROUTE_B_FROM_CURRENT_ODOM -> VERIFY_ROUTE_B -> ROUTE_B_ACTIVE

route_a_to_b:
  REQUEST_ROUTE_A -> ROUTE_A_ACTIVE
  -> ARM_ROUTE_B_AT_TRIGGER -> REQUEST_ROUTE_B -> VERIFY_ROUTE_B
  -> ROUTE_B_ACTIVE 또는 REROUTE_FAILED

route_a_to_fallback:
  REQUEST_ROUTE_A -> ROUTE_A_ACTIVE
  -> WAIT_FOR_FALLBACK_CUTOFF -> REQUEST_FALLBACK_LANELET_ROUTE
  -> VERIFY_FINISH_CAPABLE_ROUTE -> FINISH_ROUTE_ACTIVE

route_a_to_b_with_fallback:
  REQUEST_ROUTE_A -> ROUTE_A_ACTIVE
  -> ARM_ROUTE_B_AT_TRIGGER -> REQUEST_ROUTE_B -> VERIFY_ROUTE_B
  -> 성공: ROUTE_B_ACTIVE
  -> cutoff까지 미확인: REQUEST_FALLBACK_LANELET_ROUTE
  -> VERIFY_FINISH_CAPABLE_ROUTE -> FINISH_ROUTE_ACTIVE
```

위 상태 기계는 `route_supervisor`가 소유한다. `route_requester`는 `IDLE -> REQUEST_IN_FLIGHT -> RESPONSE`의 단순 실행 상태만 가지며 주행 구간이나 fallback 정책을 판단하지 않는다.

### 6.1 Route A

1. map, localization, RouteSelector 및 Mission Planner 준비를 확인한다.
2. 최신 odometry가 공식 S부터 `200258` 진입 전까지의 시작 구간인지 확인한다.
3. 현재 pose를 start로 사용해 Route A를 요청한다.
4. 서비스 응답만으로 성공 처리하지 않는다.
5. 요청 UUID를 가진 route publication과 새 trajectory 생성을 확인한 뒤 `ROUTE_A_ACTIVE`로 전환한다.
6. planner startup에서는 기존 route를 채택하지 않고 새 요청으로 교체하며, 실패하면 설정된 간격으로 재시도한다.
7. 이미 이 gateway가 Route A를 검증한 상태의 수동 재요청은 공식 start pose 6 m 이내, 최신 odometry, 속도 0.1 m/s 이하에서만 허용한다.
8. 재요청은 기존 route를 clear하지 않으며 요청 시각 이후의 route stamp와 새 trajectory/validator 연속 3개를 확인해 generation을 갱신한다.

### 6.2 Route B

1. planner startup이 `200258` 진입 이후부터 `200892` 내부이면 Route B를 초기 route로 요청한다. Route A 주행 중 재경로는 별도의 `reroute_trigger`와 deadline을 따른다.
2. 최신 reroute availability가 연속 hold time 동안 true인지 확인한다.
3. 현재 pose를 start로 사용해 Route B를 요청한다.
4. waypoint 결과의 UUID, goal, 병렬 segment 구성을 확인한 뒤 `SetPreferredPrimitive`로 고주로 선호 차선을 적용한다.
5. Route B UUID, goal, 최종 선호 lanelet 순서 및 새 trajectory를 검증한다.
6. 검증이 끝난 경우에만 Route B가 활성화됐다고 판단한다.
7. 실패하면 Route A를 유지한 채 조건을 다시 확인하고 deadline 전까지 재시도한다.

### 6.3 결승선 fallback

1. fallback을 포함한 두 case에서만 finish guard를 활성화한다.
2. 현재 선택 route가 결승선 이후 goal `200210`까지 이어지는지 UUID, goal 및 lanelet suffix로 판정한다.
3. `route_a_to_fallback`에서는 정상 Route B waypoint 요청을 하지 않고 `fallback_route.hard_cutoff_pose_ref`의 hard cutoff에서 fallback을 시작한다.
4. `route_a_to_b_with_fallback`에서는 Route B 적용을 먼저 시도하고, YAML의 fallback cutoff까지 결승선 통과 가능한 Route B가 확인되지 않은 경우에만 fallback으로 승격한다.
5. fallback builder는 `fallback_route.expected_path_lanelet_ids`의 30개 segment와 goal `200210`을 가진 `SetLaneletRoute` 요청을 만든다. 요청 위치는 YAML의 cutoff pose를 따르며 Mission Planner의 reroute safety 검사를 통과해야 한다.
6. requester는 진행 중인 정상 요청과 fallback 요청을 동시에 보내지 않으며 fallback 요청에 우선권을 준다.
7. fallback 요청도 RouteSelector main 입력과 Mission Planner의 reroute availability 및 safety 검사를 그대로 통과해야 한다.
8. Mission Planner가 발행한 새 route와 trajectory가 검증된 뒤에만 `FINISH_ROUTE_ACTIVE`로 판단한다.

requester의 trajectory age 상한은 canonical Planning Validator latency checker와 같은 1.0초다. requester는 validator 20개 flag와 연속 3개 새 출력을 추가로 요구한다. 실제 제어 송신단의 0.5초 source-age 안전정지는 별도이며 완화하지 않는다.

## 7. 안전 불변 조건

- requester는 `ClearRoute`를 호출하지 않는다.
- 같은 Route A의 수동 갱신은 현재 gateway의 VERIFIED A, 공식 start 6 m 이내 및 정차 조건에서만 허용한다.
- Route B의 요청과 검증이 성공하기 전에는 Route A를 유지한다.
- Route A는 공식 start lanelet인 `200892`로 재진입하지 않고 `200181`에서 끝나야 한다.
- 오래된 odometry 또는 오래된 reroute availability로 요청하지 않는다.
- 서로 다른 route에는 서로 다른 고정 UUID를 사용한다.
- 수신 route의 UUID와 goal만이 아니라 예상 lanelet 순서도 확인한다.
- route 교체 후 새 trajectory가 확인되어야 성공으로 처리한다.
- Route B는 `200315` 도달 전에 적용되어야 한다.
- 최종 goal `200210`은 결승선 이후에 있어야 한다.
- 대회 production에서는 `route_a_to_b_with_fallback` case를 사용한다.
- fallback이 없는 `route_a_to_b`에서 Route B가 적용되지 못하면 명시적인 `REROUTE_FAILED`로 전환한다.
- fallback이 있는 두 case에서는 cutoff까지 결승선 통과 route가 확인되지 않으면 fallback 요청으로 승격한다.

## 8. 파라미터 초안의 취급

`config/competition_route_requester.param.yaml`은 기존에 정리한 동작 정책을 보존하기 위한 초안이다.

현재 확정 가능한 값은 다음과 같다.

- YAML-only `route_operation_case`와 5개 허용 case
- Route A only, Route B only, A-to-B, A-to-fallback, A-to-B-with-fallback 선택
- 자동 Route A 요청 여부
- odometry/reroute availability 최대 age
- reroute availability hold time
- 서비스 및 검증 timeout
- 재시도 간격과 횟수
- 새 trajectory 검증 여부
- `never_clear_route`

다음 항목은 ROS1 포팅 전까지 확정하지 않는다.

- ROS1 parameter 구조
- RouteSelector service 이름과 service type
- route/state/trajectory topic 이름과 message type
- operation mode topic과 사용 조건
- reroute availability topic과 message type
- ROS2 QoS의 ROS1 queue/latch 대응

## 9. 구현 시작 게이트

다음 조건이 모두 충족된 뒤 requester 코드를 작성한다.

1. ROS1 `RouteSelector`와 `MissionPlanner`가 `base_ws/src/planning`에 들어온다.
2. main `SetWaypointRoute` 요청이 ROS1에서 빌드되고 수동 호출된다.
3. canonical route/state 출력과 Route UUID 전달 규칙이 확인된다.
4. reroute availability 및 trajectory 계약이 실제 포팅 checkout에서 확인된다.
5. Route A를 유지한 상태에서 Route B 요청이 가능한지 통합 시험으로 확인된다.
6. 확인된 인터페이스에 맞춰 파라미터 YAML을 ROS1 형식으로 다시 작성한다.

## 10. 구현 후 인수 조건

- Catkin build와 package test가 통과한다.
- 잘못된 map SHA에서 route 요청을 거부한다.
- stale odometry와 stale reroute availability에서 요청을 거부한다.
- Route A가 예상 lanelet 순서와 UUID로 설정된다.
- `200258`의 trigger 전에는 Route B를 요청하지 않는다.
- reroute availability가 false인 동안 Route B를 요청하지 않는다.
- Route B 실패 시 Route A가 계속 유지된다.
- requester 실행 중 `ClearRoute` 호출이 한 번도 발생하지 않는다.
- Route B가 `200315` 전에 적용되고 최종 goal이 `200210`으로 검증된다.
- 선택된 case에서 사용하지 않는 reroute/fallback 로직이 서비스 호출이나 요청 생성을 하지 않는다.
- 정의되지 않은 `route_operation_case`는 주행 시작 전에 거부된다.
- 각 operation case가 표에 정의된 route 요청만 수행한다.
- `route_a_to_fallback`에서 정상 rerouting 없이 fallback SetLaneletRoute 요청 인계가 검증된다.
- launch 파일에는 operation case argument가 없고 동일한 YAML만 로드한다.
- addon이 Mission Planner 출력 route나 BPP/controller 데이터 경로에 별도 publisher/mux를 삽입하지 않는다.
