# 고주로 최상위 차로 기본 선호

## 변경 범위

Route A/B `expected_path_lanelet_ids`의 고주로 진입~병목 구간을 다음으로 변경했다.

`201459 -> 201436 -> 201465 -> 200138 -> 201444 -> 201469 -> 200127 -> 201473 -> 201439`

이는 대회 route section에서 허용된 최상위 열이다. 그 왼쪽에 인접한 lanelet은 기존
`route_section_excluded_lanelet_ids`에 들어 있으므로 선호 대상으로 추가하지 않았다.
진입로 201086/201456에서 출발한 차량은 안전한 인접 LC를 순차적으로 수행해야 하며,
위 열을 즉시 점프하거나 다중 차선을 한 번에 횡단하도록 강제한 것이 아니다.

이전 기본 선호 `201457 -> 201435 -> 201466 -> 200137 -> 200187`을 대체한다.
201444에서 현재 선호 200187로 돌아가라고 요구하던 설정상의 불일치를 제거한다.
병목 이후 `200360 -> 201477 -> 201479 -> 201447 -> 201399 -> 201451`은 유지한다.
지도, primitive 집합, 양방향 LC 로직, 충돌/경계 검사, 속도 계획은 변경하지 않았다.

기존 waypoint들은 DefaultPlanner의 corridor/segment 생성용으로 유지했다.
이 waypoint들이 실제 ego의 필수 통과 위치나 최종 preferred라는 뜻은 아니다.

## 검증

- 최신 백의 실제 map 메시지를 별도 ROS master에 공급하고 설치된 DefaultPlanner
  plugin으로 Route A/B를 생성했다. 실제 preferred는 새 설정과 완전히 일치했다.
- Route A 50 sections, Route B 60 sections; 모든 preferred가 해당 primitives에 포함됨.
- 정책시험 13개 통과. geometry의 선호 차로 한정 progress guard 시험도 새 정책에 맞춤.
- 별도 ROS master의 requester 서비스 시험 18개 통과.
- source, install_aw149, install_planning_addons route YAML SHA-256 일치:
  `d903f8bc466e918c5e0b0c26d4372e1c55a99218f962c7c3b6e1d0cc5290d809`.

## 적용과 한계

실행 중인 planner/requester를 재시작하거나 현재 route를 강제 재발행하지 않았다.
플래닝 전체 재시작 후 Route A부터 새로 요청해야 기존 프로세스의 route 정의가 교체된다.
이번 검증은 경로 생성 검증이며 실제 조기 LC 성공/급제동 해소의 폐루프 증명은 아니다.
초기 LC가 진입로 59 km/h 상한을 고주로까지 유지하는 속도계획 문제는 별도 미해결이다.
상대차 위치에 따라 좌우 차로를 비교하는 최속 추월 정책도 이번 변경에 포함하지 않는다.
