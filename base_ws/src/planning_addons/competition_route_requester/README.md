# competition_route_requester

K-City v88의 Route A/B와 fallback 요청을 관리하는 ROS1 planning addon이다.
기본 운용은 v88 지도와 설정이다. v87 설정은 명시적 복귀용으로 보존한다.

## 실행과 설정

- [`config/kcity_v88_competition_route.yaml`](config/kcity_v88_competition_route.yaml): 지도 경로·SHA, waypoint, Route A/B/fallback 검증 순서
- [`config/competition_route_requester.param.yaml`](config/competition_route_requester.param.yaml): 기본 운용 설정. v88 Route A/B를 선택한다.
- [`config/competition_route_requester_v88.param.yaml`](config/competition_route_requester_v88.param.yaml): 명시적 v88 운용 설정
- [`docs/SPEC.md`](docs/SPEC.md): 요청·검증·상태 전환 계약

`competition_planning.launch`와 `competition_planning_v88.launch`가 공용
requester 실행 코드를 사용한다. 기본 운용 case는
`route_a_to_b_with_fallback`이며, case 변경은 YAML 수정 후 세 노드를
함께 재시작해야 적용된다.

## 실행 책임

- `route_requester`: Mission Planner 서비스 요청과 응답·경로·trajectory 검증
- `route_supervisor`: Route A/B 선택과 정상 재경로 상태 관리
- `finish_guard`: 결승선 이후 경로 확보를 위한 fallback 감시

정상 Route A/B는 `SetWaypointRoute`와 `SetPreferredPrimitive`를 사용하고,
fallback은 `SetLaneletRoute`를 사용한다. 실제 `LaneletRoute`는 기존
RouteSelector/Mission Planner가 생성하며 addon은 별도 route mux를 두지 않는다.

잘못된 operation case와 지도 SHA는 요청 전에 거부한다. 서비스 응답이
불명확하거나 성공 응답 후 경로 검증이 실패하면 `RECONCILIATION_REQUIRED`로
고정한다. 이 경우 자동 clear/retry를 하지 않으며 운영자가 실제 route와 UUID를
확인해야 한다.

RViz의 `Route Status`는 `verified_active`를 기준으로 Route A/B를 표시한다.
새 route를 검증 중이거나 fallback이 활성화된 동안에는 두 표시등이 꺼진다.
새 trajectory와 validator 출력은 요청 이후의 시간상 연관 증거이며,
실제 폐루프 주행 검증을 대신하지 않는다.

이전 버전 route 설정, 설명 그림과 일회성 생성 도구는 배포에서 제외한다.
작업 중 생성한 그림과 검증 기록은 Git 제외 경로인 `planner_logs/analysis/`에 둔다.

## v88 선호 차선

Route A/B와 fallback 모두 `201439 → 200360 → 201479 → 201451`을 사용한다.
200360은 앞쪽 병합 직선, 201479는 뒤쪽 병합 직선이다. 각 속도는 150/80/70 km/h다.
Route B가 다른 상류 차선을 선택해도 `preferred_lanelet_overrides`가 해당 구간의
200185/201448을 200360/201479로 선택하며, 후보가 없는 경우 요청은 실패 처리한다.
