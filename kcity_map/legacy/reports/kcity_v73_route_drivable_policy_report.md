# v73 route 주행 가능 차선 수정

## 범위

지도 OSM은 출시된 v73 그대로 유지한다. 지도에서 도로를 삭제하는 작업이 아니라 Mission Planner가 발행하는 route segment의 primitives를 수정한다.

- 모든 자동 수집/override 구간에서 제외: `201400, 201440, 200148, 201445, 200141, 200186, 201437, 201463`.
- `201435` 구간 primitives: `[201436, 201435, 200099, 201438]`.
- 제외된 lanelet이 preferred이면 잘못된 route를 발행하지 않고 계획을 실패시킨다.
- Route requester의 기존 선호 lanelet 순서와 v73 SHA는 유지한다.

## 원인 조사

`200141`은 지도에 존재한다. Lanelet2 graph의 previous는 `200186`, following은 비어 있다. 다음 구간으로 보이는 `201445`와 왼쪽 접점 node `131855`는 동일하지만, 오른쪽 끝점 `45582`와 다음 시작점 `131854`는 약 44.8619 m 떨어져 있다. 따라서 단순히 next ID를 추가할 문제가 아니라 경계 형상/분할을 재검토해야 하는 지도 결함이다. 이번에는 요청에 따라 route에서 제외하고 이 도로 형상은 변경하지 않았다.

`201438`의 previous는 `201456`, following은 `200128`이다. 좌우 LC edge는 없어서 이웃 lanelet 자동 수집으로 포함되지 않는다. 합류 접근 차선으로 route에 명시적으로 포함했다. 이 추가가 옆 차선으로의 LC 허용을 의미하지는 않는다.

`200148` 오른쪽 경계는 v60/v72에서 solid였으나 v73 제작에서 dashed로 변경됐다. 사용자의 최신 정책에 따라 해당 차선과 지정한 상류/하류 차선을 route에서 제외했다. 이번 작업은 OSM 경계 태그를 복구하는 변경은 아니다.

## 실제 플러그인 검증

별도 ROS master에서 설치된 DefaultPlanner 플러그인과 실제 v73 map loader로 검사했다.

- Route A: 45 segments, 제외 차선 0개, 201438 포함, 기존 waypoint preferred 순서 일치.
- Route B: trigger인 200178부터 55 segments, 제외 차선 0개, 201438 포함, 기존 waypoint preferred 순서 일치.
- fallback: requester가 구성하는 27개 singleton segment에 제외 차선 없음.
- 통합 launch/config 계약 검사 4개 통과.
- Mission Planner C++ 패키지 재빌드 완료.
- planning_addons/install_aw149 양쪽 integration 설정 설치 완료 및 소스 동일성 확인.
- 런타임 버전 검사: 97 references v73 및 SHA 일치.

증거 로그: `/tmp/v73-route-policy-oracle.log`.
재현 입력: `/tmp/v73-route-policy-test.yaml`.
실행 스크립트: `/tmp/run-v73-route-policy-test.py`.

## 적용

실행 중인 planning을 종료하고 다시 시작한 뒤 Route A를 새로 요청해야 기존 route 메시지가 교체된다. 지도 파일은 동일하므로 이번 route 정책 변경을 위해 prediction/localization을 재시작할 필요는 없다.
