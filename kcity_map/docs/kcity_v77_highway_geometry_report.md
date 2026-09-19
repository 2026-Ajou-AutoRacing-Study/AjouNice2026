# K-City v77 고주로 형상·LC 검수

- 지도: `lanelet_map/lanelet2_map_v77.osm`
- SHA-256: `3783ddd90d5c14f7cb42781efead28519fc5a8b082ebbe4671f57542ce0750be`
- 기준 지도: v76
- 목적: 직선 고주로와 병목부의 물결형·삼각형·다각형 lanelet을 단순 사각형으로 정리하면서 기존 LC와 앞뒤 topology를 보존한다.

## 형상 수정

`201468`, `201438`, `200099`, `201446`을 포함한 직선 고주로 39개 lanelet을 정리했다. 각 lanelet의 좌우 경계와 중심선은 시작·끝 단면만 남겼다. 기존 경계 끝점과 way ID는 유지했으므로 종방향 joint와 공유 경계 identity는 바뀌지 않는다.

- 대상 39개 모두 좌우 경계 2점, 중심선 2점
- 대상 39개 모두 서로 다른 꼭짓점 4개의 유효 사각형
- 최소 폭 2.949 m, 최대 폭 3.896 m
- 전체 1,313개 lanelet 중 invalid polygon 0개
- 인접 동일 위치 point 0개
- 대상 경계를 사용하는 비대상 lanelet 0개
- 한 lanelet만 사용하던 점선 `105550`, `105554`, `105569`, `105570`은 `virtual` 경계로 수정

![v76/v77 고주로 형상 비교](kcity_v77_highway_geometry_comparison.png)

## LC와 종방향 연결

v76의 전체 routing graph와 v77을 lanelet별로 비교했다.

- previous/following/left/right/adjacent-left/adjacent-right 변경 0개
- 실제 평행 차로의 공유 점선 27쌍 모두 동일 way ID를 사용
- 27쌍 모두 양방향 `left/right` routing 가능
- 대상 범위의 공유 점선 중 단독 참조 경계 0개
- 150 km/h lanelet 49개 모두 predecessor와 successor 존재

겹치는 합류 corridor `201467/201468`, `201474/201475`, `201439/201441`, `201441/201446`, `200185/200360`은 하나의 경계를 공유하는 평행 차로가 아니다. 이 구간은 각각 온전한 사각형을 유지하고 유효한 successor로 병합된다. 이 겹침 구간에 shared-boundary LC를 강제로 넣으면 lanelet 폭을 없애거나 다시 삼각형으로 만들어야 하므로 적용하지 않았다. LC는 각 corridor에 진입하기 전의 평행 공유 점선 구간에서 양방향으로 가능하다.

## Route Requester와 Mission Planner

v77 중심선에 맞춰 고주로 waypoint 좌표와 yaw를 다시 계산했다. 선호 경로는 다음 순서를 유지한다.

```text
200137 → 200187 → LC Left → 201444 → 201469 → 200127
       → 201473 → 201439 → 200360
```

실제 설치된 v77 map loader와 DefaultPlanner로 확인한 결과:

- Route A: 49 segments, PASS
- Route B: 59 segments, PASS
- `201444` segment alternatives: `201444, 200187, 200102`
- `200127` segment alternatives: `200127, 200130, 200134`
- `201473` segment alternatives: `201473, 201474, 201475`
- `201439` segment alternatives: `201439, 201441, 201446`

## 속도 규정

- road speed counts: 150 km/h 49개, 59 km/h 1,127개, 45 km/h 34개, 30 km/h 14개
- 150 km/h 시작 최소 margin: 1.0589 m
- 150 km/h 종료 최소 margin: 1.0235 m
- 규정 예외 범위 밖 60 km/h 초과 lanelet: 0개

## 빌드와 검사

- v77 정적 형상·routing validator: PASS
- Competition Route Requester policy: 13 PASS
- private ROS master Gateway transport: 18 PASS
- 런치·perception·mission contract: 6 PASS
- 설치본 DefaultPlanner oracle: Route A/B PASS
- source/install runtime map references: 97개 모두 v77, SHA 일치
- `install_planning_addons`, `install_aw149` 강제 재설치 완료
