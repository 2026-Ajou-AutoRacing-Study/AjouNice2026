# K-City v73 고주로 차선 변경 topology 수정

v73은 v72의 속도·교차로·정지선·회전교차로 정보를 유지하면서, 150 km/h 고주로에서 물리적으로 평행한 내부 차선 사이를 모두 양방향으로 변경할 수 있도록 Lanelet2 공유 경계를 정리한다.

## 원인과 수정

v71에서 톨게이트 평행 구간 `200148/200127/200130/200134`와 뒤의 합류 구간 `201440/201439/201441/201446`을 각각 하나의 lanelet으로 병합했다. 병합 전 경계가 v61부터 서로 다른 virtual way로 분리돼 있었기 때문에, v72의 `200127/200130/200134`는 route segment에서는 대안 차선이지만 routing graph에서는 서로 이동할 수 없었다.

v73은 마지막 정상 공유 경계가 남아 있는 v60 형상을 기준으로 평행 구간과 합류 구간을 다시 분리했다. 평행 구간에서는 `200127 ↔ 200130 ↔ 200134`가 canonical dashed boundary를 공유한다. 합류 이후에는 `200130 → 201441 → 200360`처럼 종방향 successor 관계를 사용한다.

추가로 다음 150 km/h 내부 실선 경계를 점선으로 변경했다.

- `200186 ↔ 200138`
- `200141 ↔ 201444`
- `200148 ↔ 200127`
- `200147 ↔ 200185`
- `201400 ↔ 201399`

## 결과

| 항목 | 결과 |
| --- | ---: |
| 전체 lanelet | 1,299 |
| road lanelet | 1,210 |
| 150 km/h lanelet | 36 |
| 검증한 평행 LC 경계 | 17쌍 |
| Lanelet2 양방향 graph 검사 | 34건 통과 |
| 합류 종방향 chain | 4건 통과 |
| 평행 LC lanelet 사이 최대 면적 겹침 | 0 m² |

150 km/h lanelet이 v72의 32개에서 36개로 증가한 것은 도로가 추가됐기 때문이 아니다. v71에서 삭제했던 합류 lanelet 4개를 복원해 LC 가능 구간과 합류 구간을 분리했기 때문이다.

속도 분포는 `30:14`, `45:34`, `59:1126`, `150:36`이다.

## Route Requester

Route A/B의 고주로 선호 경로는 다음 순서를 사용한다.

```text
201457 → 201435 → 200137 → 200187 → 200130 → 201441 → 200360
```

Mission Planner의 `200130` route segment 후보는 `[200127, 200130, 200134]`를 유지한다. 세 후보 모두 선호 `200130`까지 실제 Lanelet2 lateral chain으로 연결된다. 합류 lanelet `201439/201441/201446`은 같은 route segment 후보로 묶지 않는다.

fallback 시작점은 복원된 `201441`의 `s=9.142493 m`다. v72의 `200130 s=50.376946 m`와 같은 물리 위치를 사용한다.

## 산출물

- 맵: `lanelet_map/lanelet2_map_v73.osm`
- SHA-256: `0efa7cb3dfd249d5eb78c2e2552590037eb3b8725c58ebd433656e2d53a34610`
- 생성기: `scripts/kcity_map_workflow/build_v73_highway_lane_change.py`
- 검증기: `scripts/kcity_map_workflow/validate_v73_highway_lane_change.py`
- Route A: 45 lanelet
- Route B: 55 lanelet
- fallback: 27 lanelet

소스와 install space의 실행용 맵 참조는 모두 v73으로 통일했다. v72 설정은 실행 경로에서 제거하고 `docs/archive`에 보존한다.
