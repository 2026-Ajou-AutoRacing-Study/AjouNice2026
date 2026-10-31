# K-City v79 고주로 병목 정리 보고서

## 수정 목적

v78에서 병목 이후 차선 `201476`이 competition route의 주행 가능 lanelet으로 자동 포함되고, 병목의 서로 겹치는 merge corridor가 동시에 route marker에 표시되어 형상이 불필요하게 복잡해 보였다.

`201476`은 route YAML에 직접 적힌 lanelet이 아니었다. DefaultPlanner가 preferred lanelet `201477`과 점선 경계 `105618`을 공유하는 횡방향 이웃을 같은 route section의 alternative primitive로 확장하면서 포함했다.

## v79 변경

- mission planner 제외 목록에 `201476`을 추가했다.
- preferred `201439`의 route section override를 `[201439]`로 제한했다. 따라서 서로 겹치는 물리적 merge corridor `201441`, `201446`은 지도에는 유지하지만 같은 route section에는 방출하지 않는다.
- 병목 진입 전 실제 평행 차선은 `[201473, 201474, 201475]`로 유지해 LC 선택권을 보존했다.
- 병목 구간에 남아 있던 미참조 시각화 geometry way 9개를 제거했다: `100690`, `101993`, `104167`, `104201`, `105436`, `105476`, `105478`, `105480`, `105482`.
- 위 way 제거로 참조되지 않게 된 coordinate-only node 642개를 함께 제거했다.

실제 DefaultPlanner가 방출하는 병목 route section은 다음과 같다.

```text
[201473, 201474, 201475]
→ [201439]
→ [200360]
→ [201477]
```

`201476`은 어느 route section에도 포함되지 않는다.

## 검증 결과

- 지도 SHA-256: `0af4ae07da4ee569583591549da73abdce6b585abe1d917eaf4bc7b944f1b7da`
- 전체 lanelet: 1,313개
- 고주로 대상 lanelet: 39개, 사각형 39개
- invalid polygon: 0개
- v78 대비 routing graph 변화: 0개
- 양방향 LC 연결: 27쌍
- 잘못된 dashed shared boundary: 0개
- 제거된 way의 regulatory/lanelet 참조: 0개
- speed rule 위반: 0개
- 실제 설치된 DefaultPlanner oracle: PASS, `201476` 미방출 확인
- competition requester 통합 테스트: 18개 PASS
- source/install runtime map 참조 검사: 97개 모두 v79, SHA 일치
- `base_ws/install_planning_addons` 및 `base_ws/install_aw149` 재빌드 완료

겹치는 merge lanelet 자체는 여러 차선이 하나로 합류하는 물리적 후보 경로이므로 삭제하지 않았다. v79는 LC topology와 종방향 연결을 유지하면서 route marker에 동시에 방출되는 후보만 실제 운용 경로로 제한했다.
