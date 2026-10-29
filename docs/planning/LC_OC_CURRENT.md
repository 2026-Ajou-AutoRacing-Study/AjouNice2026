# LC 중 base_link 진입 후 ObstacleCruise 허용

`planning/lc_oc` 후보 구현. LC 승인·취소·복귀·완료 판단과 횡경로는 기존대로
유지하고, 승인된 LC 실행 중 base_link가 해당 LC의 목표 차선 polygon 내부에
들어간 경우에만 기존 ObstacleCruise의 속도계획을 허용한다.

| 상태 | Cruise |
|---|---|
| 승인된 RUNNING LC 없음 | 기존 동작 |
| 준비 중 / base_link 미진입 | 기존처럼 억제 |
| 차체 일부만 진입, base_link는 밖 또는 경계 위 | 기존처럼 억제 |
| 횡이동 실행 중, base_link가 목표 차선 내부 | 기존 Cruise 속도계획 허용 |
| Abort 복귀 중 / 목표 polygon 없음 | 기존처럼 억제 |

기존 `hasEgoEnteredTargetLane()`의 차체 일부 진입 판정은 바꾸지 않는다.
새 진입 판정은 현재 위치를 사용하며 진입 이력을 고정하지 않는다.

`/planning/lane_change/executing`은 계속 실제 LC lifecycle을 나타낸다.
BPP가 별도의 latched Bool `/planning/lane_change/cruise_suppressed`를 발행하며,
Cruise는 이 신호로만 LC 억제를 결정한다. 두 신호를 조합하지 않아 서로 다른
주기의 메시지 조합으로 억제가 풀리는 것을 피한다. 여러 승인 LC가 있다면
하나라도 억제가 필요한 동안 억제한다.

억제 전환 시 기존 Cruise handoff 초기화를 그대로 사용한다. 기존 객체 선정,
RSS, PID 및 자기 velocity limit 해제 동작을 유지하고, 목표 차선 전용 객체
필터나 회피·횡경로 변경은 추가하지 않는다.

BPP와 LC provider, Cruise를 함께 빌드·배포해야 한다. 이전 BPP는 새 억제 신호를
발행하지 않으므로 신구 바이너리를 섞어 실행하지 않는다. 후보 단위·통합 검사는
실제 MORAI 폐루프 충돌 회피 검증과 구분한다.
