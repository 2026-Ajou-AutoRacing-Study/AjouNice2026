# 활성 작업 우선순위

갱신: 2026-10-28. 미완료 작업과 착수 조건만 적는다. 운영값은 각 CURRENT가 권위다.
완료된 5K 수집·30,750장 학습·LiDAR 전환·이전 최적화를 다시 할 일로 쌓지 않는다.

## 현재 기준

- 인지 모델·가속·배포: [BEVFusion CURRENT](bevfusion/CURRENT.md).
- 신호등 검출·색/화살표·미션 연결: [신호등 CURRENT](perception/traffic_light/CURRENT.md).
- Tracking·Map-CV·후보 정책: [후단 CURRENT](../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md).
- 데이터·GT·수집 재개: [데이터 CURRENT](../dataset/morai_od/docs/BASE5_V2_CURRENT.md).
- 측위·제어 입력: [Localization CURRENT](localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md).
- 실행·공유: [공통 runbook](runbooks/morai_full_stack_by_terminal.md),
  [개발 녹화 안내](../scripts/README.md#개발용-기록).

<a id="perception-priorities"></a>

## 다음 확인

| 우선 작업 | 착수·완료 기준 |
| --- | --- |
| 팀 4090 LAN 송신률 원인 확인 | [Linux 단계별 분석](bevfusion/evidence/261028_TEAM_LAN_AND_DUPLICATE_TRACK_REVIEW.md)은 완료했다. Windows MORAI의 동시 계측을 받아 송신률·네트워크 경계를 확인하며, 기존 bag에 없는 물리 발생 시각은 추정으로 채우지 않음 |
| 중복·분류·회피 후보의 운영 판단 | UNKNOWN 정지 prior는 독립 입력의 퇴행으로 기본 OFF를 유지한다. [v99 대조](bevfusion/evidence/261028_V99_WINDOW60_90_LIVE_REVIEW.md) 후속 격리 시험에서 경로 유지 비용 경쟁·seed별 예산 후보도 실패 증가로 기각했다. cap60·기존 안전 검사를 유지하며, 추가 연구는 QP 계산 맥락과 후속 경로 이력을 먼저 구분 |
| 신호등 수정 후 팀 LAN 확인 | [최신 v99 기록](bevfusion/evidence/261028_TEAM_LAN_AND_DUPLICATE_TRACK_REVIEW.md)의 A/B/C 정차·재출발 분석은 완료했다. D 미관측 음수속도 결함의 로컬 수정·설치 후 새 LAN 재주행과 이동 중 G→Y 경계 확인이 남음. 계산6/10·응답지연0은 유지하며 설정 권위는 신호등 CURRENT |
| 측위 현장 재검증 | 현재 Smooth/음영 계약을 유지하고, 운영측 CP14 환경 갱신 후 해당 복귀와 실제 LAN 기록을 재검증. 세부 순서는 측위 CURRENT |
| 팀 배포 후 프로필 확인 | 현장 원본 + LAN adapter + Ego 수신 OFF + 공식 START 초기화 명시 기준. 팀 PC의 실제 NIC·로드본·설치본과 초기 pose 발행을 확인하며 새 LAN 기동 검증은 별도로 기록 |

<a id="perception-latency"></a>

## 보류·범위

- 최종 정책은 로터리/일반 최대 3초·고주로 최대 8초이며 NPC 소멸 종단이 먼저면 그 시간만 Map-CV로 출력한다. 고주로 저속·명확한 도로 방향 복구를 포함한다. [설정·적용·한계](../base_ws/src/perception_object_pipeline/docs/HIGHWAY_EIGHT_SECOND_HANDOFF.md)를 따른다. 실제 팀 PC LC 반응과 오검출 영향 확인은 후속이다.
- 전면 C++ 포팅보다 실제 측정된 병목을 먼저 개선한다. GPU ms와 header→수신 age를 혼동하지 않는다.
- 새 데이터 보강·재학습·센서 배치 변경은 기존 결과와 비용을 따로 검토한다. 이 목록 자체가
  수집·학습·원격 조작·삭제 또는 정책 변경의 실행 승인은 아니다.
- planning/control 수정은 별도 담당 범위다. 과거 계획 원문은
  [정리 전 사본](legacy/context_261024/docs/ACTIVE_PRIORITIES.md)에 보존했다.
