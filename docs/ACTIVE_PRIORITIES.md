# 활성 작업 우선순위

갱신: 2026-10-24. 미완료 작업과 착수 조건만 적는다. 운영값은 각 CURRENT가 권위다.
완료된 5K 수집·30,750장 학습·LiDAR 전환·이전 최적화를 다시 할 일로 쌓지 않는다.

## 현재 기준

- 인지 모델·가속·배포: [BEVFusion CURRENT](bevfusion/CURRENT.md).
- Tracking·Map-CV·후보 정책: [후단 CURRENT](../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md).
- 데이터·GT·수집 재개: [데이터 CURRENT](../dataset/morai_od/docs/BASE5_V2_CURRENT.md).
- 측위·제어 입력: [Localization CURRENT](localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md).
- 실행·공유: [공통 runbook](runbooks/morai_full_stack_by_terminal.md),
  [개발 녹화 안내](../scripts/README.md#개발용-기록).

<a id="perception-priorities"></a>

## 다음 확인

| 우선 작업 | 착수·완료 기준 |
| --- | --- |
| 팀 4090 LAN 단계별 지연 분석 | Windows MORAI + Linux 알고리즘 동시 기록 수령 후, 입력 주기·큐·전처리·GPU·발행·후단 비용과 누락을 같은 frame/clock 기준으로 분리. 기존 bag에 없는 단계는 미측정으로 남김 |
| 중복·분류 일관성 후보의 운영 판단 | 오프라인/native/ROS 대조 결과를 판단 팀과 확인. shape/class 후보 기본 OFF 유지. 새 입력과 폐루프 손익 확인 전 자동 승격하지 않음 |
| 측위 현장 재검증 | 현재 Smooth/음영 계약을 유지하고, 운영측 CP14 환경 갱신 후 해당 복귀와 실제 LAN 기록을 재검증. 세부 순서는 측위 CURRENT |
| 팀 배포 후 프로필 확인 | 현장 원본 + LAN adapter + Ego 수신 OFF + 공식 START 초기화 명시 기준. 팀 PC의 실제 NIC·로드본·설치본과 초기 pose 발행을 확인하며 새 LAN 기동 검증은 별도로 기록 |

<a id="perception-latency"></a>

## 보류·범위

- 예측 길이는 로터리·고주로 모두 현재 3초 유지. 판단 정책이 안정된 뒤 별도 비교한다.
- 전면 C++ 포팅보다 실제 측정된 병목을 먼저 개선한다. GPU ms와 header→수신 age를 혼동하지 않는다.
- 새 데이터 보강·재학습·센서 배치 변경은 기존 결과와 비용을 따로 검토한다. 이 목록 자체가
  수집·학습·원격 조작·삭제 또는 정책 변경의 실행 승인은 아니다.
- planning/control 수정은 별도 담당 범위다. 과거 계획 원문은
  [정리 전 사본](legacy/context_261024/docs/ACTIVE_PRIORITIES.md)에 보존했다.
