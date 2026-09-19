# AjouNice2026 문서 진입점

마지막 정리: 2026-09-19.

이 파일은 사람과 AI agent가 오래된 실험 문서를 현재 production 계약으로 오인하지
않도록 만드는 최상위 문서 router다. 파일명이 날짜상 최신이라는 이유만으로 그 문서를
실행 기준으로 삼지 않는다.

## 문서 상태 표기

- **CURRENT**: 현재 실행·학습·배포에 사용한다.
- **EVIDENCE**: CURRENT 결론의 근거가 되는 완료 결과다.
- **PLAN**: 아직 수행하지 않은 계획이다.
- **HISTORICAL**: 재현·원인 분석용이다. 명령, 경로, 기본값을 그대로 복사하지 않는다.
- **LEGACY**: 현재 stack에서 제외됐다.

문서와 코드가 충돌하면 다음 순서로 판단한다.

1. 현재 branch의 코드, tracked recipe, launch default.
2. `docs/ACTIVE_PRIORITIES.md`와 도메인별 `README.md`가 가리키는 CURRENT 문서.
3. 최신 current-state/runbook.
4. 날짜가 붙은 EVIDENCE 문서.
5. HISTORICAL/LEGACY 문서.

## agent 권장 읽기 순서

모든 작업에서 저장소 전체 문서를 무작정 읽지 않는다. 먼저 아래 세 파일을 읽고 필요한
도메인으로만 내려간다.

1. [`ACTIVE_PRIORITIES.md`](ACTIVE_PRIORITIES.md)
2. 이 파일
3. 작업 도메인의 index

| 작업 | 먼저 읽을 문서 |
|---|---|
| Base3-Online rosbag 수집·변환 | [`dataset/morai_od/docs/README.md`](../dataset/morai_od/docs/README.md) |
| BEVFusion 학습·추론·평가 | [`bevfusion/README.md`](bevfusion/README.md) |
| Localization | [`localization/README.md`](localization/README.md) |
| MORAI full stack 실행 | [`runbooks/morai_full_stack_by_terminal.md`](runbooks/morai_full_stack_by_terminal.md) |
| Planning integration | [`planning/INTEGRATION-HANDOFF.md`](planning/INTEGRATION-HANDOFF.md) |
| 대회 현장 점검 | [`competition/260915_field_inspection_checklist.md`](competition/260915_field_inspection_checklist.md) |

## 현재 인지 baseline 요약

- Production detector: `B3-BF0-F40 E25` BEVFusion.
- Production threshold: Vehicle/Pedestrian/Obstacle `0.40/0.15/0.10`.
- Base3 offline dataset은 기존 general-domain baseline이다.
- 최신 domain experiment는 production 입력 경로로 취득한 Base3-Online 고주로
  Vehicle corpus다.
- 최신 Base3-Online canonical state:
  [`BASE3_ONLINE_CURRENT_STATE.md`](../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md).

## 오래된 문서를 다룰 때

- `260720_*zero_shot*`, PointPainting, Fast-BEV, BF1 문서는 현재 BEVFusion E25의
  기본 실행법이 아니다.
- 문서 안에 `/home/luv/...`가 있어도 portable/team path 계약보다 우선하지 않는다.
- `morai_base3_online_highway_v1`은 nearest-state 비교본이다. 현재 학습 전달본은
  `base3_online_highway_motion_v2`다.
- 과거 문서를 수정해 역사를 지우기보다는 도메인 index에서 상태를 명시하고 CURRENT
  문서로 연결한다.
