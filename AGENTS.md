# AjouNice2026 agent context routing

이 파일은 저장소 전체에 적용되는 문서 탐색 규칙이다. 세부 구현 계약을 여기에 복제하지
않고 canonical 문서로 연결한다.

## 작업 시작 시

1. `git status --short`와 현재 branch를 확인한다.
2. `docs/ACTIVE_PRIORITIES.md`를 읽는다.
3. `docs/README.md`에서 해당 도메인의 CURRENT index로 이동한다.
4. 날짜가 최신이라는 이유만으로 단독 문서를 authoritative하게 취급하지 않는다.

문서 충돌 시 코드/tracked recipe/launch default, ACTIVE_PRIORITIES, 도메인 CURRENT
문서, EVIDENCE, HISTORICAL 순서로 신뢰한다.

## Base3-Online / BEVFusion 작업

먼저 다음을 읽는다.

- `dataset/morai_od/docs/README.md`
- `dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`
- `docs/bevfusion/README.md`

현재 학습용 online dataset은 `base3_online_highway_motion_v2`다.
`morai_base3_online_highway_v1`은 nearest-state 비교 재현용이므로 별도 지시 없이
학습 기본값으로 사용하지 않는다.

Base3-Online 팀 학습은
`docs/bevfusion/260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md`와 tracked recipe를 따른다.
로컬 `artifacts/`, `/home/luv/morai_cap`, raw `.bag`, `.env.bevfusion.local`은 Git 전달물이
아니다. 경로 존재 여부와 portable/container mount를 각각 확인한다.

## 문서 갱신 규칙

- 실행 기준은 CURRENT, 수치 근거는 EVIDENCE, 미완료는 PLAN, 과거 구현은
  HISTORICAL/LEGACY로 표시한다.
- 새 current-state를 만들었으면 `docs/README.md`와 해당 도메인 `README.md`에서
  연결한다.
- 과거 결과를 지우거나 최신 내용처럼 덮어쓰지 않는다. 상단 상태 배너와 최신 문서
  링크를 추가한다.
- 경로, dataset 규모, checkpoint 선택이 바뀌면 current-state와 handoff를 함께
  갱신한다.
