# BEVFusion 문서 작업 지침

[CURRENT](CURRENT.md)가 현재 모델 선택·계약·남은 과제의 단일 기준이다.
[README](README.md)는 작업별 인덱스이며 상태와 실측을 중복 복사하지 않는다.
데이터셋 정책을 바꿀 때만 [Base5 데이터 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md)를 함께 확인한다.

- 새 결정은 CURRENT의 해당 항목을 교체하고 검증 날짜와 근거 링크를 갱신한다.
- 수치·표본·실패·실행 조건은 `evidence/`에 기록한다. 기본 검색에서 제외하고 관련 anchor의 절만 읽는다.
- 원격 명령은 원격 런북, production 명령은 production handoff에 둔다.
- 계획·구현 완료·실제 검증 완료·production 승격을 구분한다.
- 과거 문서는 `legacy/` 또는 명시적인 재현 항목으로 연결하고 현재 실행 경로에 넣지 않는다.
- 정리로 경로를 이동할 때 old→new·원문 SHA를 기록하고 링크를 함께 갱신한다. 공개 anchor는 새 기준이나 보존된 이력으로 연결한다.
- 문서는 정상적인 한국어 띄어쓰기와 ‘~다’ 문체로 작성한다.
- 한 작업이 끝나면 관련 변경·검증을 묶어 일반 commit/push하며 작은 검사마다 수행하지 않는다.
