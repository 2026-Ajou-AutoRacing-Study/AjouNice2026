# Base4 실험 설정 — 재현 전용

Base4 temporal/velocity/query-memory의 완료·기각 config와 recipe를 보존한다.
현재 Base5·E12 production·E24/E25 복귀 config가 아니다.
기존 공용 plugin은 Base5와 함께 import되는 실제 의존성이므로 원래 경로에 유지한다.

파일은 원래 bytes를 유지한다. `_base_`·recipe의 repo-relative model config와 `__file__`·import는 당시 tree 기준이다.
이 보관 위치의 직접 실행을 보장하지 않는다. 재현하려면 ignored cleanup manifest의 `source_commit`을 별도 worktree로 복원하고 당시 데이터·source·checkpoint 계약을 선택한다.
근거는 [Base4 문서 이력](../../../../../../docs/bevfusion/legacy/research_2609_2610/BASE4_TEMPORAL_TEAM_HANDOFF.md)에 있다.
