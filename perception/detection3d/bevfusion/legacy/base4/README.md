# Base4 전용 도구 — 재현 전용

현재 production·Base5 학습·후보 replay에서 import하지 않는 Base4 audit·checkpoint adaptation·velocity 도구를 격리했다.
`tools/`의 단위 테스트와 해당 모듈은 함께 이동했으며 모든 파일의 원문 bytes를 보존했다.
옛 `__file__`·relative path·import·shell repo-root는 당시 tree 기준이다. 새 위치의 직접 실행을 보장하지 않는다.
재현은 ignored cleanup manifest의 `source_commit`을 별도 worktree로 복원한 뒤 수행한다.
ROS/MORAI 실행·데이터·설치본은 이 정리에서 변경하지 않았다.

모델 설정은 `../../configs/legacy/base4/`, 당시 절차는 [문서 이력](../../../../../docs/bevfusion/legacy/research_2609_2610/BASE4_TEMPORAL_TEAM_HANDOFF.md)을 명시적으로 선택한다.
