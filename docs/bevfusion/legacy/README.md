# Legacy BEVFusion documents

이 디렉터리는 현재 production 실행이나 다음 실험의 기준이 아닌 완료·기각 문서다.

- `260720_morai_four_camera_zero_shot_*`: 초기 four-camera zero-shot feasibility.
- `BASE3_BF1_ONLINE_DOMAIN_FINETUNE.md`: 채택되지 않은 BF1 image-domain augmentation.
- `260918_base3_e40_post_training_checklist.md`: 완료된 E40 평가 체크리스트.
- `260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md`: 완료된 Base3-Online 팀 학습 절차.
- `research_2609_2610/`: Base3 E40·Base4 temporal/velocity/query-memory와 10월 초 완료 연구. 당시 production·다음 계획은 현재값이 아니다.
- `context_261024/`: 정리 전 CURRENT·README 원문. 원문 SHA와 old→new는 ignored cleanup manifest에 보존한다.

현재 실행·학습·평가 계약은 상위 [`README.md`](../README.md)를 따른다. 여기의
checkpoint, image tag, dataset root와 threshold를 현재 기본값으로 사용하지 않는다.

현재 E25 복귀 handoff·Base3 baseline config·공유 plugin은 실제 의존성과 명시적 복귀를 위해 상위에 유지한다.
Base4 전용 config/recipe는 `perception/detection3d/bevfusion/configs/legacy/base4/`, 전용 도구는 `perception/detection3d/bevfusion/legacy/base4/`에 있다.
보존 Markdown은 baseline Git 원문 bytes를 유지한다. 내부 상대 링크는 manifest의 원래 경로 기준이며 현재 router에서 문서·anchor를 선택한다.
이동한 연구 코드도 원문 bytes를 유지한다. 과거 `__file__`·relative path·import는 당시 tree 기준이며, 실행 재현은 manifest의 `source_commit`을 별도 worktree로 복원한다.
