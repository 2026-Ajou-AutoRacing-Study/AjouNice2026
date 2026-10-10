# Perception agent context

이 디렉터리의 인지 모델 작업에서는 다음 순서로 CURRENT 문서를 확인한다.

1. `../docs/ACTIVE_PRIORITIES.md`
2. `../docs/bevfusion/README.md`
3. `../dataset/morai_od/docs/README.md`

Base5 작업의 현재 실행 기준은
`../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md`와
`../docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md`의
2026-10-10 준비 결과다. Sunny13 5K 완료, 다른5환경 실제25K/본학습은 아직이다.
Base5-only single-frame·4-class·7D이며 temporal/OD velocity는 OFF다.
과거 Base3/Base4 config나 HEAD 브랜치를 자동으로 현재 기본값으로 승격하지 않는다.

Base3-Online 재현 작업은
`../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`와 tracked recipe가 우선한다.
별도 요청된 Velocity/temporal 연구 작업만
`../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`를 우선한다.
`morai_base3_online_highway_v1`은 비교 재현용이며 기본 학습 dataset은
`base3_online_highway_motion_v2`다. 날짜가 최신이라는 이유만으로 HISTORICAL 문서의
checkpoint, dataset root, threshold를 현재 기본값으로 사용하지 않는다.
