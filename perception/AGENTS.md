# Perception agent context

이 디렉터리의 인지 모델 작업에서는 다음 순서로 CURRENT 문서를 확인한다.

1. `../docs/ACTIVE_PRIORITIES.md`
2. `../docs/bevfusion/README.md`
3. `../dataset/morai_od/docs/README.md`

Base3-Online 작업은
`../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`와 tracked recipe가 우선한다.
Velocity/temporal 작업은
`../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`를 우선한다.
`morai_base3_online_highway_v1`은 비교 재현용이며 기본 학습 dataset은
`base3_online_highway_motion_v2`다. 날짜가 최신이라는 이유만으로 HISTORICAL 문서의
checkpoint, dataset root, threshold를 현재 기본값으로 사용하지 않는다.
