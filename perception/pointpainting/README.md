# MORAI PointPainting — historical implementation

> **상태: LEGACY/ROLLBACK.** 현재 production detector는 BEVFusion
> `B3-BF0-F40 E25`다. PointPainting을 새 기본값이나 Base4 설계 출발점으로 사용하지
> 않는다.

이 디렉터리의 코드, config와 release 기록은 과거 실험 재현 및 rollback을 위해
보존한다. 활성 perception 문서는 [`../../docs/bevfusion/README.md`](../../docs/bevfusion/README.md),
다음 velocity/temporal 계획은
[`../../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`](../../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md)를
따른다.

- 과거 연구 문서: [`docs/legacy/README.md`](docs/legacy/README.md)
- release 기록: [`releases/`](releases/)
- runtime 구현: `base_ws/src/morai_pointpainting_ros/`

과거 9-feature PointPainting의 feature 순서는
`[x,y,z,intensity,background,pedestrian,vehicle,obstacle,unknown]`이었다. 이 계약을
현재 BEVFusion input 또는 Base4 velocity schema에 적용하지 않는다.
