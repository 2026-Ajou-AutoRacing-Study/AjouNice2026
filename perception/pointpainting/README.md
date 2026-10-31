# MORAI PointPainting — historical implementation

> **상태: LEGACY/ROLLBACK.** 현재 production detector는 BEVFusion
> 전체 30,750장 E12다(E24/E25는 명시적 복귀). PointPainting을 새 기본값이나 Base4 설계 출발점으로 사용하지
> 않는다.

이 디렉터리의 코드, config와 release 기록은 과거 실험 재현 및 호환을 위해
보존한다. 현재 작업은 [인지 작업 안내](../../docs/perception/README.md)에서 선택한다.
명시적인 Base4 재현 작업에서만 [당시 velocity/temporal 계획](../../docs/bevfusion/legacy/research_2609_2610/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md)을 읽는다.
그 계획을 현재 Base5의 다음 작업으로 자동 선택하지 않는다.

- 과거 연구 문서: [`docs/legacy/README.md`](docs/legacy/README.md)
- release 기록: [`releases/`](releases/)
- runtime 구현: `base_ws/src/morai_pointpainting_ros/`

과거 9-feature PointPainting의 feature 순서는
`[x,y,z,intensity,background,pedestrian,vehicle,obstacle,unknown]`이었다. 이 계약을
현재 BEVFusion input 또는 Base4 velocity schema에 적용하지 않는다.
