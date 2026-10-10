# Perception agent context

이 디렉터리의 인지 모델 작업에서는 다음 순서로 CURRENT 문서를 확인한다.

1. `../docs/ACTIVE_PRIORITIES.md`
2. `../docs/bevfusion/README.md`
3. `../dataset/morai_od/docs/README.md`

Base5 작업의 현재 실행 기준은
`../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md`와
`../docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md`의
2026-10-11 JPEG 입력 전환 결과다. Sunny13 5K 완료, 다른 5환경 실제 25K/본학습은 아직이다.
현재 카메라 RGB는 train/val/test/분석 모두 native Q90 JPEG다.
`bevfusion_base5_dense_512_native_jpeg90.py`와 all-JPEG portable manifest를 사용한다.
PNG는 로컬 원본·변환용으로 보존하며 mask/depth/native GT 등 무손실 보조 파일은 변환하지 않는다.
과거 train JPEG/val PNG와 PNG→online Q90 설정은 비교 이력이며 현재 기본값이 아니다.
2026-10-11 Vast 재개 후 원격 all-JPEG 전환·전수 검증·새 prepare-only 계약을 완료했다.
최신 원격 경로·검사·외부 백업의 미완료 조건은 EVIDENCE의 13절을 먼저 따른다.
Base5-only single-frame·4-class·7D이며 temporal/OD velocity는 OFF다.
과거 Base3/Base4 config나 HEAD 브랜치를 자동으로 현재 기본값으로 승격하지 않는다.

2026-10-10 사용자 정정: 여러 대화에 걸친 하나의 task가 완료됐을 때 관련 구현·검증·문서를
모아 커밋하고 현재 작업 브랜치에 일반 push를 수행한다. 매 응답이나 작은 검사마다
커밋·push하지 않는다. 다만 변경분과 미전송 커밋을 장기간 쌓아두지는 않는다.
민감 정보·ignored raw/평가 산출물은 추가하지 않으며 force push나 타 브랜치 병합은 별도 요청 없이 하지 않는다.

Base3-Online 재현 작업은
`../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`와 tracked recipe가 우선한다.
별도 요청된 Velocity/temporal 연구 작업만
`../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`를 우선한다.
`morai_base3_online_highway_v1`은 비교 재현용이며 기본 학습 dataset은
`base3_online_highway_motion_v2`다. 날짜가 최신이라는 이유만으로 HISTORICAL 문서의
checkpoint, dataset root, threshold를 현재 기본값으로 사용하지 않는다.
