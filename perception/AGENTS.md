# Perception agent context

이 디렉터리의 인지 모델 작업에서는 다음 순서로 CURRENT 문서를 확인한다.

1. `../docs/ACTIVE_PRIORITIES.md`
2. `../docs/bevfusion/README.md`
3. `../dataset/morai_od/docs/README.md`

Base5 작업의 현재 실행 기준은
`../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md`와
`../docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md`의
2026-10-11 JPEG 입력 전환 결과다. Sunny13 5K 완료, 다른 5환경 실제 25K/본학습은 아직이다.
2026-10-11 최신 파생 GT는 EVIDENCE 16절의 `native_rpy_fixed_yaw_enclosure_v1`이다.
현재 입력은 `../artifacts/handoff/base5_sunny13_jpeg90_upright_v1_261011_release`다.
raw·센서·클래스·GT 수·split은 유지하고 파생 infos 크기만 native 8모서리 enclosure로 보완했다.
현재 native-JPEG config는 old yaw-only manifest를 거부한다. 15절 heatmap 수정도 유지한다.
Vast/Drive는 이 작업에서 변경하지 않았다. 원격 재개 시 최신 소스·기존 JPEG의 라벨 재변환·
새 work-dir 준비를 수행하며 센서를 재전송하거나 이전 계약으로 resume하지 않는다.
현재 카메라 RGB는 train/val/test/분석 모두 native Q90 JPEG다.
`bevfusion_base5_dense_512_native_jpeg90.py`와 all-JPEG portable manifest를 사용한다.
PNG는 로컬 원본·변환용으로 보존하며 mask/depth/native GT 등 무손실 보조 파일은 변환하지 않는다.
과거 train JPEG/val PNG와 PNG→online Q90 설정은 비교 이력이며 현재 기본값이 아니다.
2026-10-11 Vast 재개 후 원격 all-JPEG 전환·전수 검증·새 prepare-only 계약을 완료했다.
최신 로컬 입력은 EVIDENCE 14절의 Base5 유효 반사점 필터·uint8 운반이다.
현재 native-JPEG config만 train/val/test와 새 추론에서 exact XYZ-zero/nonfinite를 제거한다.
이미지는 GPU 정규화 전까지 uint8이며 기존 BEV 증강 강도를 유지한다. raw·GT·E25는 유지한다.
사용자가 Vast를 다시 중단한 뒤 로컬에서 검증했으므로 13절 원격 소스·prepare-only 계약은
이 변경 이전이다. 재개 시 최신 소스 전달과 새 work-dir·짧은 재검증이 필요하다.
최신 원격 경로·외부 백업의 미완료 조건은 13절을 함께 따른다.
2026-10-11 추가 최신 학습 정합은 EVIDENCE 15절이다. Base5-only rectangular head의
학습 heatmap X/Y 교환을 수정했고 고유 11,052 GT·증강 포함 29,714회 peak 검사와 실제
전체 모델 4-iteration 역전파·저장·재개를 로컬에서 통과했다. 원본 GT·E25는 바꾸지 않는다.
이전 feature 축/유한 loss 검사가 target 생성까지 보장한 것은 아니다. Vast 반영은 아직이며
다음은 로컬 공간 격자 5/20·7.5/30·10/40cm 비용/위험 비교, 선택 격자의 depth 간격 비교다.
속도만으로 AP 우위를 확정하지 않으며 원격 재개 때 최신 소스와 새 준비 계약을 사용한다.
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
