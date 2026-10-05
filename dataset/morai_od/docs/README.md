# MORAI OD dataset 문서 index

마지막 정리: 2026-10-05.

## Base5 근접·원거리 보강 — CURRENT PILOT

- [독립 Base5 10K 상세 설계 — PLAN](BASE5_INDEPENDENT_10K_DESIGN.md): Base3/E25와 별도 corpus, 범용60/미션35/배경5, 고주로·로터리·회피·동작 외관 세부 quota. 기존 v1 구현과 구분한다.

- [G80 배치·복합객체 GT 후속 확인](261005_g80_and_composite_gt_followup.md): G80 수동actor/다른Ego·위치·배치필드9-case 재로드 및 사람/개/자전거/유모차/아기 component 확인.

- [새 MR 카탈로그 연결·12-scene 실제 검증](261005_base5_mr_asset_review_smoke.md): static Vehicle 연결, G80 지면 실패/보류, 복합 객체 동일UID 다중GT 발견. 본수집 전 남은 gate.

- [Base5 MR 동기 캡처 Network](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md): 목적별6-interface JSON 및 MR 설치 경로. 정적 검증과12-scene live 캡처 통과.

- [MR 새 자산 목록 검수·포함/제외 합의](MR_ASSET_COLLECTION_CURRENT.md): 새43/33/70종과 Base5 pool 연결 완료. 동물/아기/복합 객체의3-class 학습 정책은 합의 완료·변환 구현 대기. 표지판은 도로변 전용.

- [Base5-Static 생성기·10k 설계](BASE5_STATIC_CURRENT.md): SaveSensorData single-frame용 bounded generator. 자산 review live 완료, 전체 배치/본 수집은 미실행. 최대64 scene/batch, 대량 사전 생성 금지.

- [MR 복구 객체 rosbag 보강·학습 결과](261004_base5_restored_replay_results.md): 20 scene/60 frame, WoodBox6장 camera review, 727/124 replay E2 완료. Offline best E1, production 미변경.
- [Base3 분포 공백·완료 학습 분석·다음 실행](../../../docs/bevfusion/261005_BASE3_GAPS_AND_BASE5_PILOT_RESULTS.md): 초근접·원거리 허점과 후보 rosbag A/B 계획.
- [복구 객체 GT 보정 결과·재연결](261004_mr_restored_asset_gt_calibration_runbook.md): MR20 scene/40 capture 완료, 복구4종 reference 확보, CargoBox 역사적 size 차이와 UDP 재연결 필요사항.
- [MR 전용 asset 수집 기준](MR_ASSET_COLLECTION_CURRENT.md): WoodBox·MRP 제외 자산 복구, MR 원본 template/default, GT calibration 미완료 항목.
- [`261004_base5_diversity_and_replay_pilot.md`](261004_base5_diversity_and_replay_pilot.md)
  - 후속 43/48 scene, 거리 margin 검증, 명시적 camera-sparse review 정책, 688/115 replay 학습 준비.
- [`261004_base5_reinforcement_smoke_runbook.md`](261004_base5_reinforcement_smoke_runbook.md)
  - MR 24개 정지 + 2개 저속 통과 수집, 84-frame 변환, sparse/거리 경계 품질 gate와 실행·모니터 명령.
  - 본 학습 dataset이나 production 교체 결과는 아니다.

## Base4 rosbag capture — CURRENT

2026-09-30 live 복구 검증 및 본 수집 보류 사유:
[`260930_base4_competition_v2_live_validation.md`](260930_base4_competition_v2_live_validation.md).

1. [`BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md`](BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md)
   - MR/MRP minimal hybrid 계약, 녹화 topic, 실행·audit·임시 변환, 다음 temporal gate.
2. [`morai_mr_mrp_hybrid_scenario_capture_runbook.md`](morai_mr_mrp_hybrid_scenario_capture_runbook.md)
   - launcher별 전체 터미널 순서와 live 검증 근거.
3. [`../../../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`](../../../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md)
   - 고주로 Vehicle velocity head/causal 3-sweep 수집·학습·평가 PLAN.
4. [`260921_mr_heading_and_sync_annotation_calibration.md`](260921_mr_heading_and_sync_annotation_calibration.md)
   - MR `/Ego_topic.heading`의 `-1.347 deg` 보정과 동기 annotation 교차검증 EVIDENCE.
5. [`260922_base4_velocity_diagnostic_pilot_runbook.md`](260922_base4_velocity_diagnostic_pilot_runbook.md)
   - 속도 head 원인 분리를 위한 28-scene MR 진단 수집의 준비 경로와 터미널별 실행 명령.
6. [`260922_base4_velocity_hardcase_v2_results.md`](260922_base4_velocity_hardcase_v2_results.md)
   - 속도 진단 + 병목 + 일반 고속 횡 offset을 합친 51-scene V1/T1 학습 준비 결과.
7. [`260922_base4_highway_first_experiment_dataset.md`](260922_base4_highway_first_experiment_dataset.md)
   - 159-scene raw와 157-scene Ego-deskew twin으로 완성한 고주로 1차 실험 dataset.

## Base3-Online — CURRENT BASELINE / EVIDENCE

다음 순서로 읽는다.

1. [`BASE3_ONLINE_CURRENT_STATE.md`](BASE3_ONLINE_CURRENT_STATE.md)
   - 현재 경로, 데이터 규모, 최종 변환 계약, 전달물, 다음 실험을 한 페이지에 정리.
2. [`base3_online_rosbag_collection_runbook.md`](base3_online_rosbag_collection_runbook.md)
   - 실제 MORAI/ROS 연결, 녹화, audit, 변환 명령.
3. [`260919_base3_online_highway_vehicle_1k_results.md`](260919_base3_online_highway_vehicle_1k_results.md)
   - 수집·정합·visibility·Docker 검증 수치가 있는 EVIDENCE.
4. [`260920_base3_online_highway_static_v2_collection_plan.md`](260920_base3_online_highway_static_v2_collection_plan.md)
   - Base4 전 controlled experiment용 1,000-scene 정적 Vehicle 추가 수집 PREPARED.
5. [`260920_morai_spawn_asset_and_multimodal_gt_plan.md`](260920_morai_spawn_asset_and_multimodal_gt_plan.md)
   - MR/MRP asset 호환성, 고주로 Random Spawn 분포 가설, WoodBox와 BEVFusion
     camera-visible sparse GT 후속 실험 EVIDENCE/PLAN.
6. [`260920_questions_for_morai.md`](260920_questions_for_morai.md)
   - MORAI 담당자에게 전달할 핵심 질문 다섯 가지와 multi-asset GT 정합 첨부 근거.

### 현재 사용 경로

| 용도 | 경로/이름 | 상태 |
|---|---|---|
| 원본 bag | `rosbags/datasets/base3_online/highway_1k/` | CURRENT source |
| runtime metadata | `/home/luv/morai_cap/260919_base3_online_highway_vehicle_1k/` | CURRENT source |
| nearest-state dataset | `artifacts/datasets/morai_base3_online_highway_v1/` | EVIDENCE only |
| motion-aligned dataset | `artifacts/datasets/morai_base3_online_highway_motion_v2/` | CURRENT local |
| 팀원 해제 경로 | `$HOME/ajounice_bevfusion_artifacts/datasets/base3_online_highway_motion_v2/` | CURRENT portable |
| PKL 내부 root | `/datasets/base3_online_highway_motion_v2` | CURRENT container |

## Base3-Static — LEGACY/EVIDENCE

- [`legacy/base3_static_10k_final_capture_audit.md`](legacy/base3_static_10k_final_capture_audit.md)
- [`legacy/base3_10k_route_wide_static_design.md`](legacy/base3_10k_route_wide_static_design.md)
- [`base3_static_asset_catalog.md`](base3_static_asset_catalog.md)
- [`legacy/base3_dynamic_ped_addon_v1_capture_audit.md`](legacy/base3_dynamic_ped_addon_v1_capture_audit.md)

Base3-Static은 폐기된 데이터가 아니다. Production E25를 만든 general-domain baseline이다.
완료된 설계·학습·감사 문서는 `legacy/`에 보존하며 Base3-Online 또는 Base4 수집
명령을 찾을 때 먼저 읽지 않는다. Asset catalog 두 문서는 시나리오 자산 조회를 위해
상위에 유지한다.

## HISTORICAL

다음 문서는 `legacy/`로 이동한 연구 이력 또는 이전 dataset 세대다.

- `legacy/base2_*`, `legacy/dataset_v2_*`, `legacy/temporal_sequence_*`: Base2/temporal 연구.
- `legacy/sunny13_*`, `legacy/od180_*`: 이전 정적 생성기 세대.
- `legacy/rosbag_replay_dataset_migration_plan.md`: ReplayInfo 중심의 이전 migration 계획.
- `legacy/production_online_rosbag_dataset_plan.md`: 완료된 Base3-Online 설계·pilot 계획.
- `legacy/geometry_trial_20.md`, `legacy/scenario_runner_*`: 초기 geometry/runner 검증.

이 문서들의 명령·경로·class 정책을 Base3-Online에 자동 적용하지 않는다.
