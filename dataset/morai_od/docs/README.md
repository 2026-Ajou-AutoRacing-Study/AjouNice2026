# MORAI OD dataset 문서 index

마지막 정리: 2026-09-20.

## Base4 rosbag capture — CURRENT

1. [`BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md`](BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md)
   - MR/MRP minimal hybrid 계약, 녹화 topic, 실행·audit·임시 변환, 다음 temporal gate.
2. [`morai_mr_mrp_hybrid_scenario_capture_runbook.md`](morai_mr_mrp_hybrid_scenario_capture_runbook.md)
   - launcher별 전체 터미널 순서와 live 검증 근거.
3. [`../../../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`](../../../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md)
   - 고주로 Vehicle velocity head/causal 3-sweep 수집·학습·평가 PLAN.

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
   - MORAI 담당자에게 전달할 핵심 질문 네 가지와 WoodBox 첨부 근거.

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
