# MORAI OD dataset 문서 index

마지막 정리: 2026-09-19.

## Base3-Online — CURRENT

다음 순서로 읽는다.

1. [`BASE3_ONLINE_CURRENT_STATE.md`](BASE3_ONLINE_CURRENT_STATE.md)
   - 현재 경로, 데이터 규모, 최종 변환 계약, 전달물, 다음 실험을 한 페이지에 정리.
2. [`base3_online_rosbag_collection_runbook.md`](base3_online_rosbag_collection_runbook.md)
   - 실제 MORAI/ROS 연결, 녹화, audit, 변환 명령.
3. [`260919_base3_online_highway_vehicle_1k_results.md`](260919_base3_online_highway_vehicle_1k_results.md)
   - 수집·정합·visibility·Docker 검증 수치가 있는 EVIDENCE.
4. [`production_online_rosbag_dataset_plan.md`](production_online_rosbag_dataset_plan.md)
   - 설계 배경과 pilot에서 본 수집까지의 결정 이력. 일부 Phase 문장은 역사적 계획임.
5. [`../../../docs/bevfusion/260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md`](../../../docs/bevfusion/260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md)
   - 팀원 PC에서 fresh/E25-init 학습하고 고주로 rosbag을 평가하는 CURRENT handoff.

### 현재 사용 경로

| 용도 | 경로/이름 | 상태 |
|---|---|---|
| 원본 bag | `rosbags/datasets/base3_online/highway_1k/` | CURRENT source |
| runtime metadata | `/home/luv/morai_cap/260919_base3_online_highway_vehicle_1k/` | CURRENT source |
| nearest-state dataset | `artifacts/datasets/morai_base3_online_highway_v1/` | EVIDENCE only |
| motion-aligned dataset | `artifacts/datasets/morai_base3_online_highway_motion_v2/` | CURRENT local |
| 팀원 해제 경로 | `$HOME/ajounice_bevfusion_artifacts/datasets/base3_online_highway_motion_v2/` | CURRENT portable |
| PKL 내부 root | `/datasets/base3_online_highway_motion_v2` | CURRENT container |

## Base3-Static — EVIDENCE / 이전 baseline

- [`base3_static_10k_final_capture_audit.md`](base3_static_10k_final_capture_audit.md)
- [`base3_10k_route_wide_static_design.md`](base3_10k_route_wide_static_design.md)
- [`base3_static_asset_catalog.md`](base3_static_asset_catalog.md)
- [`base3_dynamic_ped_addon_v1_capture_audit.md`](base3_dynamic_ped_addon_v1_capture_audit.md)

Base3-Static은 폐기된 데이터가 아니다. Production E25를 만든 general-domain baseline이다.
다만 Base3-Online 수집 명령과 metadata 계약을 찾을 때 위 문서를 먼저 읽으면 안 된다.

## HISTORICAL

다음 문서는 연구 이력 또는 이전 dataset 세대다.

- `base2_*`, `dataset_v2_*`, `temporal_sequence_*`: Base2/temporal 연구.
- `sunny13_*`, `od180_*`: 이전 정적 생성기 세대.
- `rosbag_replay_dataset_migration_plan.md`: ReplayInfo 중심의 이전 migration 계획.
- `geometry_trial_20.md`, `scenario_runner_*`: 초기 geometry/runner 검증.

이 문서들의 명령·경로·class 정책을 Base3-Online에 자동 적용하지 않는다.
