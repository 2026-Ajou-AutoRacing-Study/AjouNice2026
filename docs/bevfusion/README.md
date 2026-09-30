# BEVFusion 문서 index

마지막 정리: 2026-09-30.

## CURRENT

| 목적 | 문서 |
|---|---|
| Production E25 실행·판단팀 전달 | [`BASE3_BEVFUSION_PRODUCTION_HANDOFF.md`](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md) |
| Production E25 rosbag Tracking/Prediction RViz 재생 | [`E25_ROSBAG_RVIZ_RUNBOOK.md`](E25_ROSBAG_RVIZ_RUNBOOK.md) |
| 팀 공용 Docker 학습 | [`TEAM_DOCKER_TRAINING_HANDOFF.md`](TEAM_DOCKER_TRAINING_HANDOFF.md) |
| Base4 temporal 팀원 인계·재현 | [`BASE4_TEMPORAL_TEAM_HANDOFF.md`](BASE4_TEMPORAL_TEAM_HANDOFF.md) |
| Base3-Online dataset 현재 상태 | [`../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`](../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md) |
| 다음 velocity/temporal 통제 실험과 architecture ladder | [`260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`](260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md) |
| Base4 24-scene 수집·9D/3-sweep·학습 smoke 결과 | [`260921_BASE4_TEMPORAL_P24_PIPELINE_SMOKE.md`](260921_BASE4_TEMPORAL_P24_PIPELINE_SMOKE.md) |
| Base4 V1/T1 E4 고주로 detection·tracking·prediction 판정 | [`260921_BASE4_V1_T1_E4_HIGHWAY_EVALUATION.md`](260921_BASE4_V1_T1_E4_HIGHWAY_EVALUATION.md) |
| Base4 velocity head learnability·LR 진단 | [`260922_BASE4_VELOCITY_LEARNABILITY_AND_LR_ABLATION.md`](260922_BASE4_VELOCITY_LEARNABILITY_AND_LR_ABLATION.md) |
| Base4 center-motion/association 오차 분해 | [`260923_BASE4_CENTER_MOTION_PROBE.md`](260923_BASE4_CENTER_MOTION_PROBE.md) |
| Base4 T2 sweep-separated temporal BEV 구현·smoke | [`260922_BASE4_T2_TEMPORAL_BEV_P0.md`](260922_BASE4_T2_TEMPORAL_BEV_P0.md) |

Base4 수집·학습·Q1 실험 코드는 최신 production 기준 위의
`perception/base4-temporal-competition-v2`에 복구했다. 모든 temporal/velocity 옵션은
기본 OFF이며, 이 복구가 production E25를 교체한다는 뜻은 아니다.

Production 표준은 `B3-BF0-F40 E25`, class threshold는 `0.40/0.15/0.10`이다.
Offline 최고 epoch만으로 production을 바꾸지 않고 detection, tracking, Map-CV를 같은
mission rosbag에서 비교한다.

## EVIDENCE

- [`260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md`](260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md)
  - Base3-Online Vehicle-only E25 fine-tuning의 7-bag 및 tracking/Map-CV 결과.
  - Vehicle detection·center·2D AVE는 개선됐지만 Obstacle 망각 때문에 E25 유지.
- [`260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md`](260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md)
  - Tracker motion model/noise, 올바른 2D AVE 계약, GT-velocity oracle 근거.
- [`260921_ROSBAG_ASSET_CATALOG_GT_REEVALUATION.md`](260921_ROSBAG_ASSET_CATALOG_GT_REEVALUATION.md)
  - 학습 변환기와 rosbag 평가기의 asset별 BBox center/size 계약 통합 및 기존 고주로 A/B.
- [`260918_base3_e40_production_evaluation_results.md`](260918_base3_e40_production_evaluation_results.md)
  - E25를 production으로 선택한 offline/rosbag/tracking/Map-CV 근거.
- [`260924_STATIC_OBSTACLE_TRACKING_DIAGNOSIS.md`](260924_STATIC_OBSTACLE_TRACKING_DIAGNOSIS.md)
  - Rotated BEV NMS와 tracker lifecycle/duplicate-birth 개선의 saved-bag 근거.
## HISTORICAL

- [`legacy/BASE3_BF1_ONLINE_DOMAIN_FINETUNE.md`](legacy/BASE3_BF1_ONLINE_DOMAIN_FINETUNE.md): 이전 BF1 image-domain augmentation 실험.
- [`legacy/260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md`](legacy/260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md): 완료된 fresh/E25-init 팀 학습 절차.
- [`legacy/260918_base3_e40_post_training_checklist.md`](legacy/260918_base3_e40_post_training_checklist.md): 완료된 E40 사후평가 절차.
- `legacy/260720_morai_four_camera_zero_shot_*`: 초기 zero-shot feasibility 기록.

HISTORICAL 문서의 checkpoint, image tag, dataset root, threshold를 현재 기본값으로
사용하지 않는다.
