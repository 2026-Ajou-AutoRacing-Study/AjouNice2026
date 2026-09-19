# BEVFusion 문서 index

마지막 정리: 2026-09-19.

## CURRENT

| 목적 | 문서 |
|---|---|
| Production E25 실행·판단팀 전달 | [`BASE3_BEVFUSION_PRODUCTION_HANDOFF.md`](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md) |
| 팀 공용 Docker 학습 | [`TEAM_DOCKER_TRAINING_HANDOFF.md`](TEAM_DOCKER_TRAINING_HANDOFF.md) |
| Base3-Online fresh/E25-init 학습 | [`260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md`](260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md) |
| Base3-Online dataset 현재 상태 | [`../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`](../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md) |

Production 표준은 `B3-BF0-F40 E25`, class threshold는 `0.40/0.15/0.10`이다.
Offline 최고 epoch만으로 production을 바꾸지 않고 detection, tracking, Map-CV를 같은
mission rosbag에서 비교한다.

## EVIDENCE

- [`260918_base3_e40_production_evaluation_results.md`](260918_base3_e40_production_evaluation_results.md)
  - E25를 production으로 선택한 offline/rosbag/tracking/Map-CV 근거.
- [`260918_base3_e40_post_training_checklist.md`](260918_base3_e40_post_training_checklist.md)
  - corrected validation과 checkpoint sweep 절차.

## HISTORICAL

- `BASE3_BF1_ONLINE_DOMAIN_FINETUNE.md`: 이전 BF1 image-domain augmentation 실험.
- `260720_morai_four_camera_zero_shot_*`: 초기 zero-shot feasibility 기록.

HISTORICAL 문서의 checkpoint, image tag, dataset root, threshold를 현재 기본값으로
사용하지 않는다.
