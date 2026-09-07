# PointPainting Base2 S1 E38 release.

Runtime path:

`artifacts/detection3d/pointpainting/releases/260825_base2_s1_singleframe9_geom_aug_e38/`

## Files.

| File | Bytes | SHA256 |
|---|---:|---|
| `pointpainting_base2_s1_best_epoch38.pth` | 58,653,657 | `939076e933ac537beaec7183f002b10f10af7e20e92a634da00424ae6cee669c` |
| `pointpillar_morai_pointpainting_base2_singleframe9_geom_aug.yaml` | 2,936 | `c6e4a6d45a7e42eee667f4ffb883198865c58ce2fba18908a0decf3806979536` |

## Contract.

- Single-frame 9-feature PointPainting input.
- 7D box output without detector velocity regression.
- Base2 Train 5,509 / Val 1,181 target frames.
- Train-only left-right/front-back flip, ±45 degree rotation and 0.95–1.05
  scaling.
- Checkpoint epoch is 38 and state dictionary contains 127 keys.
- LRASPP uses the shared
  `260817_sunny13_1500_9feat_80e/lraspp_4class_best_miou.pth` checkpoint.
- This is the current Vehicle production comparator. Legacy E65 remains the
  historical static-data baseline.

## Controlled benchmark.

- Base2 Val overall center mAP: `0.9417`.
- Base2 Val Vehicle AP: `0.9767`.
- New roundabout Vehicle center mAP: `0.6878`.
- Highway1 Vehicle center mAP: `0.7717`.
- Full OD, Tracking and Map-CV results are in
  `perception/pointpainting/docs/rosbag_vehicle_leaderboard.md`.
