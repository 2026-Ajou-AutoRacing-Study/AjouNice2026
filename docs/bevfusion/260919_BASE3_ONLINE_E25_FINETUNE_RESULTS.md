# Base3-Online E25 fine-tuning 결과

> **문서 상태: EVIDENCE.** 고주로 Vehicle-only rosbag-derived dataset으로 production
> E25를 fine-tune한 첫 통제 실험 결과다. 현재 production은 계속 E25다.

## 실험

- Recipe: `online_highway_e25_finetune_e12.yaml`
- 초기값: production `B3-BF0-F40 E25`
- Dataset: `morai_base3_online_highway_motion_v2`
- Train/val/test: `692 / 147 / 144` frame, Vehicle-only
- Epoch/LR: `12 / 2e-5`
- GT sampling: 없음
- Runtime: 약 22분, RTX 5060 Ti
- W&B: <https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-online-highway-e25-finetune-e12-rtx5060ti-260919-170430>

학습은 E12까지 오류 없이 완료됐다. Validation Vehicle mean AP는 E5~E12에서
`0.9775~0.9777`로 포화됐다. 자동 center-mAP best는 E7이며, E12는 mATE/mAOE가
각각 `0.1471 m / 0.0286 rad`로 가장 낮았다.

## Production 7-bag detection

class threshold는 Vehicle/Pedestrian/Obstacle `0.40/0.15/0.10`으로 고정했다.

| Rosbag / target | E25 | FT-E7 | FT-E12 |
|---|---:|---:|---:|
| 신로터리 / Vehicle | 0.7565 | 0.7906 | **0.7944** |
| 고주로 합류 / Vehicle | 0.5700 | **0.6066** | 0.6002 |
| 고주로1 / Vehicle | 0.7716 | **0.8308** | 0.8306 |
| 횡단 / Pedestrian | 0.9999 | **1.0000** | 0.9888 |
| 거리별 / Pedestrian | 0.9889 | 0.9889 | 0.9889 |
| 왕복 / Obstacle | **0.6002** | 0.5506 | 0.5378 |
| multi-asset / Obstacle | **0.6472** | 0.6212 | 0.6208 |
| **7-bag macro** | 0.7620 | **0.7698** | 0.7659 |

Class macro는 다음과 같다.

| 모델 | Vehicle 3-bag | Pedestrian 2-bag | Obstacle 2-bag |
|---|---:|---:|---:|
| E25 | 0.6994 | 0.9944 | **0.6237** |
| **FT-E7** | **0.7427** | **0.9944** | 0.5859 |
| FT-E12 | 0.7417 | 0.9889 | 0.5793 |

Vehicle domain adaptation은 세 bag 모두 개선됐지만 Vehicle-only fine-tuning으로
Obstacle 망각이 발생했다. E12는 E7보다 Vehicle 이득이 커지지 않은 채 Pedestrian과
Obstacle이 더 나빠져 후보에서 제외한다.

## FT-E7 tracking과 Map-CV

동일한 세 Vehicle detection bag을 production EKF tracker와 Map-CV에 재생했다.

| 모델 | Tracking F1 ↑ | Center p50 m ↓ | Speed MAE m/s ↓ | AVE m/s ↓ | FDE@1s m ↓ | FDE@2s m ↓ | FDE@3s m ↓ | 3s coverage ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| **E25** | **0.9391** | 0.5457 | **0.5683** | **0.7252** | **1.0849** | **2.4427** | **4.4912** | **0.9103** |
| FT-E7 | 0.9320 | **0.4373** | 0.5882 | 0.8843 | 1.1248 | 2.5876 | 4.7680 | 0.9016 |

CV-stable FDE@1/2/3초도 E25 `0.846/1.503/2.561 m` 대비 FT-E7
`0.861/1.662/2.872 m`로 악화됐다. FT-E7은 center p50을 `10.8 cm` 개선했지만
tracker velocity와 미래경로 품질을 손상했다.

## 결론

1. production은 `B3-BF0-F40 E25`를 유지한다.
2. FT-E7은 online-domain Vehicle geometry 개선의 EVIDENCE checkpoint로 보존한다.
3. E12 연장은 채택하지 않는다.
4. 다음 실험은 Vehicle-only fine-tuning 반복이 아니라 Base3 general-domain rehearsal을
   섞거나, 더 작은 LR/짧은 epoch로 Obstacle 망각과 tracker velocity 악화를 제한한다.
5. 모델 선택은 online val AP가 아니라 7-bag과 tracking/Map-CV까지 유지한다.

## Artifact

- Run: `artifacts/detection3d/bevfusion/team_training/sw/bf-sw-online-highway-e25-finetune-e12-rtx5060ti-260919-170430/`
- E7 SHA-256: `3302a775ec69d8269f0932000d17aa4e23c54c5795d32054f82c208d77a98970`
- E12 SHA-256: `2ac39a7b638c1c5a75a02aca64ea89c1836be8f56b447fe5204c0cfcb4a2f517`
- E7 detection: `artifacts/evaluation/bevfusion/online_highway_e25_ft_e7_rosbag7_260919/`
- E12 detection: `artifacts/evaluation/bevfusion/online_highway_e25_ft_e12_rosbag7_260919/`
- E7 downstream: `artifacts/evaluation/bevfusion/online_highway_e25_ft_e7_tracking_mapcv_260919/`
