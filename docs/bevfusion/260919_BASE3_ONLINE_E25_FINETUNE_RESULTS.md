# Base3-Online E25 fine-tuning 결과

> **문서 상태: EVIDENCE.** 고주로 Vehicle-only rosbag-derived dataset으로 production
> E25를 fine-tune한 첫 통제 실험 결과다. 주 평가는 학습 범위와 같은
> `고주로 합류 + 고주로1`이며, 7-bag은 전역 회귀 확인용 보조 평가다.

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

## 고주로 Vehicle detection — primary

class threshold는 Vehicle `0.40`으로 고정했다. 두 bag을 같은 가중치로 평균하면
E25 `0.6708`, FT-E7 `0.7187`, FT-E12 `0.7154`다. 따라서 고주로 Vehicle OD라는
실험 목표에는 FT-E7이 명확하게 성공했다.

| Rosbag | E25 | FT-E7 | FT-E12 |
|---|---:|---:|---:|
| 고주로 합류 | 0.5700 | **0.6066** | 0.6002 |
| 고주로1 | 0.7716 | **0.8308** | 0.8306 |
| **고주로 2-bag macro** | 0.6708 | **0.7187** | 0.7154 |

## Production 7-bag detection — regression guard

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

## 고주로 FT-E7 tracking과 Map-CV — primary

학습 범위와 같은 `고주로 합류 + 고주로1`만 production EKF tracker와 Map-CV에
재생해 bag별 지표를 같은 가중치로 평균했다.

2026-09-19 tracker 감사에서 기존 evaluator가 body-frame `linear.y`를 버리고
`linear.x`와 box yaw만으로 AVE를 재구성하던 오류를 수정했다. 또한 raw bag의
카메라·LiDAR 전체를 재생해 30 Hz tracker timer가 누락되던 조건을 제거하고
`/Ego_topic + detections`만 1.0x로 재생했다. 아래 표가 수정된 canonical 결과다.

| 모델 | Tracking F1 ↑ | Center p50 m ↓ | Speed MAE m/s ↓ | 2D AVE m/s ↓ | Long/Lat MAE m/s ↓ | FDE@1s m ↓ | FDE@2s m ↓ | FDE@3s m ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| E25 | 0.9324 | 0.5993 | **0.6551** | 0.9685 | **0.6546**/0.5696 | **1.2110** | **2.6471** | 4.6948 |
| **FT-E7** | **0.9330** | **0.4771** | 0.6681 | **0.8906** | 0.6592/**0.4449** | 1.2149 | 2.6561 | **4.6948** |

위 FDE는 급가감속·spawn 구간까지 포함한 Top-1 mean이다. Production 모델 선택의
주 지표인 **CV-stable minFDE-K mean**은 반대로 FT-E7이 모두 개선됐다.

| 모델 | CV-stable minFDE-K@1s ↓ | @2s ↓ | @3s ↓ |
|---|---:|---:|---:|
| **E25** | **0.8288** | **1.2611** | **1.7659** |
| FT-E7 | 0.8358 | 1.2887 | 1.7882 |

FT-E7은 center p50을 `12.2 cm`, 올바른 2D AVE를 `0.078 m/s` 개선했다. Speed MAE는
`0.013 m/s`, CV-stable 3초 minFDE-K는 `0.022 m` 나빠졌지만 전체 운동 구간 3초
Top-1 FDE는 소수 넷째 자리까지 같다. 따라서 detector geometry 이득은 실재하며,
기존에 보고한 큰 tracking motion regression은 재현되지 않는다.

## 결론

1. 고주로 전용 실험은 성공이다. FT-E7은 고주로 OD와 CV-stable Map-CV를 모두
   개선한 specialization 후보로 보존한다.
2. 다만 단일 전역 production은 `B3-BF0-F40 E25`를 유지한다. FT-E7은 Obstacle
   망각이 남아 있고, 고주로 외 일반화까지 검증한 전역 대체 모델이 아니다.
3. E12 연장은 채택하지 않는다.
4. 다음 detector 실험은 Vehicle-only fine-tuning 반복이 아니라 Base3 general-domain
   rehearsal을 섞거나, 더 작은 LR/짧은 epoch로 Obstacle 망각을 제한한다.
5. 고주로 실험의 primary 평가는 고주로 두 bag으로 제한한다. 7-bag은 전역 production
   승격 시의 regression guard로만 사용한다.
6. Tracker position measurement noise는 별도 3-bag A/B에서 `0.30 -> 0.40 m`가
   speed/2D AVE와 Map-CV FDE를 개선해 새 production 기본값으로 채택했다. 상세 근거는
   [`260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md`](260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md)에 남긴다.

## Artifact

- Run: `artifacts/detection3d/bevfusion/team_training/sw/bf-sw-online-highway-e25-finetune-e12-rtx5060ti-260919-170430/`
- E7 SHA-256: `3302a775ec69d8269f0932000d17aa4e23c54c5795d32054f82c208d77a98970`
- E12 SHA-256: `2ac39a7b638c1c5a75a02aca64ea89c1836be8f56b447fe5204c0cfcb4a2f517`
- E7 detection: `artifacts/evaluation/bevfusion/online_highway_e25_ft_e7_rosbag7_260919/`
- E12 detection: `artifacts/evaluation/bevfusion/online_highway_e25_ft_e12_rosbag7_260919/`
- E7 downstream: `artifacts/evaluation/bevfusion/online_highway_e25_ft_e7_tracking_mapcv_260919/`
