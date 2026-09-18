# Base3 BEVFusion Fresh E40 corrected 평가 결과

## 결론

- Offline validation best는 `E26`이다.
- Offline runner-up은 `E25`이며 center mAP 차이는 `0.00058`이다.
- 실제 mission rosbag과 production tracker/Map-CV까지 포함한 최종 후보는
  **`B3-BF0-F40 E25`**다.
- 기존 production `B3-BF0 E10`보다 8-bag detection macro AP, Vehicle tracking,
  velocity와 1/2/3초 prediction이 모두 개선됐다.
- `E26`은 offline-best로 보존하지만 production 후보로는 `E25`보다 낮다.
- 기존 production alias와 기본 checkpoint는 운영 변경을 분리하기 위해 이
  평가 커밋에서는 바꾸지 않는다. E25 inference artifact의 Docker model-ready
  healthcheck까지 완료했다.

Corrected W&B run:
<https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-fresh-e40-range30-corrected-val>

## 평가 계약

- Base3 frozen split.
- GT와 prediction 모두 `[-30, -30, -3, 30, 30, 2] m` box-center 필터.
- Val GT: Vehicle 1,100 / Pedestrian 1,005 / Obstacle 2,185.
- Rosbag threshold: Vehicle/Pedestrian/Obstacle `0.40/0.15/0.10`.
- Detection 8-bag은 모두 frame coverage `1.0`.
- 고주로1은 `0.5x`, multi-asset obstacle은 `0.25x`, 나머지는 `1.0x`.
- Tracking/Map-CV는 Vehicle GT가 존재하는 4-bag만 평가했다.

## Corrected Val과 Test

| 모델 | Split | Center mAP | IoU mAP@0.5 |
|---|---|---:|---:|
| B3-BF0 E10 | Test | 0.974991 | 0.947590 |
| B3-BF0-F40 E25 | Val | 0.980829 | 0.956486 |
| B3-BF0-F40 E25 | Test | 0.979407 | **0.961747** |
| **B3-BF0-F40 E26** | **Val** | **0.981408** | 0.960322 |
| B3-BF0-F40 E26 | Test | **0.980080** | 0.961191 |
| PointPainting PP1 E34 | Test | 0.9809 | 0.9640 |

Test에서 E25는 E10 대비 center mAP `+0.00442`, IoU mAP@0.5
`+0.01416`이다. E26은 각각 `+0.00509`, `+0.01360`이다. PP1 수치는 같은
Base3 frozen split 참고값이며 rosbag production 우열을 대신하지 않는다.

E25 Test의 class AP는 Vehicle `0.96674`, Pedestrian `0.98878`, Obstacle
`0.98271`이다. E10 대비 Vehicle AP는 조금 낮지만 Vehicle mATE는
`0.0772 -> 0.0703 m`, Obstacle AP는 `0.96466 -> 0.98271`, Obstacle mATE는
`0.0554 -> 0.0483 m`로 개선됐다.

## Mission rosbag detection

아래 AP는 center distance `0.5/1/2/4 m` 평균이다.

| Rosbag / target class | E10 | E25 | E26 | E25-E10 |
|---|---:|---:|---:|---:|
| 신로터리 / Vehicle | **0.7725** | 0.7565 | 0.7626 | -0.0160 |
| 고주로 합류 / Vehicle | **0.5833** | 0.5700 | 0.5585 | -0.0133 |
| 고주로1 / Vehicle | 0.7667 | **0.7716** | 0.7681 | +0.0049 |
| Vehicle 30 m / Vehicle | 0.6924 | **0.8436** | 0.8292 | +0.1512 |
| Pedestrian 횡단 / Pedestrian | **1.0000** | 0.9999 | 0.9889 | -0.0001 |
| Pedestrian 거리별 / Pedestrian | **1.0000** | 0.9889 | **1.0000** | -0.0111 |
| Obstacle 왕복 / Obstacle | **0.6221** | 0.6002 | 0.6106 | -0.0219 |
| Base3 multi-asset / Obstacle | 0.6067 | 0.6472 | **0.6487** | +0.0405 |
| **8-bag macro** | 0.7555 | **0.7722** | 0.7708 | **+0.0168** |

Mission별 macro AP:

| 모델 | Vehicle 4-bag | Pedestrian 2-bag | Obstacle 2-bag |
|---|---:|---:|---:|
| E10 | 0.7037 | **1.0000** | 0.6144 |
| **E25** | **0.7354** | 0.9944 | 0.6237 |
| E26 | 0.7296 | 0.9944 | **0.6296** |

E25의 macro 이득은 Vehicle 30 m와 multi-asset에서 크다. 반면 로터리·합류
detection AP와 Obstacle 왕복은 E10보다 낮으므로 이 수치만으로 교체하지 않고
downstream 결과를 함께 사용했다.

## Vehicle tracking과 Map-CV

저장된 detection bag을 같은 production EKF, UNKNOWN stationary policy와 Map-CV에
재생했다. 아래는 Vehicle 4-bag macro다. AVE는 velocity vector L2 error다.

| 모델 | Tracking F1 ↑ | Center p50 m ↓ | Speed MAE m/s ↓ | AVE m/s ↓ | FDE@1s m ↓ | FDE@2s m ↓ | FDE@3s m ↓ | 3s coverage ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| E10 | 0.9260 | 0.4927 | 0.4816 | 0.6326 | 0.9480 | 2.0772 | 3.7685 | 0.8865 |
| **E25** | **0.9449** | **0.4808** | 0.4572 | **0.5768** | **0.9052** | **1.9726** | **3.5781** | 0.9145 |
| E26 | 0.9445 | 0.4901 | **0.4520** | 0.5888 | 0.9095 | 1.9898 | 3.6257 | **0.9154** |

CV-stable 4-bag macro FDE@1/2/3s도 E10 `0.733/1.290/2.182 m`에서 E25
`0.710/1.218/2.032 m`로 개선됐다.

핵심 구간에서 E25의 tracking center p50 / top-1 FDE@3s 변화:

| Rosbag | E10 center p50 | E25 center p50 | E10 FDE@3s | E25 FDE@3s |
|---|---:|---:|---:|---:|
| 신로터리 | 0.433 m | **0.417 m** | 4.625 m | **4.232 m** |
| 고주로 합류 | **0.717 m** | 0.750 m | 5.830 m | **5.619 m** |
| 고주로1 | 0.472 m | **0.470 m** | 3.695 m | **3.623 m** |
| Vehicle 30 m | 0.350 m | **0.286 m** | 0.923 m | **0.839 m** |

합류의 위치 p50은 `+0.034 m` 나빠졌지만 F1, AVE와 3초 FDE는 개선됐다.
판단·추월에 더 직접적인 전체 Vehicle tracking/prediction 계약에서는 E25가 가장
균형이 좋다.

## Artifact

- Full checkpoint:
  `artifacts/detection3d/bevfusion/team_training/sw/bf-sw-fresh-e40-rtx5060ti-260918-005536/epoch_25.pth`
- Full SHA-256:
  `0fc30b97517484d1b2da1caef716e358af57e4049b2e7637820a080e54b03df3`
- Inference checkpoint:
  `artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth`
- Inference SHA-256:
  `36816cf769c87d49483d002111abfad8e1afd043afd74a0b9fc54aac82133011`
- E26 full SHA-256:
  `e819eb24221e82af9a95d6cdef05a0da7a0c6f3da33e98bb970e1843d231ea0b`

Inference artifact는 team CUDA 12.8 image에서 server load와 healthcheck
`model_loaded=True`를 통과했다.

## 최종 판단

1. Offline checkpoint 기록은 E26 best, E25 runner-up으로 남긴다.
2. 실차 production 교체 후보는 E25로 정한다.
3. E10 기본값을 실제로 교체할 때는 E25로 짧은 MORAI interactive 주행과 planviz
   확인 후 checkpoint path와 SHA 계약을 한 커밋에서 함께 바꾼다.
4. 다음 재학습은 epoch 연장이 아니라 계획된 `Base3-Online` sensor-domain pilot을
   우선한다.

