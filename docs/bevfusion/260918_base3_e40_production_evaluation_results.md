# Base3 BEVFusion Fresh E40 production 평가 결과

## 결론

- Corrected offline validation best는 `B3-BF0-F40 E26`이다.
- Production 표준은 `B3-BF0-F40 E25`로 정한다.
- 실제 경로 detection AP는 E10이 근소하게 높지만, 최종 시스템 목표인 Vehicle
  tracking AVE와 Map-CV FDE가 E25에서 일관되게 개선됐다.
- 기존 `B3-BF0 E10`은 rollback baseline으로 보존한다.
- 초기 PointPainting 통제 실험인 `Vehicle 30 m` bag은 대회 경로 분포를
  대표하지 않으므로 production 평균과 모델 선택에서 제외했다.
- 다음 재학습은 epoch 연장보다 계획된 `Base3-Online` sensor-domain pilot을
  우선한다.

Corrected W&B run:
<https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-fresh-e40-range30-corrected-val>

## 평가 계약

- Base3 frozen split.
- GT와 prediction 모두 `[-30, -30, -3, 30, 30, 2] m` box-center 필터.
- Val GT: Vehicle 1,100 / Pedestrian 1,005 / Obstacle 2,185.
- Rosbag threshold: Vehicle/Pedestrian/Obstacle `0.40/0.15/0.10`.
- Production detection은 실제 경로·미션 7-bag이며 frame coverage는 모두 `1.0`이다.
- 고주로1은 `0.5x`, multi-asset obstacle은 `0.25x`, 나머지는 `1.0x`다.
- Tracking/Map-CV는 실제 경로 Vehicle GT가 존재하는 로터리·고주로 합류·
  고주로1 세 bag만 평가했다.

## 제외한 통제 bag

`pointpainting_vehicle_static_moving_within30m_raw.bag`은 2026-07-31에 만든
68.3초 길이의 PointPainting Vehicle 전용 통제 평가다. 30 m 이내 정적·동적
차량 회귀 비교에는 유용하지만, 최근 대회 경로 주행과 객체 분포를 대표하지 않는다.

E25 AP가 이 bag에서 `0.6924 -> 0.8436`으로 크게 올라 기존 8-bag macro를
과도하게 끌어올렸다. 파일은 과거 모델과의 controlled regression 자산으로
보존하되 production suite와 평균에서는 사용하지 않는다.

## Corrected Val과 Test

| 모델 | Split | Center mAP | IoU mAP@0.5 |
|---|---|---:|---:|
| B3-BF0 E10 | Test | 0.974991 | 0.947590 |
| B3-BF0-F40 E25 | Val | 0.980829 | 0.956486 |
| B3-BF0-F40 E25 | Test | 0.979407 | **0.961747** |
| **B3-BF0-F40 E26** | **Val** | **0.981408** | 0.960322 |
| B3-BF0-F40 E26 | Test | **0.980080** | 0.961191 |
| PointPainting PP1 E34 | Test | 0.9809 | 0.9640 |

E25 Test class AP는 Vehicle `0.96674`, Pedestrian `0.98878`, Obstacle
`0.98271`이다. E10 대비 Vehicle AP는 조금 낮지만 Vehicle mATE는
`0.0772 -> 0.0703 m`, Obstacle AP는 `0.96466 -> 0.98271`, Obstacle mATE는
`0.0554 -> 0.0483 m`로 개선됐다. Offline 성능만으로 production을 고르지는 않는다.

## Production mission rosbag detection

아래 AP는 center distance `0.5/1/2/4 m` 평균이다.

| Rosbag / target class | E10 | E25 | E26 | E25-E10 |
|---|---:|---:|---:|---:|
| 신로터리 / Vehicle | **0.7725** | 0.7565 | 0.7626 | -0.0160 |
| 고주로 합류 / Vehicle | **0.5833** | 0.5700 | 0.5585 | -0.0133 |
| 고주로1 / Vehicle | 0.7667 | **0.7716** | 0.7681 | +0.0049 |
| Pedestrian 횡단 / Pedestrian | **1.0000** | 0.9999 | 0.9889 | -0.0001 |
| Pedestrian 거리별 / Pedestrian | **1.0000** | 0.9889 | **1.0000** | -0.0111 |
| Obstacle 왕복 / Obstacle | **0.6221** | 0.6002 | 0.6106 | -0.0219 |
| Base3 multi-asset / Obstacle | 0.6067 | 0.6472 | **0.6487** | +0.0405 |
| **7-bag macro** | **0.7645** | 0.7620 | 0.7625 | -0.0024 |

Class별 production macro AP:

| 모델 | Vehicle 3-bag | Pedestrian 2-bag | Obstacle 2-bag |
|---|---:|---:|---:|
| **E10** | **0.7075** | **1.0000** | 0.6144 |
| E25 | 0.6994 | 0.9944 | 0.6237 |
| E26 | 0.6964 | 0.9944 | **0.6296** |

E25/E26의 Obstacle macro는 개선됐지만 E25는 왕복 bag에서 퇴행했고,
E26도 Vehicle 로터리·합류가 낮다. 현재 mission detection만 보면 E10을 교체할
근거가 없다.

## 실제 경로 Vehicle tracking과 Map-CV

저장된 detection bag을 같은 production EKF, UNKNOWN stationary policy와
Map-CV에 재생했다. 아래 macro에는 로터리·고주로 합류·고주로1만 포함한다.
AVE는 velocity vector L2 error다.

| 모델 | Tracking F1 ↑ | Center p50 m ↓ | Speed MAE m/s ↓ | AVE m/s ↓ | FDE@1s m ↓ | FDE@2s m ↓ | FDE@3s m ↓ | 3s coverage ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| E10 | 0.9377 | **0.5403** | 0.5926 | 0.7887 | 1.1357 | 2.5669 | 4.7168 | **0.9118** |
| **E25** | **0.9391** | 0.5457 | 0.5683 | **0.7252** | **1.0849** | **2.4427** | **4.4912** | 0.9103 |
| E26 | 0.9376 | 0.5547 | **0.5653** | 0.7416 | 1.0896 | 2.4643 | 4.5502 | 0.9095 |

CV-stable macro FDE@1/2/3s도 E10 `0.874/1.592/2.748 m`에서 E25
`0.846/1.503/2.561 m`로 개선됐다. 반면 E25 center p50은 E10보다 `5.4 mm`
나쁘고 3초 coverage도 `0.0016` 낮다. E25는 속도·prediction 개선 후보이지
전 지표에서 우월한 모델은 아니지만 최종 planning 입력 품질 기준으로 선택한다.

E25의 구간별 center p50 / top-1 FDE@3s 변화:

| Rosbag | E10 center p50 | E25 center p50 | E10 FDE@3s | E25 FDE@3s |
|---|---:|---:|---:|---:|
| 신로터리 | 0.433 m | **0.417 m** | 4.625 m | **4.232 m** |
| 고주로 합류 | **0.717 m** | 0.750 m | 5.830 m | **5.619 m** |
| 고주로1 | 0.472 m | **0.470 m** | 3.695 m | **3.623 m** |

합류 위치 오차는 나빠졌고, 로터리·고주로1의 velocity/prediction은 개선됐다.
합류 위치 오차 퇴행이 판단·추월에 영향을 주지 않는지는 MORAI interactive와
현장 로그에서 계속 감시한다.

## Artifact

- E25 full checkpoint:
  `artifacts/detection3d/bevfusion/team_training/sw/bf-sw-fresh-e40-rtx5060ti-260918-005536/epoch_25.pth`
- E25 full SHA-256:
  `0fc30b97517484d1b2da1caef716e358af57e4049b2e7637820a080e54b03df3`
- E25 inference checkpoint:
  `artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth`
- E25 inference SHA-256:
  `36816cf769c87d49483d002111abfad8e1afd043afd74a0b9fc54aac82133011`
- E26 full SHA-256:
  `e819eb24221e82af9a95d6cdef05a0da7a0c6f3da33e98bb970e1843d231ea0b`

Inference artifact는 team CUDA 12.8 image에서 server load와 healthcheck
`model_loaded=True`를 통과했다.

## 최종 판단

1. Offline 기록은 E26 best, E25 runner-up으로 남긴다.
2. 최종 시스템 목표를 tracker/Map-CV의 속도와 미래경로 품질로 두고 production
   표준은 E25로 정한다.
3. E10은 per-frame mission detection이 더 중요한 상황의 rollback baseline으로
   보존한다.
4. E25 checkpoint path와 SHA를 production 실행 계약에 고정한다.
5. 다음 데이터 실험은 `Base3-Online` sensor-domain pilot을 우선한다.
