# Base4 velocity head learnability와 LR 진단

> 날짜: 2026-09-22
> 상태: full train/val, 두 고주로 rosbag detection, tracker/Map-CV gate 완료.

## 먼저 내린 결론

기존 Base4 E4 결과만 보고 MR 데이터를 더 수집해서는 안 된다. 새로 초기화된 velocity
head에 Base3 spatial fine-tuning용 `2e-5` learning rate를 그대로 적용한 것이 큰 병목으로
확인됐다. 또한 현재 T1의 단순 3-sweep early concatenation은 같은 조건의 V1보다 velocity를
더 잘 학습하지 못했다.

따라서 다음 순서는 다음과 같다.

1. velocity loss와 LR 계약을 고친 full train/val V1/T1을 먼저 비교한다.
2. 같은 고주로 rosbag에서 detector velocity, tracking AVE와 Map-CV FDE를 평가한다.
3. T1이 V1보다 일반화 이득을 보이지 않으면 raw scene을 더 모으기 전에 temporal fusion
   구조를 바꾼다.
4. 기존 두 고주로 bag은 development 평가이며 최종 test로 주장하지 않는다.

## 왜 별도 learnability gate를 만들었나

첫 4-epoch 실험은 V1 raw val speed MAE `5.578 m/s`, T1 raw
`9.471 m/s`였다. 이 결과에는 세 요인이 섞여 있었다.

- Base3 E25 checkpoint에 velocity head가 없어 해당 head는 random initialization이다.
- 기존 recipe의 learning rate는 전체 spatial model 보존을 위한 `2e-5`였다.
- 팀 runner의 warmup `500 optimizer update`가 4-epoch run 전체 update의 약 30%를
  차지했다.

이를 분리하기 위해 training의 첫 64 frame을 train/val/test 모두에 사용했다. 이 결과는
일반화 성능이 아니라 새 head가 주어진 표본을 학습할 수 있는지만 보는 진단이다.

## 64-frame overfit 결과

모든 행은 같은 raw dataset의 동일한 첫 64 frame, batch 4, warmup 10 update를 사용했다.
64 frame에는 6개 scene, Vehicle 64개가 있으며 GT speed는 약 `2.72--16.67 m/s`다.

| 입력 | velocity code weight | LR | Epoch | speed MAE | mAVE | mATE |
|---|---:|---:|---:|---:|---:|---:|
| V1 single frame | 0.2 | 2e-5 | 40 | 6.909 | 6.928 | 0.141 |
| V1 single frame | 1.0 | 2e-5 | 40 | 4.564 | 4.580 | 0.133 |
| T1 3-sweep | 1.0 | 2e-5 | 40 | 5.349 | 5.363 | 0.146 |
| V1 single frame | 1.0 | 1e-4 | 20 | **1.766** | **1.803** | 0.119 |
| T1 3-sweep | 1.0 | 1e-4 | 20 | 2.553 | 2.573 | **0.112** |

핵심 해석은 다음과 같다.

- velocity weight `0.2 -> 1.0`은 유효하지만 그것만으로 충분하지 않다.
- LR `2e-5 -> 1e-4`의 효과가 훨씬 크다. velocity target 연결이나 단위가 완전히
  끊어진 버그는 아니다.
- V1은 한 frame만으로 절대 NPC velocity를 직접 관측할 수 없다. 64-frame 결과에는
  scene/위치/appearance memorization이 포함되므로 production 가능성을 의미하지 않는다.
- T1은 과거 sweep 2개를 Ego-motion 정합하고 point age를 다섯 번째 feature로 제공하지만,
  현재 early-fusion 구조에서는 V1보다 느리게 학습했다. 단순히 point를 더 넣는 것이
  temporal modeling 개선으로 이어지지 않았다.

## 데이터에 temporal velocity 단서가 실제로 있는가

있다. GT box로 동일 객체 point를 골라 직전 sweep을 현재 LiDAR frame으로 정합한 뒤,
current/past point median displacement를 시간차로 나눈 독립 sanity check를 수행했다.

- 유효 pair: `1,869`
- 추정 velocity와 GT `vx` 상관계수: `0.841`
- 2D velocity error p50: `1.440 m/s`
- speed MAE: `1.954 m/s`

이는 raw sweep에 동적 단서가 없다는 뜻이 아니라, 현재 T1 encoder가 noisy object trail과
time feature를 충분히 분리해 쓰지 못한다는 쪽을 지지한다. 이 계산은 GT box association을
사용한 oracle sanity check이므로 production estimator 수치로 사용하지 않는다.

## 구현된 재현 장치

팀 experiment runner에 다음 옵션을 추가했다.

- `schedule.warmup_optimizer_updates`: GPU별 gradient accumulation과 무관한 optimizer
  update 단위 warmup.
- `schedule.validation_interval`: epoch validation 주기.
- `dataset.overfit_samples`: 동일한 첫 N개 train 표본을 train/val/test에 사용하는
  diagnostic-only gate.

Velocity A/B config는 기존 box parameter 8개 weight를 유지하면서 `vx,vy`만
`0.2 -> 1.0`으로 바꾼다. 기존 production E25와 Base4 baseline config는 변경하지 않았다.

## Full V1 train/val 결과

동일 raw dataset의 sequence-disjoint train 1,688 frame / val 340 frame에서 V1의
학습률 적용 범위를 비교했다. Val에는 저속 구간이 빠져 있어 아래 수치는 최종 실주행
성능이 아니라 같은 split 안의 상대 비교로만 사용한다.

| V1 설정 | Epoch | center mAP | speed MAE | mAVE | mATE |
|---|---:|---:|---:|---:|---:|
| 전체 모델 LR `1e-4` | 2 | **0.3019** | **3.0772** | **3.1631** | 0.2130 |
| 전체 모델 LR `1e-4` | 4 | 0.3045 | 3.6269 | 3.6522 | 0.2011 |
| 전체 모델 LR `1e-4` | 8 | 0.3013 | 3.6921 | 3.7209 | 0.2111 |
| backbone `2e-5`, velocity head `1e-4` | 2 | 0.2899 | 3.8261 | 3.8454 | 0.2874 |
| backbone `2e-5`, velocity head `1e-4` | 4 | 0.2940 | 3.2884 | 3.3117 | 0.2467 |
| backbone `2e-5`, velocity head `1e-4` | 8 | 0.2951 | 3.2168 | 3.2467 | **0.2367** |

전체 모델 고 LR은 E2에서 가장 빠르게 좋아지지만 이후 velocity가 다시 악화됐다. 반면
head-only 고 LR은 느리지만 계속 개선됐다. 이는 velocity head만의 문제가 아니고 공유
BEV/decoder feature도 속도에 맞게 일부 적응해야 하지만, spatial backbone 전체를 높은
LR로 오래 갱신하는 것도 좋지 않음을 뜻한다.

현재 val 최상 V1은 전체 LR `1e-4`의 E2다. 이 checkpoint를 XYZI+time 입력으로 확장하며
기존 spatial/velocity parameter를 그대로 보존하고, 새 time channel만 zero init한 뒤 T1을
warm-start한다. T1에서는 backbone 기본 LR `2e-5`, velocity head와 sparse time input만
`1e-4`를 사용한다.

## T1 warm-start 결과

V1 전체 LR `1e-4` E2를 T1의 XYZI+time 입력으로 확장했다. 기존 네 채널과 velocity
head는 그대로 보존하고 새 time channel만 zero init했다. 이후 기본 LR `2e-5`, velocity
head와 sparse time input LR `1e-4`로 6 epoch 학습했다.

| Epoch | center mAP | speed MAE | mAVE | mATE |
|---:|---:|---:|---:|---:|
| 2 | **0.3026** | 2.7935 | 2.8272 | **0.2171** |
| 4 | 0.3023 | 2.7124 | 2.7465 | 0.2234 |
| 6 | 0.3012 | **2.3373** | **2.3729** | 0.2257 |

현재 early-concat T1도 올바른 초기화와 branch LR에서는 velocity를 학습한다. V1 E2
대비 E6 speed MAE는 약 24% 좋아졌고 geometry는 비슷한 수준이다. 다만 이 val split은
같은 자동 수집 분포의 sequence-disjoint split이므로 실주행 일반화를 대신하지 않는다.

## 동일 고주로 rosbag detection 결과

Vehicle threshold `0.40`, 30 m, center-distance protocol, MR heading correction
`-1.347 deg`를 동일하게 적용했다.

| 모델 | bag | AP@center mean | F1 | mATE | speed MAE | mAVE |
|---|---|---:|---:|---:|---:|---:|
| V1 E2 | merge | 0.7560 | 0.8581 | **0.3499** | 4.2139 | 4.2960 |
| T1 E6 | merge | 0.7503 | **0.8810** | 0.3632 | **3.4847** | **3.5380** |
| V1 E2 | highway1 | 0.7647 | 0.8840 | 0.4041 | **2.7621** | **2.8182** |
| T1 E6 | highway1 | **0.7742** | **0.9128** | **0.3963** | 2.8077 | 2.8509 |

T1은 merge velocity를 약 17% 개선했지만 highway1에서는 V1과 같은 수준이다. 공간 검출은
악화되지 않았으나, 이 결과만으로 detector velocity를 tracker에 매 frame 넣을 수는 없다.

## Detector velocity의 시간 안정성

동일 GT 차량을 frame별 1 m 이내에서 대응하고, 예측 map velocity를 causal GT heading에
투영했다. `error step`은 인접 detection frame 사이에서 signed longitudinal velocity
error가 얼마나 변했는지를 뜻한다.

| 모델 | bag | signed bias | speed MAE | error step p50/p95 | GT speed step p50/p95 |
|---|---|---:|---:|---:|---:|
| V1 E2 | merge | +3.953 | 4.213 | 0.757 / 2.759 | 0.164 / 0.509 |
| V1 E2 | highway1 | +2.071 | 2.572 | 0.748 / 2.465 | 0.149 / 0.488 |
| T1 E6 | merge | **-0.291** | **3.419** | 0.969 / 4.220 | 0.163 / 0.483 |
| T1 E6 | highway1 | **-0.265** | 2.589 | 0.939 / 3.810 | 0.145 / 0.481 |

T1은 V1의 큰 양의 bias를 제거했지만 frame-to-frame jitter는 오히려 커졌다. T1의 apparent
acceleration p95는 merge/highway1에서 각각 `49.7/41.9 m/s²`이고 GT는 `6.4/4.1
m/s²`다. 평균 speed MAE 하나만 보면 숨는 실패이며, velocity head 선택에는 temporal
stability metric이 필수다.

재현 도구는
`base_ws/src/tracking_prediction_evaluation/scripts/analyze_detection_velocity_temporal.py`다.

## Tracker velocity fusion gate

Tracker 모드는 다음 의미다.

- mode 0: detector velocity 무시.
- mode 1: 새 track 초기화에만 사용.
- mode 2: 초기화와 매 detection Kalman update에 사용.

아래는 CV-stable slice의 결과다. FDE는 best-of-K 3초 평균이다.

| T1 E6 모드 | bag | center mean | speed MAE | AVE | FDE@3s | coverage |
|---|---|---:|---:|---:|---:|---:|
| mode 0 | merge | 0.509 | 1.045 | 1.104 | 3.080 | 0.916 |
| mode 1, noise 1 | merge | **0.513** | **0.988** | **1.061** | **2.476** | 0.914 |
| mode 2, noise 4 | merge | 0.589 | 1.215 | 1.298 | 3.692 | 0.907 |
| mode 2, noise 1 | merge | 0.969 | 2.488 | 2.532 | 7.974 | 0.760 |
| mode 0 | highway1 | 0.404 | 0.637 | 0.747 | 1.918 | 0.953 |
| mode 1, noise 1 | highway1 | 0.404 | **0.606** | **0.714** | **1.847** | 0.946 |
| mode 2, noise 4 | highway1 | **0.398** | 0.734 | 0.832 | 2.274 | 0.946 |
| mode 2, noise 1 | highway1 | 0.517 | 1.763 | 1.808 | 6.202 | 0.930 |

Mode 1은 두 bag에서 소폭 또는 중간 수준의 이득이 있지만, mode 2는 covariance를 느슨하게
해도 악화된다. 따라서 production 기본은 계속 mode 0이며, T1을 사용한다면 mode 1만 후보로
남긴다. 매-frame fusion은 velocity temporal jitter를 모델 또는 measurement policy에서
줄인 뒤 다시 연다.

## 다음 결정

1. 더 많은 동일 분포 raw scene 수집은 당장 우선하지 않는다.
2. E6에서 val velocity가 계속 개선됐으므로 낮은 LR의 짧은 continuation을 한 번만 수행해
   under-training 여부를 분리한다. 이 실험은 아래와 같이 완료됐다.
3. Rosbag jitter가 유지됐으므로 early concatenation 추가 epoch를 중단하고, object-aware
   temporal aggregation 또는 temporal consistency 학습으로 넘어간다.
4. production detector와 tracker 기본값은 이 gate만으로 변경하지 않는다.

## Low-LR continuation 결과

T1 E6에서 base LR `1e-5`, velocity/time branch `5e-5`로 4 epoch를 더 학습했다. 별도
optimizer/scheduler로 시작한 fine-tuning run이므로 아래 epoch 1--4는 누적 E7--E10에
해당하는 비교로만 해석한다.

| continuation epoch | center mAP | speed MAE | mAVE | mATE |
|---:|---:|---:|---:|---:|
| 1 | 0.3010 | 2.2923 | 2.3185 | 0.2251 |
| 2 | 0.3033 | 2.2632 | 2.2947 | **0.2170** |
| 3 | 0.3040 | **2.2441** | **2.2732** | 0.2203 |
| 4 | **0.3040** | 2.2770 | 2.3065 | 0.2212 |

Epoch 3를 같은 rosbag에 재평가했다.

| 모델 | bag | AP | mATE | speed MAE | mAVE | signed bias | error-step p95 |
|---|---|---:|---:|---:|---:|---:|---:|
| T1 E6 | merge | 0.7503 | 0.3632 | 3.4847 | 3.5380 | -0.291 | **4.220** |
| T1 cont E3 | merge | **0.7639** | **0.3545** | **3.4422** | **3.4913** | **-0.180** | 4.299 |
| T1 E6 | highway1 | 0.7742 | 0.3963 | 2.8077 | 2.8509 | -0.265 | **3.810** |
| T1 cont E3 | highway1 | **0.7787** | **0.3948** | **2.6612** | **2.7014** | **-0.216** | 3.956 |

평균 detection 지표는 조금 좋아졌지만 temporal jitter는 두 bag 모두 악화됐다. Tracker
mode 1의 두-bag macro stable minFDE-K 3초는 `2.161 -> 2.074 m`로 개선됐으나, V1 E2
position-only의 `1.646 m`에는 여전히 크게 못 미친다. Mode 0은 오히려 `2.499 ->
2.524 m`로 나빠졌다.

따라서 추가 epoch는 본질적 해결이 아니다. 다음 실험은 dataset 증량이나 E12 연장이 아니라
시간적으로 일관된 velocity를 직접 학습·집계하는 구조여야 한다. W&B run:
<https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-highway-t1-continue-e6-newheads5x-e4-rtx5060ti-260922-161431>.
