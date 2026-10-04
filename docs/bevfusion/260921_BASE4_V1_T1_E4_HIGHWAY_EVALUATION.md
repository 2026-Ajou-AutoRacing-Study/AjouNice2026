# Base4 고주로 V1/T1 E4 평가

> 날짜: 2026-09-21
> 상태: 개발용 1차 판정. production 모델 선정 결과가 아니다.

## 결론

- 두 학습은 모두 epoch 4까지 정상 종료됐고 각 run의 E4가 best checkpoint다.
- single-frame 9D 모델 `B4H-V1`은 기존 E25보다 두 고주로 bag의 차량 검출
  geometry가 좋아졌다.
- 3-sweep 9D 모델 `B4H-T1`은 V1보다 합류 구간 center는 조금 정확하지만 recall,
  순항 성능과 latency가 나빠졌다. 현재 방식의 단순 sweep concatenation은 채택하지 않는다.
- 두 모델 모두 in-domain val velocity는 좋아 보였지만 rosbag detector velocity AVE는
  약 `3.8--4.0 m/s`다. 이 velocity를 EKF 매 update에 넣으면 association, tracking center와
  prediction이 크게 악화됐다. **production velocity fusion은 계속 off로 둔다.**
- 이 pilot은 vehicle-only 고주로 데이터로 학습한 개발 실험이다. Pedestrian/Obstacle을
  포함한 대회 production E25를 대체하지 않는다.

## 모델과 평가 계약

| 이름 | 입력 | checkpoint |
|---|---|---|
| B4H-V1 E4 | current LiDAR + 4 camera, 9D box | `bf-sw-base4-hwy3h1-v1-e4-rtx5060ti-260921-084026/best_val_center_mAP_epoch_4.pth` |
| B4H-T1 E4 | current + causal past 2 LiDAR sweeps + 4 camera, 9D box | `bf-sw-base4-hwy3h1-t1-3sweep-e4-rtx5060ti-260921-103639/best_val_center_mAP_epoch_4.pth` |

- score threshold: Vehicle `0.40`
- range: 30 m
- GT: `asset-catalog-bbox-center-v1`
- T1 online input: 과거 sweep을 Ego pose로 현재 LiDAR frame에 정렬하고 양수 age를
  fifth point feature로 기록한다.
- rosbag: 고주로 합류와 `highway1`. 둘 다 반복 사용된 development bag이며 새로운
  held-out test가 아니다.

## 학습 validation

두 run의 val은 같은 생성기·route·speed family에서 나온 쉬운 in-domain split이다.

| 모델 | Vehicle mean AP | AP@0.5 m | mATE | mASE | mAOE | mAVE | speed MAE |
|---|---:|---:|---:|---:|---:|---:|---:|
| V1 E4 | **0.9811** | **0.9576** | 0.1266 m | **0.0866** | 0.0175 rad | 0.5187 m/s | **0.4155 m/s** |
| T1 E4 | 0.9801 | 0.9536 | **0.1218 m** | 0.0948 | **0.0136 rad** | **0.5135 m/s** | 0.4524 m/s |

차이가 매우 작고 서로 다른 지표가 엇갈린다. val만 보면 T1 채택 근거가 없다.

## rosbag detection

| Bag | 모델 | mAP | F1@1 m | mATE | mASE | mAOE |
|---|---|---:|---:|---:|---:|---:|
| 합류 | E25 참고 | 0.6261 | 0.8218 | 0.6264 m | **0.1161** | **0.0271 rad** |
| 합류 | V1 | **0.7941** | **0.9292** | 0.3741 m | 0.1323 | 0.0459 rad |
| 합류 | T1 | 0.7642 | 0.9005 | **0.3389 m** | 0.1521 | 0.0343 rad |
| highway1 | E25 참고 | 0.7381 | 0.9372 | 0.5439 m | **0.0996** | **0.0271 rad** |
| highway1 | V1 | **0.7976** | **0.9562** | **0.4124 m** | 0.1275 | 0.0379 rad |
| highway1 | T1 | 0.7118 | 0.8816 | 0.4566 m | 0.1313 | 0.0428 rad |

E25 행도 같은 catalog GT로 재평가한 값이라 detection 비교에는 사용할 수 있다. V1의
geometry 개선은 분명하지만 vehicle-only highway specialization의 결과이므로 전역 모델
교체 근거는 아니다.

프레임 coverage는 V1 두 bag 모두 100%, T1은 합류 `420/421` evaluated frame 및
highway1 100%였다.

## detector velocity의 rosbag 일반화

2 m 이내 geometry match에 한해 detector가 발행한 velocity와, 보정된 BBox-center
trajectory를 causal position fit한 GT velocity를 비교했다.

| Bag | 모델 | match | Pred speed mean | GT speed mean | Speed MAE | 2D AVE | Lateral MAE |
|---|---|---:|---:|---:|---:|---:|---:|
| 합류 | V1 | 288 | 12.291 | 9.792 | 3.843 | 3.966 | 0.733 |
| 합류 | T1 | 276 | 12.245 | 9.710 | **3.716** | **3.820** | **0.659** |
| highway1 | V1 | 394 | 12.802 | 10.954 | **3.644** | **3.753** | 0.849 |
| highway1 | T1 | 374 | 13.032 | 10.650 | 3.714 | 3.770 | **0.677** |

T1은 횡속도는 줄였지만 종속도 오차가 지배적이고, 전체 AVE 개선은 합류에서 3.7%,
highway1에서는 사실상 없다. val `~0.5 m/s`와 rosbag `~3.8 m/s`의 차이는 현재
동일 생성기 val이 velocity domain shift를 대표하지 못한다는 직접적인 증거다.

## tracking과 Map-CV prediction: velocity fusion off

현재 production tracker 계약인 CTRV, measurement XY noise 0.4 m, detector velocity
fusion off로 비교했다.

| Bag | 모델 | Track F1 | MOTA | Center mean/p50 | Speed MAE | 2D AVE | Top-1 FDE@1/2/3 s |
|---|---|---:|---:|---:|---:|---:|---:|
| 합류 | V1 | **0.909** | **0.832** | 0.496 / 0.437 m | **0.825** | **1.039** | 1.554 / **3.341** / **5.717** m |
| 합류 | T1 | 0.890 | 0.796 | **0.435 / 0.346 m** | 0.862 | 1.104 | **1.523** / 3.428 / 5.992 m |
| highway1 | V1 | **0.978** | **0.957** | **0.412 / 0.356 m** | **0.471** | **0.708** | **0.864 / 2.075 / 3.842** m |
| highway1 | T1 | 0.921 | 0.846 | 0.460 / 0.396 m | 0.602 | 0.790 | 1.014 / 2.369 / 4.336 m |

합류 전체 구간의 FDE는 NPC 가감속이 섞여 크다. CV-stable motion-only FDE@3 s도
V1/T1이 각각 합류 `1.940/2.119 m`, highway1 `0.948/1.286 m`로 V1이 낫다.

## detector velocity fusion mode 2

9D velocity를 `has_twist=true`로 발행하고 noise 1.0 m/s로 EKF 매 update에 넣었다.

| Bag | 모델 | Track F1 | Center p50 | Speed MAE | 2D AVE | Top-1 FDE@3 s |
|---|---|---:|---:|---:|---:|---:|
| 합류 | V1 | 0.706 | 0.665 m | 1.405 | 1.530 | 3.450 m |
| 합류 | T1 | 0.669 | 0.568 m | 1.334 | 1.418 | 2.817 m |
| highway1 | V1 | 0.831 | 0.939 m | 2.055 | 2.148 | 5.595 m |
| highway1 | T1 | 0.753 | 0.946 m | 2.024 | 2.074 | 5.473 m |

일부 FDE 평균은 잘못된 velocity로 association/coverage가 무너지며 쉬운 track만 남은
selection effect가 포함되므로 개선으로 해석하면 안 된다. F1, MOTA, center와 AVE가 모두
큰 폭으로 악화한다. covariance tuning으로 숨길 문제가 아니라 우선 velocity generalization을
고쳐야 한다.

## Runtime

5060 Ti에서 저장된 online latency는 다음과 같다.

| 모델 | Bag | mean | p50 | p95 | 평균 latency 역수 |
|---|---|---:|---:|---:|---:|
| V1 | 합류 | 83.7 ms | 82.1 | 92.8 | 11.94 Hz |
| V1 | highway1 | 82.5 ms | 82.2 | 86.7 | 12.13 Hz |
| T1 | 합류 | 101.8 ms | 94.8 | 134.9 | 9.83 Hz |
| T1 | highway1 | 95.8 ms | 95.4 | 99.8 | 10.44 Hz |

T1은 V1보다 약 16--22% 느리고 합류 tail latency도 크다.

## 구현 중 확인한 runtime 계약 누락

기존 online bridge에는 두 문제가 있었다.

1. 3-sweep checkpoint를 실행해도 current Nx4 point만 서버에 전달했다.
2. 서버가 반환한 9D box의 velocity를 ROS `DetectedObjects`에 싣지 않았다.

이번 평가를 위해 다음을 opt-in으로 구현했다.

- Nx5 point runtime과 causal sweep buffer.
- Ego pose 기반 past-to-current sweep 정렬과 positive age feature.
- `publish_detector_velocity=true`일 때 Vehicle 9D velocity를 body-frame twist로 발행.
- Base3/E25 기본값은 temporal 0, velocity publish false로 보존.
- detection evaluator에 detector velocity speed/2D/종·횡 오차를 추가.

## 다음 실험

1. 동일 생성기 val이 아닌 held-out 실주행형 고주로 sequence를 validation에 포함한다.
2. 속도별 balanced sampling 또는 continuous speed target 분포를 구성한다. 현재 모델은
   rosbag에서 평균 속도를 약 2 m/s 높게 내며 개별 오차는 약 3.8 m/s다.
3. T1 단순 point concatenation은 중단하고, 먼저 V1 velocity label/loss/calibration을
   해결한다. 이후 temporal은 T2 BEV feature fusion 또는 query memory를 통제 실험한다.
4. velocity fusion은 두 bag 모두 AVE/FDE gate를 통과하기 전까지 off를 유지한다.

별도 `perception/base4-temporal-3sweep` 구현은 같은 raw-point concatenation 계열이며
online runtime과 deskew가 없고 time 부호·velocity 단위가 현재 계약과 다르므로 모델
경로 전체를 merge하지 않는다. 해당 구현의 scene leakage, sweep transform 및
trajectory-velocity 검사만 현재 `audit_base4_temporal_quality.py`에 이식했다.

## 산출물

- detection:
  - `artifacts/evaluation/bevfusion/base4_hwy3h1_v1e4_velocity_highway_260921`
  - `artifacts/evaluation/bevfusion/base4_hwy3h1_t1e4_3sweep_velocity_highway_260921`
- production tracker 계약, velocity off:
  - `artifacts/evaluation/bevfusion/base4_hwy3h1_v1e4_highway_pipeline_260921_v2`
  - `artifacts/evaluation/bevfusion/base4_hwy3h1_t1e4_3sweep_highway_pipeline_260921_v2`
- real detector velocity, mode 2:
  - `artifacts/evaluation/bevfusion/base4_hwy3h1_v1e4_velocity_pipeline_mode2_final_260921`
  - `artifacts/evaluation/bevfusion/base4_hwy3h1_t1e4_3sweep_velocity_pipeline_mode2_final_260921`
