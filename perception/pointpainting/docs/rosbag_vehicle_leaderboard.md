# PointPainting Vehicle rosbag leaderboard.

## 목적.

- 동일한 실주행 입력에서 PointPainting checkpoint의 OD, Tracking, Map-CV
  성능을 비교하는 production 선발표임.
- 각 모델은 항상 한 행으로 추가함.
- offline validation 점수는 이 표에 섞지 않음. Base2 validation과 실제 rosbag의
  모델 순위가 일치하지 않았기 때문임.
- 현재 두 benchmark에는 Vehicle GT만 존재함. Pedestrian·Obstacle AP를 이 표로
  판단하지 않으며, 해당 class의 benchmark bag이 생기면 별도 표를 추가함.

## 고정 입력과 평가 규칙.

| 별칭 | 고정 rosbag |
|---|---|
| 신로터리 | `rosbags/raw_sensor_w_cam/perception_benchmark/260821_235243_roundabout_dynamic_competition_raw.bag` |
| 고주로1 | `rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag` |

- OD는 `morai-online-center-distance-v1`을 사용함.
- GT reference는 `bbox-center-calibrated-v1`을 사용함.
- 검출 범위는 `30 m`, production score threshold는 `0.10`으로 고정함.
- `F1@1m`은 best threshold가 아니라 고정 score `0.10`의 operating point임.
- `/ score0.40` 행만 예외이며, Base2 Val의 Vehicle·Obstacle best score가 각각
  `0.407`, `0.409`였던 결과를 사전에 반올림한 calibration 후보임. 두 rosbag의
  best threshold를 장면별로 골라 쓰지 않음.
- Tracking 지표는 전체 녹화 구간을 사용함.
- Prediction은 GT 종방향 가속도 절댓값이 전체 horizon 동안 `0.5 m/s²`
  이하인 CV-stable 구간을 사용함.
- Planning이 plausible Top-K를 받는다는 전제에 맞춰 Prediction 주 지표는
  `minFDE-K`임.
- `—`는 미평가임. 추정값이나 다른 bag 결과로 채우지 않음.

## 모델 설명.

| 모델 | 소유·구성 | 대표 checkpoint | 비고 |
|---|---|---|---|
| Legacy E65 | 팀원 학습. Static 1,500-frame, single-frame 9-feature. | `pointpainting_B0_best_epoch65.pth` | 이전 production baseline. |
| Team B0 E80 / 1S | 팀원 학습. Base2, object-compensated 3-sweep 학습 checkpoint를 online 1-sweep으로 입력. | `checkpoint_epoch_80.pth` | 학습·추론 sweep 불일치 ablation임. |
| Team B0 E80 / 3S | 위와 같은 checkpoint를 Ego-only 3-sweep online 입력으로 평가. | `checkpoint_epoch_80.pth` | 학습의 object compensation과 online 입력이 완전히 같지 않음. |
| T0 E35 | 우리 학습. Ego-aligned 3-sweep, detector Vehicle velocity head. | `checkpoint_epoch_35.pth` | Detector velocity를 사용하지 않는 downstream 결과와 혼동하지 않아야 함. |
| S0 E28 | 우리 학습. Base2 single-frame, 기하 증강 없음. | `checkpoint_epoch_28.pth` | Single-frame 통제군. |
| S1 E38 | 우리 학습. S0 + LR/FB flip, rotation, scaling. | `best_center_mAP_epoch_38.pth` | 현재 최상위 production 후보. |
| S1b E30 | 우리 학습. S1에서 FB flip만 제거. | `best_center_mAP_epoch_30.pth` | Offline 동률, online 하락으로 기각. |
| S1g E37 | 우리 학습. S1 + class별 GT database sampling 10개. | `best_center_mAP_epoch_37.pth` | Vehicle score 0.40에서는 최상위 후보지만 전체 3-class production 승격은 calibration 검증 결과 보류함. |
| S1h E32 | 우리 학습. S1g에서 Obstacle GT sampling만 10개에서 3개로 감소. | `best_center_mAP_epoch_32.pth` | Obstacle FP는 감소했지만 Pedestrian·로터리 Vehicle이 하락해 기각함. |
| B3S-PP0 E39 | 우리 학습. Base3-Static 10,902-frame, single-frame 9-feature, Base3 LRASPP, 기하 증강, GT sampling 없음. | `best_center_mAP_epoch_39.pth` | Class-centric spatial baseline. 고주로 Tracking은 강하지만 로터리 Prediction과 횡단 Pedestrian이 하락해 production 승격은 보류함. |

## Offline Base2 Val.

`mAP`와 class AP는 checkpoint 선택 epoch의 center-distance mean AP임. `mATE`는
같은 epoch에서 2 m 이내 matched TP의 center error임. Offline Val과 online
rosbag은 입력 분포가 다르므로 서로 대체하지 않고 함께 판단함.

| 모델 | 3-class mAP | Vehicle AP | Pedestrian AP | Obstacle AP | Vehicle mATE | Pedestrian mATE | Obstacle mATE |
|---|---:|---:|---:|---:|---:|---:|---:|
| S1 E38 | 0.9417 | 0.9767 | 0.9918 | 0.8565 | 0.096 m | 0.051 m | 0.091 m |
| **S1g E37** | **0.9618** | **0.9960** | **0.9966** | **0.8929** | **0.062 m** | 0.042 m | **0.068 m** |
| S1h E32 | 0.9533 | 0.9960 | 0.9946 | 0.8694 | 0.067 m | **0.042 m** | 0.081 m |

## Pedestrian·Obstacle controlled rosbag.

`best F1`은 장면별 threshold sweep의 진단 상한임. 실제 production에서 장면별로
threshold를 바꾼다는 뜻이 아님. 이 bag들은 2026-07 sensor 조건이므로 절대값보다
모델 간 상대 비교에 사용함.

| 모델 | Ped crossing mAP / best F1 | Ped distance mAP / best F1 | Obstacle3 mAP / best F1 | Ped-negative Obstacle mAP / best F1 |
|---|---:|---:|---:|---:|
| S1 E38 | 0.000 / 0.022 | 0.000 / 0.009 | 0.067 / 0.305 | 0.129 / **0.444** |
| **S1g E37** | **0.442 / 0.671** | **0.642 / 0.821** | 0.157 / 0.336 | **0.261** / 0.330 |
| S1h E32 | 0.381 / 0.587 | 0.444 / 0.672 | **0.169 / 0.414** | 0.067 / 0.279 |

### Vehicle-only scene class FP at score 0.40.

| 모델 | 로터리 Vehicle / Ped / Obstacle FP | 고주로 Vehicle / Ped / Obstacle FP |
|---|---:|---:|
| S1g E37 | 16 / 0 / 3 | 20 / 2 / 79 |
| **S1h E32** | **4 / 0 / 0** | **18 / 3 / 21** |

- S1h는 목표했던 Obstacle FP 감소에는 성공함.
- 그러나 Pedestrian 두 bag과 다른 Obstacle scene AP가 함께 하락함.
- Obstacle sampling은 단순 class 수량뿐 아니라 세 class feature 학습에 영향을
  주는 regularizer였다고 판단함.

## OD.

단위는 `mATE=m`, `mAOE=rad`임. `Coverage`는 Detection message 수를 raw
LiDAR frame 수로 나눈 값임.

| 모델 | 로터리 Coverage | 로터리 center mAP | 로터리 F1@1m | 로터리 mATE | 로터리 mAOE | 고주로 Coverage | 고주로 center mAP | 고주로 F1@1m | 고주로 mATE | 고주로 mAOE |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Legacy E65 | 94.2% | 0.657 | 0.874 | 0.542 | 0.885 | 97.6% | 0.657 | 0.801 | 0.513 | 0.583 |
| Team B0 E80 / 1S | 94.2% | 0.446 | 0.731 | 0.552 | 1.049 | 97.6% | 0.646 | 0.889 | 0.492 | 0.641 |
| Team B0 E80 / 3S | 93.8% | 0.200 | 0.360 | 0.873 | 1.131 | 97.1% | 0.268 | 0.409 | 0.847 | 0.515 |
| T0 E35 | 93.8% | 0.414 | 0.578 | 0.636 | 0.223 | — | — | — | — | — |
| S0 E28 | 94.2% | 0.527 | 0.623 | 0.550 | 0.321 | 97.6% | 0.712 | 0.574 | 0.449 | 0.061 |
| **S1 E38** | 94.2% | **0.688** | **0.884** | **0.516** | 0.785 | 97.6% | **0.772** | **0.940** | **0.462** | 0.472 |
| S1b E30 | 94.2% | 0.607 | 0.789 | 0.572 | 1.018 | 97.6% | 0.710 | 0.838 | 0.539 | 0.063 |
| S1g E37 / score0.10 | 94.2% | 0.720 | 0.888 | 0.502 | 0.522 | 97.6% | 0.803 | 0.882 | 0.436 | 0.546 |
| **S1g E37 / score0.40** | 94.2% | **0.720** | **0.892** | **0.502** | **0.522** | 97.6% | **0.803** | 0.940 | **0.436** | 0.546 |
| S1h E32 / score0.40 | 93.8% | 0.683 | 0.853 | 0.507 | 0.648 | 97.6% | 0.783 | 0.934 | 0.466 | 0.653 |
| B3S-PP0 E39 / score0.10 | 94.5% | 0.715 | 0.871 | 0.530 | 0.611 | 97.6% | 0.765 | 0.938 | 0.457 | 0.366 |

## Tracking.

`Center p50`은 matched track BBox center와 보정된 GT BBox center 사이 거리임.
`Speed p50`은 matched track의 절대 속도 오차임.

| 모델 | 로터리 F1 | 로터리 MOTA | 로터리 Center p50 | 로터리 Speed p50 | 고주로 F1 | 고주로 MOTA | 고주로 Center p50 | 고주로 Speed p50 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Legacy E65 | 0.936 | 0.872 | 0.558 m | 0.406 m/s | 0.862 | 0.721 | 0.538 m | 0.493 m/s |
| Team B0 E80 / 1S | — | — | — | — | — | — | — | — |
| Team B0 E80 / 3S | — | — | — | — | — | — | — | — |
| T0 E35 | 0.659 | 0.374 | 0.653 m | 0.591 m/s | — | — | — | — |
| S0 E28 | 0.769 | 0.551 | **0.473 m** | 0.456 m/s | — | — | — | — |
| **S1 E38** | 0.917 | 0.840 | 0.530 m | **0.323 m/s** | **0.953** | **0.907** | **0.466 m** | **0.370 m/s** |
| S1b E30 | 0.884 | 0.780 | 0.569 m | 0.412 m/s | 0.915 | 0.827 | 0.497 m | 0.399 m/s |
| S1g E37 / score0.10 | 0.943 | 0.885 | 0.531 m | 0.318 m/s | 0.862 | 0.724 | 0.430 m | 0.327 m/s |
| **S1g E37 / score0.40** | **0.960** | **0.922** | **0.464 m** | **0.312 m/s** | 0.953 | 0.905 | **0.441 m** | **0.325 m/s** |
| S1h E32 / score0.40 | 0.894 | 0.797 | 0.508 m | **0.307 m/s** | **0.962** | **0.925** | 0.456 m | 0.366 m/s |
| B3S-PP0 E39 / score0.10 | 0.935 | 0.871 | 0.541 m | 0.435 m/s | 0.951 | 0.900 | 0.433 m | 0.326 m/s |

## Map-CV Prediction.

각 셀은 `1초 / 2초 / 3초 minFDE-K`임. p50은 대표 정확도, p90은 tail
진단값임.

| 모델 | 로터리 p50 | 로터리 p90 | 고주로 p50 | 고주로 p90 |
|---|---:|---:|---:|---:|
| Legacy E65 | 0.640 / 0.976 / 1.367 m | 2.441 / 4.202 / 6.308 m | 0.576 / 0.876 / 1.300 m | 1.359 / 2.376 / 3.022 m |
| Team B0 E80 / 1S | — | — | — | — |
| Team B0 E80 / 3S | — | — | — | — |
| T0 E35 | 1.009 / 1.589 / 2.283 m | 2.637 / 4.745 / 6.486 m | — | — |
| S0 E28 | 0.574 / 0.992 / 1.314 m | 2.686 / 4.640 / 6.324 m | — | — |
| **S1 E38** | **0.577 / 0.826 / 1.174 m** | **1.900 / 3.287 / 4.556 m** | **0.463 / 0.718 / 1.056 m** | **1.213 / 2.060 / 2.984 m** |
| S1b E30 | 0.639 / 1.015 / 1.404 m | 2.560 / 3.951 / 5.640 m | 0.507 / 0.827 / 1.212 m | 1.496 / 2.510 / 3.418 m |
| S1g E37 / score0.10 | 0.488 / 0.563 / 0.861 m | 1.415 / 2.324 / 3.595 m | 0.421 / 0.580 / 0.761 m | 0.918 / 1.634 / 2.394 m |
| **S1g E37 / score0.40** | **0.464 / 0.727 / 1.096 m** | **1.344 / 1.930 / 2.875 m** | 0.491 / **0.683 / 0.956 m** | **0.984 / 1.628 / 2.410 m** |
| S1h E32 / score0.40 | 0.488 / 0.750 / **1.073 m** | **1.315** / 2.081 / 3.054 m | **0.466** / 0.704 / 0.979 m | 1.108 / 1.819 / 2.664 m |
| B3S-PP0 E39 / score0.10 | 0.614 / 1.172 / 1.660 m | 1.897 / 3.283 / 4.564 m | 0.432 / 0.614 / 0.819 m | 1.007 / 1.669 / 2.221 m |

## 현재 판정과 미평가 backlog.

- 고정 score `0.10`만 비교하면 S1g는 고주로 confidence calibration 실패로
  Tracking F1이 `0.862`까지 하락하므로 S1 E38을 대체할 수 없음.
- Base2 Val에서 미리 정한 score `0.40`을 사용하면 S1g가 로터리 OD·Tracking과
  두 장면의 Map-CV tail을 개선하고, 고주로 Tracking F1은 S1과 동률로 회복함.
- Vehicle-only bag의 잘못된 다른 class 출력도 감소함.
  - 로터리 Pedestrian/Obstacle FP는 S1의 `1/151`에서 S1g score0.40의 `0/3`으로 감소함.
  - 고주로 Pedestrian/Obstacle FP는 `25/184`에서 `2/79`로 감소함.
- Vehicle-only 후보는 `S1g E37 / score0.40`임.
- 후속 class별 검증에서 Pedestrian recall과 Obstacle FP를 global score 하나로
  동시에 만족하지 못했으므로 전체 3-class production 승격은 보류함.
  - 현재 default는 S1을 유지함.
  - 상세 결과는 `s1g_class_production_validation.md`에 기록함.
- S1h는 Obstacle sampling `10 -> 3`으로 고주로 Obstacle FP를 `79 -> 21`로
  줄였지만 다음 손실이 발생해 기각함.
  - Pedestrian crossing/distance mAP `0.442/0.642 -> 0.381/0.444`.
  - 로터리 Tracking F1 `0.960 -> 0.894`.
  - 로터리 3초 p90 `2.875 -> 3.054 m`.
- 따라서 S1g의 Obstacle sampling을 단순히 줄이는 방향은 후속 production
  calibration 전략으로 사용하지 않음.
- S1b는 offline validation에서 S1과 동률이지만 두 rosbag에서 하락해 기각함.
- Team B0 E80 두 입력 방식은 Detection bag만 존재함. 동일 tracker·predictor로
  pipeline output을 생성해야 Tracking·Map-CV 행을 채울 수 있음.
- T0 E35는 고주로1 OD와 downstream 평가가 없음.
- S0 E28은 고주로1 downstream 평가가 없음.
- 새 checkpoint를 추가할 때 두 bag의 OD를 모두 완료한 뒤 downstream 평가를
  수행함. 한 scene만 평가한 결과로 production 순위를 정하지 않음.

## 새 모델 행 템플릿.

```markdown
| MODEL E## | — | — | — | — | — | — | — | — | — | — |
```

- 먼저 모델 설명 표에 입력 sweep, velocity head, 증강, 학습 데이터와 checkpoint를
  기록함.
- 같은 모델명을 OD, Tracking, Map-CV 세 표에서 동일하게 사용함.
- 평가 설정을 바꿨다면 기존 행을 덮어쓰지 않고 `/ score0.XX`, `/ 3S`처럼 별도
  행으로 추가함.
