# PP1 rosbag LiDAR visibility 재평가

## 목적

- 기존 online 평가는 30 m 안의 모든 `/Object_topic` GT를 사용했다.
- Base3 Train/Val/Test는 원 box에 0.03 m point-count tolerance를 적용하고 target
  LiDAR sweep에서 5 points 이상인 GT만 normal positive로 사용했다.
- 두 계약을 분리해 PP1의 전체 system 성능과 Base3-compatible observable OD 성능을
  함께 기록한다.

## 고정 조건

- Model: `B3S-PP1-GTDB-V20P10O4 E34`.
- Class threshold: Vehicle `0.40`, Pedestrian `0.15`, Obstacle `0.10`.
- Center matching: 1 m.
- Evaluation range: 30 m.
- LiDAR evidence: detection header와 가장 가까운 raw LiDAR sweep을 사용하고, point
  count는 해당 LiDAR 시각으로 Ego와 GT trajectory를 보간해 계산한다.
- Invalid box: length, width, height 중 하나라도 0 이하인 GT.
- Observable: valid box이며 5 points 이상.
- Observable precision/AP에서는 1 m 안에서 제외 GT만 더 가까운 prediction을 ignore한다.
  제외 GT를 검출한 결과가 false positive가 되는 것을 막기 위한 slice 평가 규칙이다.
- 이 평가의 AP는 이미 class threshold로 filter된 deployment detection bag에서 계산한
  truncated AP다. Full score-ranking AP와 혼용하지 않는다.

## 결과

| Scene | Target | Global GT | 0 pt | 1--4 pt | 5+ pt | Invalid | Global AP | Global R/F1 @1m | Observable AP | Observable P/R/F1 @1m | Global/Observable mATE |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 장애물 왕복 | Obstacle | 683 | 618 | 13 | 52 | 0 | 0.2970 | 44.7% / 58.9% | 0.3823 | 44.1% / 71.2% / 54.4% | 0.563 / 0.532 m |
| 신로터리 | Vehicle | 316 | 0 | 4 | 312 | 0 | 0.7278 | 87.0% / 91.7% | 0.7378 | 96.8% / 88.1% / 92.3% | 0.464 / 0.464 m |
| 고주로 합류 | Vehicle | 274 | 10 | 7 | 257 | 0 | 0.6154 | 79.9% / 83.0% | 0.6404 | 85.9% / 82.9% / 84.4% | 0.522 / 0.522 m |
| 고주로1 | Vehicle | 402 | 2 | 1 | 399 | 0 | 0.7703 | 91.8% / 94.3% | 0.7730 | 96.8% / 92.2% / 94.5% | 0.458 / 0.455 m |
| Obstacle3 stress | Obstacle | 1,301 | 433 | 157 | 585 | 126 | 0.0000 | 6.5% / 11.9% | 0.0160 | 72.0% / 11.5% / 19.8% | 0.660 / 0.679 m |

## 해석

- Vehicle bag은 GT의 94--99%가 5+ points다. 기존 global 평가가 visibility 때문에
  크게 왜곡된 것은 아니다.
- 고주로 합류의 observable recall은 `79.9% -> 82.9%`로 가장 크게 회복하지만 모델
  순위나 production 판단을 뒤집을 수준은 아니다.
- 왕복 CargoBox는 높이가 0.43 m라 VLP16의 수직 beam 사이로 빠져 exact GT box 안에
  point가 없는 sweep이 많다. PP1은 주변 pillar와 semantic 문맥으로 이런 frame도
  상당수 검출하므로 0 point를 곧바로 검출 불가능으로 해석하지 않는다.
- Obstacle3는 invalid/sparse GT를 제거해도 recall이 `11.5%`에 그친다. 전체 6.5%가
  visibility 계약 때문에 낮아진 것은 맞지만, 다중 obstacle 형상과 online domain에
  대한 실제 일반화 약점도 남는다.
- Production inference에는 point-count gate를 넣지 않는다. 이 gate는 학습 label 품질과
  offline diagnostic slice를 위한 것이다.

## 산출물

- Evaluator:
  `base_ws/src/tracking_prediction_evaluation/scripts/evaluate_rosbag_detection_visibility.py`
- Per-bag reports:
  `artifacts/evaluation/pointpainting/260912_pp1_visibility_reaudit_v2/*.json`
