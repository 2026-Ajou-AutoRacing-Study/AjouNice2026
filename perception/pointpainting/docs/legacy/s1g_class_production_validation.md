# S1g class별 production 검증과 gain slice 분석.

## 목적.

- `S1g E37`이 Vehicle뿐 아니라 Pedestrian·Obstacle에서도 production 후보인지
  검증함.
- 전체 mAP 상승을 다음 두 효과로 분리함.
  - 실제 검출 ranking·표현력 향상.
  - score threshold와 confidence calibration 변화.
- Vehicle gain이 어느 거리·방향·상호작용 slice에서 발생했는지 확인함.

## 비교 조건.

- 기준 모델은 `S1 E38`임.
- 실험 모델은 `S1g E37`임.
  - S1과 같은 single-frame 9-feature 및 geometric augmentation을 사용함.
  - Pedestrian·Vehicle·Obstacle GT database sample을 frame당 각각 최대 10개
    추가함.
- Detection은 score `0.01`부터 저장한 동일 online output을 threshold별로 다시
  평가함.
- GT BBox center calibration, 최대 거리 30 m, center AP 조건은 Vehicle
  leaderboard와 같음.
- Pedestrian·Obstacle bag은 2026-07에 취득한 구 sensor 조건임.
  - S1과 S1g의 상대 비교에는 사용할 수 있음.
  - 최종 대회 production threshold 확정에는 새 competition-version bag이 필요함.

## Offline Val 변화 참고.

| Class | S1 run max mean AP | S1g run E40 mean AP | 변화 |
|---|---:|---:|---:|
| Vehicle | 0.9829 | 0.9960 | +0.0131 |
| Pedestrian | 0.9956 | 0.9944 | -0.0012 |
| Obstacle | 0.8565 | 0.8941 | +0.0376 |
| 3-class center mAP | 0.9417 | 0.9615 | +0.0199 |

- 이 표의 class 값은 checkpoint별 완전 동시 snapshot 비교가 아니라 run trend를
  확인하기 위한 참고값임.
  - 실제 online 비교 checkpoint는 S1 E38과 S1g E37로 고정함.
  - S1g E37의 checkpoint 선택 지표인 3-class center mAP는 `0.9618`임.
- Offline gain trend는 주로 Obstacle과 Vehicle에서 발생함.
- Pedestrian Val은 이미 포화 상태여서 수치상 gain이 없음.
- 그런데 online Pedestrian은 S1g에서 크게 개선됐으므로 GT sampling이 Val
  점수보다 domain generalization에 더 크게 기여한 것으로 해석함.

## Pedestrian production 검증.

### Positive bag.

| Bag | 모델 | center mAP | score | Precision@1m | Recall@1m | F1@1m |
|---|---|---:|---:|---:|---:|---:|
| Crossing, 10-20 m | S1 | 0.000 | 0.10 | 1.000 | 0.011 | 0.022 |
| Crossing, 10-20 m | S1g | **0.442** | 0.10 | 0.959 | **0.516** | **0.671** |
| Crossing, 10-20 m | S1g | 0.442 | 0.40 | 0.973 | 0.264 | 0.415 |
| Distance, 5-18 m | S1 | 0.000 | 0.10 | 1.000 | 0.004 | 0.009 |
| Distance, 5-18 m | S1g | **0.642** | 0.10 | 1.000 | **0.696** | **0.821** |
| Distance, 5-18 m | S1g | 0.642 | 0.40 | 1.000 | 0.271 | 0.427 |

- S1은 두 bag에서 Pedestrian을 사실상 검출하지 못함.
- S1g의 Pedestrian 표현력 개선은 매우 큼.
- 그러나 Vehicle-only 고주로 bag에서 Pedestrian FP는 score `0.10/0.20/0.30/0.40`
  순서로 `339/14/3/2`개임.
- Pedestrian 전용 negative bag에서는 score 0.10에서도 Pedestrian FP가 0개였음.
  - 즉 단순 배경 오검출이 아니라 Vehicle 또는 복잡한 고주로 장면과의 class
    confusion이 남아 있음.
- score 0.10은 Pedestrian recall에는 좋지만 전체 scene production threshold로는
  불안정함.

## Obstacle production 검증.

| Bag | 모델 | center mAP | best score | best F1@1m | mATE | mASE | mAOE |
|---|---|---:|---:|---:|---:|---:|---:|
| Obstacle3 | S1 | 0.067 | 0.102 | 0.305 | 0.548 m | 0.667 | 1.457 rad |
| Obstacle3 | S1g | **0.157** | 0.221 | **0.336** | 0.705 m | 0.675 | **1.206 rad** |
| Ped-negative obstacle scene | S1 | 0.129 | 약 0.10 | **0.444** | — | — | — |
| Ped-negative obstacle scene | S1g | **0.261** | 0.135 | 0.330 | 0.800 m | 0.872 | 1.615 rad |

- S1g Obstacle AP는 두 bag에서 모두 약 2배 상승함.
- 하지만 실제 operating point는 일관되지 않음.
  - Obstacle3에서는 score 약 0.22가 최적임.
  - 다른 obstacle scene에서는 score 약 0.14가 최적임.
- score 0.22를 Vehicle-only bag에 적용하면 Obstacle FP가 로터리 `291`, 고주로
  `1,837`개 발생함.
- score 0.40은 Vehicle-only FP를 로터리 `3`, 고주로 `79`개까지 줄이지만,
  Obstacle3 recall은 `14.9%`로 하락함.
- S1g는 Obstacle candidate ranking은 개선했지만 duplicate·class confusion과
  calibration은 production 수준이 아님.

## Vehicle gain slice.

### 방향별 Recall@1m.

| Scene | Slice | S1 | S1g | 변화 | S1 miss -> S1g miss |
|---|---|---:|---:|---:|---:|
| 로터리 | Front | 86.9% | 86.0% | -1.0%p | 23 -> 25 |
| 로터리 | Left | 55.7% | **80.3%** | **+24.6%p** | **27 -> 12** |
| 로터리 | Rear | 96.2% | 96.2% | 0.0%p | 3 -> 3 |
| 고주로 | Front | 96.0% | **98.4%** | +2.4%p | 5 -> 2 |
| 고주로 | Left | 89.8% | **96.0%** | +6.2%p | 5 -> 2 |
| 고주로 | Right | 65.5% | 65.5% | 0.0%p | 10 -> 10 |
| 고주로 | Rear | **98.0%** | 97.0% | -1.0%p | 4 -> 6 |

### Planning relevance와 거리별 Recall@1m.

| Scene | Slice | S1 | S1g | 변화 |
|---|---|---:|---:|---:|
| 로터리 | Conflict-critical | 89.2% | 90.5% | +1.3%p |
| 로터리 | Planning-relevant | 67.6% | **94.3%** | **+26.6%p** |
| 로터리 | 10-15 m | 85.6% | **94.1%** | +8.5%p |
| 로터리 | 15-20 m | 84.8% | 89.0% | +4.2%p |
| 로터리 | 20 m+ | 78.6% | 77.6% | -1.0%p |
| 고주로 | Conflict-critical | 97.7% | 97.3% | -0.5%p |
| 고주로 | Planning-relevant | 88.2% | **91.5%** | +3.3%p |
| 고주로 | Approaching | 91.6% | **96.8%** | +5.3%p |
| 고주로 | 10-15 m | 89.0% | **92.1%** | +3.1%p |
| 고주로 | 20 m+ | **94.0%** | 91.2% | -2.9%p |

- 가장 큰 gain은 로터리 좌측·planning-relevant Vehicle임.
  - Base2에서 부족하다고 진단했던 로터리 좌측 positive slice를 GT sampling이
    실제로 보강한 결과와 일치함.
- 고주로 gain은 Front·Left·Approaching·planning-relevant에 집중됨.
- 고주로 Right와 20 m+는 개선되지 않음.
- gain은 모든 방향을 고르게 올린 결과가 아니라, 학습에서 부족했던 객체-positive
  slice의 recall을 올린 결과임.

## 판정.

- `S1g E37`은 representation checkpoint로는 S1보다 우수함.
  - Vehicle 핵심 slice recall이 개선됨.
  - Pedestrian online AP가 0에서 의미 있는 수준으로 회복됨.
  - Obstacle ranking AP도 약 2배 상승함.
- 그러나 현재 `global score 0.40`을 적용한 3-class production 승격은 보류함.
  - Vehicle에는 적합함.
  - Pedestrian recall을 과도하게 자름.
  - Obstacle은 scene별 최적 threshold 차이와 FP 문제가 큼.
- 현재 production default는 S1을 유지함.
- S1g는 다음 class-calibration 실험의 기준 checkpoint로 유지함.

## 다음 실험.

1. 최신 competition-version 센서 설정으로 Pedestrian crossing과 static obstacle
   mission bag을 새로 취득함.
2. class별 score threshold를 지원하는 experimental launch를 만들고 다음 범위를
   고정 sweep함.
   - Vehicle `0.35-0.45`.
   - Pedestrian `0.10-0.35`.
   - Obstacle `0.15-0.45`.
3. 각 positive mission bag과 Vehicle/background negative bag을 함께 사용해
   class별 Precision·Recall·FP/frame을 동시에 최적화함.
4. threshold만으로 Obstacle FP를 해결하지 못하면 다음 S1 계열에서 GT sampling
   수량·collision filtering·sampling context를 조정함.
5. 로터리 좌측 recall이 이미 `80.3%`로 개선됐으므로 기존 Vehicle targeted
   add-on의 우선순위는 낮춤.
6. 개선되지 않은 고주로 Right는 실제 planning 중요도가 확인될 때만 targeted
   add-on을 수집함.

## S1h 후속 결과.

- Obstacle GT sampling만 `10 -> 3`으로 줄인 S1h E32를 같은 조건으로 평가함.
- 고주로 score 0.40 Obstacle FP는 `79 -> 21`로 감소함.
- Obstacle3 mAP/best F1은 `0.157/0.336 -> 0.169/0.414`로 개선함.
- 반면 다음 성능은 하락함.
  - Pedestrian crossing mAP `0.442 -> 0.381`.
  - Pedestrian distance mAP `0.642 -> 0.444`.
  - 로터리 Vehicle mAP `0.720 -> 0.683`.
  - 로터리 Tracking F1 `0.960 -> 0.894`.
- Obstacle sampling 감소는 FP를 줄였지만 S1g의 전체 representation gain을
  유지하지 못했으므로 production 후보로 기각함.
- 다음 단계에서는 sampling 수량을 다시 바꾸기보다 S1g에 class별 calibration과
  class-specific post-processing을 적용하는 것이 우선임.
