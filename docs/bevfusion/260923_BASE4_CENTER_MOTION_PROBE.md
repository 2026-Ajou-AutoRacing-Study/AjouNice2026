# Base4 predicted-center motion probe

> 날짜: 2026-09-23.
> 상태: 기존 고주로 detection bag 재사용 분석 및 실제 EKF/Map-CV Q0 A/B 완료.
> 재추론 없음.

## 결론

현재 Base4 velocity 실패의 가장 큰 원인은 association이 아니라 **velocity head가 이미
존재하는 frame 간 center displacement를 사용하지 못하는 것**이다. 기존 detection center를
causal하게 연결하기만 해도 speed MAE가 head의 `2.53--4.23 m/s`에서 baseline에 따라
`0.17--0.71 m/s`까지 내려갔다. Production E25의 center-motion도 같은 효과를 보였으므로
velocity head가 있는 Base4 모델로 detector를 먼저 교체할 이유도 없다.

따라서 다음 우선순위는 대규모 temporal detector를 바로 학습하는 것이 아니다.

1. causal center-motion `B4H-Q0`를 optional EKF measurement로 구현했다.
2. 기존 두 development bag의 end-to-end gate는 통과하지 못했다.
3. 새 다중차량·차선변경 held-out bag에서 quality gate를 설계·검증한다.
4. Q0 단독으로 부족하면 learned query memory `B4H-Q1`과 병행한다.

Production detector와 tracker 기본값은 아직 바꾸지 않는다.

## 실제 EKF·Map-CV Q0 결과

Production E25의 저장 detection을 재사용하여 같은 CTRV EKF와 Map-CV를 1배속으로
다시 실행했다. Q0는 Vehicle의 map-frame raw detection center만 사용하고, association
gap이 `0.25 s`를 넘으면 history를 초기화한다. `<0.3 s`는 position-only,
`0.45/0.65/0.90 s` baseline과 `1.5 -> 0.6 m/s` noise schedule을 사용했다.

| Case (2-bag macro) | F1 | Center p50 m | Speed MAE m/s | 2D AVE m/s | Lat MAE m/s | Top-1 FDE 1/2/3 s m |
|---|---:|---:|---:|---:|---:|---:|
| E25 position-only | 0.9341 | **0.615** | **0.645** | 0.964 | 0.574 | **1.224 / 2.640 / 4.663** |
| Q0 every detection | **0.9350** | 0.628 | 0.680 | **0.960** | **0.518** | 1.366 / 2.825 / 4.870 |
| Q0, minimum update interval 0.20 s | 0.9344 | 0.622 | 0.703 | 0.985 | 0.534 | 1.356 / 2.846 / 4.925 |

Q0 every-detection은 횡속도와 2D AVE를 아주 조금 개선했지만 speed와 Map-CV FDE를
악화했다. 겹치는 displacement window를 독립 관측처럼 반복 융합하지 않도록 `0.20 s`
update interval도 적용했으나 더 좋아지지 않았다. 즉 center-motion 관측의 pairwise
정확도와 downstream EKF 효용은 같은 의미가 아니다. startup/miss를 포함한 전체 track,
CTRV state와의 상관, longitudinal bias를 함께 다뤄야 한다.

구현은 다음 위치에 optional 연구 기능으로 남긴다.

- `ekf-multi-object-tracking`: raw associated-center history와 adaptive velocity covariance.
- `tracker_center_motion_velocity_fusion:=true`: 연구용 launch gate.
- production/default: `false`.

현재 구현의 `0.20 s` interval은 상관 관측 과신을 막는 안전장치이지, 성능이 검증된
production tuning이 아니다. 다음 A/B는 단순 noise sweep보다 innovation/residual,
연속 관측 수, track age를 함께 쓰는 accept/reject quality gate를 우선한다.

유효 end-to-end 산출물:

- `artifacts/evaluation/bevfusion/q0_center_motion_260923_v4/`: every-detection Q0.
- `artifacts/evaluation/bevfusion/q0_center_motion_interval020_260923/`: 0.20초 gate Q0.

## 질문과 분해 방식

같은 저장 detection을 다음 네 단계로 비교했다.

1. **GT center + GT identity**: 동일 timestamp pair의 정확한 geometric-center displacement.
   모든 pairwise estimate의 0-error reference다. 별도 annotation audit에서 이 값과 velocity
   label의 3-frame MAE는 `0.033 m/s`였다.
2. **Predicted center + GT identity**: frame별 detection을 GT identity로 묶는다. Detector
   center noise가 만드는 속도 오차다.
3. **Predicted center + causal association**: GT를 보지 않고 이전 detection center와
   constant-velocity Hungarian association만 사용한다. 2번 대비 증가분이 association
   비용이다.
4. **Velocity head**: 같은 GT-matched pair에 기록된 detector `vx,vy`를 1번 reference와
   비교한다.

모든 detection은 Vehicle score `0.40`, GT center match `1 m`, range `30 m` 계약을 썼다.
GT는 asset별 geometric BBox center 보정을 적용했다. Center-motion estimate에는 미래
frame을 사용하지 않으며 current timestamp 기준 과거만 사용한다.

## 0.45초 baseline 결과

| 모델 | bag | pair | Pred center + GT ID speed MAE | Causal association speed MAE | Causal vector error | Head speed MAE |
|---|---|---:|---:|---:|---:|---:|
| Production E25 | highway1 | 366 | 0.283 | **0.287** | 0.467 | N/A |
| Production E25 | merge | 195 | 0.565 | **0.638** | 0.701 | N/A |
| V1 E2 | highway1 | 328 | 0.345 | **0.383** | 0.561 | 2.544 |
| V1 E2 | merge | 219 | 0.578 | **0.670** | 0.737 | 4.230 |
| T1 E6 | highway1 | 346 | 0.491 | **0.515** | 0.623 | 2.533 |
| T1 E6 | merge | 226 | 0.703 | **0.708** | 0.776 | 3.439 |

두 bag macro 기준으로 causal center-motion speed MAE는 Production E25
`0.463 m/s`, V1 `0.526 m/s`, T1 `0.612 m/s`다. 같은 pair에서 V1/T1 velocity
head는 각각 약 `3.39`, `2.99 m/s`다.

Association이 추가한 speed MAE는 V1에서 `0.038/0.092 m/s`, T1에서
`0.024/0.004 m/s`뿐이었다. Known endpoint identity pair의 switch는 네 조합 모두 0건이다.
다만 이 두 bag은 차량 교차와 밀집도가 낮으므로 association 일반화가 해결됐다는 뜻은
아니다.

## Baseline 길이 A/B

아래 값은 `causal association speed MAE`다.

| 모델 / bag | 0.20초 | 0.45초 | 0.65초 | 0.90초 |
|---|---:|---:|---:|---:|
| Production E25 / highway1 | 0.451 | 0.287 | 0.220 | **0.168** |
| Production E25 / merge | 0.899 | 0.638 | 0.564 | **0.488** |
| V1 / highway1 | 0.617 | 0.383 | 0.295 | **0.218** |
| V1 / merge | 1.099 | 0.670 | 0.541 | **0.452** |
| T1 / highway1 | 0.804 | 0.515 | 0.367 | **0.282** |
| T1 / merge | 1.408 | 0.708 | 0.574 | **0.461** |

Vector error도 같은 방향으로 줄었다. 0.90초에서 Production E25는 highway1/merge
`0.263/0.520 m/s`, V1은 `0.315/0.478 m/s`, T1은 `0.345/0.489 m/s`다.

이 결과는 center noise가 짧은 dt에서 속도로 증폭된다는 가설을 직접 지지한다. 0.90초가
가장 정확하지만 새 track의 velocity 확보가 너무 늦다. 첫 runtime 후보는 다음과 같이
단일 고정 baseline보다 multi-age 정책을 사용한다.

- track age `<0.3 s`: detector velocity를 신뢰하지 않고 position-only.
- `0.3--0.6 s`: 0.45초 중심 estimate, 큰 covariance.
- `0.6--0.9 s`: 0.65초 또는 전체 history robust fit, 중간 covariance.
- `>=0.9 s`: 0.9초 history estimate, 낮은 covariance.
- association gap, teleport, 큰 innovation에서는 history reset.

## 모델 비교에서 드러난 추가 사실

Production E25의 center-motion은 두 bag macro에서 V1/T1보다 가장 좋았다. T1의 velocity
head는 V1보다 merge bias를 줄였지만, center displacement 품질 자체는 네
baseline 모두 V1보다 나빴다. 단순 3-sweep early concatenation이 temporal center를 더
안정화하지 못했고 일부 ghosting/noise를 추가했다는 기존 판단과 일치한다.

반대로 V1의 단일-frame center도 시간축에서 연결하면 0.5 m/s 근처의 velocity observation을
만들 수 있다. 즉 다음 개선은 반드시 detector 내부에 있어야 하는 것이 아니다. 안정적인
Q0가 downstream gate를 통과하면 production E25 공간 detector를 유지하면서 tracker 앞단의
motion observation adapter만 추가하는 쪽이 더 단순하고 안전할 수 있다.

## 남은 제한과 다음 검증

- 두 bag은 반복 사용된 development bag이며 final test가 아니다.
- GT identity reconstruction과 per-frame 1 m GT match는 평가에만 사용했고 runtime
  association에는 사용하지 않았다.
- Identity switch 0건은 쉬운 교통 밀도의 결과일 수 있다.
- Detection miss/재등장, 인접 차선 접근, 추월 중 교차와 30 m 경계에서는 별도 검증이
  필요하다.
- Speed magnitude뿐 아니라 tracker 2D AVE, center, track continuity와 Map-CV FDE를
  같은 Q0 출력에서 평가해야 한다.

재현 도구:

`base_ws/src/tracking_prediction_evaluation/scripts/analyze_detection_center_motion_probe.py`

단위 테스트:

`base_ws/src/tracking_prediction_evaluation/test/test_detection_center_motion_probe.py`

산출물은 각 기존 평가 폴더의 `center_motion_probe*.json`에 저장했다.
