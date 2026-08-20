# PointPainting GT velocity oracle ablation (2026-08-20)

## 질문

현재 PointPainting은 box의 위치·yaw를 발행하지만 velocity head는 없다. 이때
정확한 종방향 속도를 detector measurement로 추가하면 EKF tracking과 Map-CV
prediction이 개선될 수 있는지, 재학습 전에 상한을 확인한다.

## 실험 원칙

- PointPainting의 위치, yaw, dimensions, class, confidence, UUID는 바꾸지 않는다.
- `/Object_topic.velocity`는 bag마다 단위가 달라 직접 사용하지 않는다.
- BBox-center로 보정된 MORAI GT의 과거 0.5초 위치에 causal polynomial fit을
  적용하여 현재 종방향 속도를 복원한다.
- GT와 2 m 이내로 매칭된 tracked vehicle의 speed만 교체한다.
- 미래 GT 위치는 현재 속도 산출에 사용하지 않는다.
- 이는 production이 아니라 velocity measurement의 **oracle upper bound**이다.

비교 모드는 다음과 같다.

- `recorded_runtime`: 기존 output bag에 기록된 tracking/prediction.
- `baseline`: 기존 tracked speed로 동일 Map-CV를 오프라인 재생성.
- `oracle_initial_speed_hold`: track 최초 매칭 시 GT speed를 넣고 계속 유지.
- `oracle_velocity_update`: 매 detection frame마다 현재 GT speed를 다시 주입.

`recorded_runtime`과 `baseline` 결과는 두 장면에서 동일했다. 따라서 아래 개선은
오프라인 prediction 재생성 방식의 차이가 아니라 speed 교체 효과이다.

## CV 안정 구간 결과

### 로터리 1

| 모드 | Tracking speed error p50 | 1 s minFDE-K p50 | 2 s minFDE-K p50 | 3 s minFDE-K p50 |
| --- | ---: | ---: | ---: | ---: |
| Baseline | 0.453 m/s | 0.726 m | 1.136 m | 1.527 m |
| Initial speed hold | 0.061 m/s | 0.593 m | 0.573 m | 0.550 m |
| Velocity update | **0.000 m/s** | **0.621 m** | **0.597 m** | **0.584 m** |

로터리 차량은 관측 이후 속도가 비교적 안정적이라 initial hold도 잘 작동했다.
다만 이는 장면 특성에 의존하며 일반 production 방식으로 사용할 수 없다.

### 고주로 1

| 모드 | Tracking speed error p50 | 1 s minFDE-K p50 | 2 s minFDE-K p50 | 3 s minFDE-K p50 |
| --- | ---: | ---: | ---: | ---: |
| Baseline | 0.440 m/s | 0.631 m | 0.893 m | 1.242 m |
| Initial speed hold | 2.264 m/s | 0.648 m | 0.574 m | 0.628 m |
| Velocity update | **0.000 m/s** | **0.367 m** | **0.409 m** | **0.490 m** |

Initial hold의 p50 FDE만 보면 일부 horizon에서 작아 보이지만, speed p90 오차가
5.604 m/s이고 3초 FDE mean/p90은 4.122/9.671 m로 크게 악화했다. 처음 관측된
차량이 가속 중일 수 있으므로 최초 속도를 계속 유지하는 방식은 안전하지 않다.

## 가속 구간을 포함한 결과

고주로 전체 구간에서 velocity update의 3초 minFDE-K는 다음처럼 바뀌었다.

- mean: `4.090 -> 2.907 m`.
- p50: `2.240 -> 1.143 m`.
- p90: `10.946 -> 8.131 m`.

현재 속도를 완벽히 알아도 미래 가속·감속 자체를 미리 알 수 없으므로 tail은
남는다. 즉 velocity head는 큰 개선 수단이지만 미래 motion model을 대체하지는
않는다.

## 결론

- **Velocity를 최초 한 번만 초기화하는 방식은 채택하지 않는다.**
- **Vehicle velocity measurement를 매 detection update에 EKF로 넣는 경로는
  구현 가치가 높다.**
- 단일-frame PointPainting velocity head가 oracle에 도달할 수는 없으므로 실제
  구현 후에는 predicted velocity의 calibration과 covariance가 필요하다.
- 다음 단계는 optional velocity measurement를 표준 `DetectedObjects`에 담고,
  EKF가 `has_twist`와 covariance가 유효할 때만 update하도록 만드는 것이다.
- 그 뒤 현재 position-only EKF와 position+predicted-velocity EKF를 같은 bag에서
  재평가한다.

## 실제 EKF velocity measurement replay

위 오프라인 상한을 확인한 뒤 EKF에 optional velocity measurement 경로를
구현했다. 기본값은 `detection_velocity_fusion_mode=0`이므로 기존 production은
변하지 않는다.

- mode 0: detector velocity를 무시한다.
- mode 1: 새 track의 velocity 초기화에만 사용한다.
- mode 2: 초기화와 매 detection update에 사용한다.
- `DetectedObjectKinematics.has_twist=true`인 객체만 fusion한다.
- body-frame detector velocity를 map-frame `VX/VY`로 변환한 뒤 2D Kalman
  measurement update를 수행한다.

GT velocity를 넣은 detection bag으로 mode 2 EKF와 실제 Map-CV node를 다시
실행했다. GT와 매칭되지 않은 detection에는 velocity를 주입하지 않았다.

### CV 안정 구간 실제 EKF 결과

| 장면 | 모드 | Speed error p50/p90 | 1 s minFDE-K p50 | 2 s minFDE-K p50 | 3 s minFDE-K p50 |
| --- | --- | ---: | ---: | ---: | ---: |
| 로터리 1 | Position-only baseline | 0.453 / 1.313 m/s | 0.726 m | 1.136 m | 1.527 m |
| 로터리 1 | GT velocity update EKF | **0.040 / 0.126 m/s** | 0.727 m | **0.778 m** | **0.822 m** |
| 고주로 1 | Position-only baseline | 0.440 / 1.339 m/s | 0.631 m | 0.893 m | 1.242 m |
| 고주로 1 | GT velocity update EKF | **0.006 / 0.267 m/s** | **0.322 m** | **0.387 m** | **0.493 m** |

로터리 1초 p50은 거의 그대로다. 짧은 horizon에서는 현재 center 오차와
Lanelet 정합이 지배적이기 때문이다. 반면 속도 오차가 시간에 따라 누적되는
2~3초에서는 명확하게 개선됐다. 고주로는 1초부터 속도 오차 영향이 커서 모든
horizon이 개선됐다.

Tracking center p50은 로터리에서 `0.733 -> 0.789 m`로 소폭 악화했고,
고주로에서 `0.536 -> 0.519 m`로 소폭 개선했다. 매우 작은 oracle velocity
noise(`0.05 m/s`)로 강하게 update하면 velocity-position covariance를 통해
position에도 영향이 생긴다. 실제 velocity head 적용 시에는 validation으로
measurement covariance를 calibration해야 한다.

### 최종 판단

- 이 실험은 **vehicle velocity head 또는 별도 velocity estimator를 구현할
  가치가 높다**는 결론을 지지한다.
- 속도를 한 번만 초기화해서는 부족하다. 매 detection마다 update해야 한다.
- 단, 실제 single-frame velocity는 GT처럼 완벽하지 않으므로 head 구현 후
  noise sweep과 position/association 회귀 검증이 필수다.
- 현재 production mode는 계속 0으로 유지한다. 학습되지 않은 0 velocity를
  `has_twist=true`로 보내면 안 된다.

재현 스크립트:

`base_ws/src/tracking_prediction_evaluation/scripts/evaluate_velocity_oracle_ablation.py`

Oracle detection bag 생성:

`base_ws/src/tracking_prediction_evaluation/scripts/create_gt_velocity_oracle_detections.py`
