# E25와 Base4 Q1 P0--P9 production 영향 감사

> 날짜: 2026-09-25
> 상태: **CURRENT DECISION / EVIDENCE**
> 기준 브랜치: `perception/static-obstacle-tracking-diagnosis-260924`
> 목적: P0--P9 연구 코드가 존재한다는 사실과 실제 E25 production 동작을 분리한다.

## 한 줄 결론

P0--P9는 Base4 temporal detector와 detector velocity 활용 가능성을 검증한 연구이며,
**현재 E25 production 출력에는 적용하지 않는다.** E25 checkpoint는 계속 7D
position-only detector이고 tracker도 velocity fusion mode 0을 사용한다.

다만 P0--P9 이전에 완료한 tracker 측정 노이즈 조정 등 검증된 후단 개선은 E25
production에 반영되어 있다. 따라서 현재 production은 단순한 `E25 checkpoint + 옛날
tracker`는 아니지만, P0--P9 temporal velocity가 들어간 E25도 아니다.

## 세 층의 계약

### 1. Detection checkpoint

- 표준 모델: `B3-BF0-F40 E25`.
- 출력: 7D box 중심·크기·yaw와 class score.
- class threshold: Vehicle `0.40`, Pedestrian `0.15`, Obstacle `0.10`.
- detector velocity head와 learned temporal query memory는 사용하지 않는다.
- P0--P9 실험 때문에 E25 weight나 detection 결과가 바뀌지 않았다.

### 2. 현재 production 후단

```text
E25 7D detection
  -> CTRV EKF tracker, position-only mode 0
  -> UNKNOWN stationary policy
  -> Lanelet2 Map-CV
  -> /perception/object_recognition/objects
```

현재 실제 적용값은 다음과 같다.

| 항목 | Production 값 | E25에 미치는 영향 |
|---|---|---|
| Tracker motion model | CTRV | position history에서 motion 추정 |
| Position measurement noise | `0.40 m` | 검증 후 채택된 실제 개선 |
| Detector velocity fusion | mode `0`, off | E25 7D 계약 유지 |
| Center-motion fusion | off | Q0 연구 출력 미사용 |
| Maximum internal prediction age | 운영 alias `0.50 s` | 간헐적 detector 지연에서 즉시 폐기 방지 |
| Vehicle yaw canonicalization | `1.0 m/s` 이상, `100/80 deg` hysteresis | 운동방향 기준 yaw flip 완화 |
| UNKNOWN motion policy | stationary 강제 | 가짜 속도와 가짜 이동 궤적 억제 |
| Prediction | Lanelet2 Map-CV, `3.0 s / 0.1 s` | 현재 production prediction |

UNKNOWN stationary와 Map-CV는 현재 파이프라인의 구성 요소지만 P0--P9 temporal
detector 성과를 E25에 이식한 것은 아니다. 각각 별도 production 정책이다.

### 3. P0--P9 연구 옵션

다음 구현은 소스에 존재하더라도 기본값이 꺼져 있거나 별도 Base4 checkpoint/config가
필요하다. 소스에 존재한다는 이유만으로 E25 production에 적용됐다고 해석하지 않는다.

| 단계 | 연구 내용 | E25 production 영향 |
|---|---|---|
| P0 | ego-warped sparse query memory | 없음 |
| P1 | displacement/dt value와 auxiliary velocity | 없음 |
| P2 | post-decoder center 계약 통일 | 없음 |
| P3 | 다중 causal pair motion aggregation | 없음 |
| P4 | mutual matching과 hard consensus | 기각, 없음 |
| P5 | robust 후보 soft blend | 기각, 없음 |
| P6 | `dt^2` age-weighted slope | Base4 detector 연구에만 적용 |
| P7 | free-intercept temporal fit | Base4 detector 연구 baseline, E25에는 없음 |
| P8 | detector velocity LiDAR→map 회전, 초기 종방향 mode 4 | 코드 수정은 채택, E25는 velocity가 없고 mode 0이므로 출력 변화 없음 |
| P9 | covariance 전달과 uncertainty-aware mode 5 | 연구 옵션, 기본 mode 0이므로 출력 변화 없음 |

P8의 frame 수정은 올바른 계약 수정이다. 향후 9D detector를 사용할 때 반드시 필요하지만,
velocity가 없는 E25에는 회전할 값이 없으므로 현재 출력은 바뀌지 않는다. P9의 covariance
경로도 같은 이유로 E25의 production 결과를 바꾸지 않는다.

## E25 후단에서 실제로 채택된 정량 개선

P0--P9와 별개로 tracker position measurement noise를 `0.30 → 0.40 m`로 조정한 변경은
E25에 실제 적용됐다. 근거 커밋은 `5d50ae44`이고 상세 평가는
[`260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md`](260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md)에
있다.

Production E25 고주로 두 bag의 동일 조건 비교는 다음과 같다.

| 지표 | `Rxy=0.30 m` | `Rxy=0.40 m` | 변화 |
|---|---:|---:|---:|
| Tracking F1 | `0.9324` | `0.9341` | 개선 |
| Speed MAE | `0.655 m/s` | `0.645 m/s` | 개선 |
| Stable FDE@3s | `1.766 m` | `1.696 m` | 개선 |
| All Top-1 FDE@3s | `4.695 m` | `4.663 m` | 개선 |
| Center p50 | `0.599 m` | `0.615 m` | `0.016 m` 악화 |

이는 spatial center를 거의 유지하면서 tracker velocity와 prediction을 소폭 개선한
trade-off다. 큰 성능 향상이나 detector 개선으로 해석하지 않는다.

Motion-model A/B에서는 CV가 일부 횡속도를 안정화했지만 로터리 FDE를 악화했고,
CA/CTRA도 전체 계약을 통과하지 못했다. 따라서 CTRV 유지 역시 실험 후 확정한 결정이지
새로운 P0--P9 기능의 적용은 아니다.

## P0--P9에서 얻었지만 production에 올리지 않은 개선

P7은 Base4 hardcase val speed MAE를 `0.676 m/s`까지 낮추고 개발 bag에서도
`0.811/0.779 m/s`를 기록했다. P8/P9의 초기 종방향 결합도 일부 speed·AVE와 FDE를
개선했다. 그러나 다음 이유로 E25 후단에 채택하지 않았다.

- E25는 detector velocity 자체를 출력하지 않는다.
- P8은 두 개발 bag의 Map-CV 전 horizon을 동시에 개선하지 못했다.
- P9 초기 결합은 stable FDE가 일관되지 않았다.
- P9 continuous fusion은 highway1 speed MAE `0.445 → 0.695 m/s`, stable FDE@3s
  `0.905 → 2.478 m`로 악화했다.
- 반복 사용한 두 개발 bag에 맞춘 threshold 최적화 위험이 있다.

따라서 P0--P9의 성과는 다음 temporal detector 설계와 좌표계·불확실성 계약에 대한
근거이지, 현재 E25 production의 성능 수치에 합산할 수 있는 개선량이 아니다.

## 현재 정적 장애물 문제와의 관계

2026-09-24 정적 장애물 rosbag에서 확인한 duplicate detection, 짧은 detection gap,
track ID 재생성 문제는 P0--P9 속도 연구와 별개다. 아직 E25 production에 다음 개선은
적용되지 않았다.

- class-aware BEV NMS로 동일 장애물 duplicate box 억제.
- 정적 장애물의 짧은 miss 구간에 대한 association/track lifecycle 보강.
- 초근접·측면 관측 분포의 detector 보강.

이 항목이 검증되어 채택되면 E25 checkpoint를 바꾸지 않고도 tracker/planning 입력을
실제로 개선할 수 있다. 적용 전에는 현재 성능으로 기록한다.

## 보고·평가 시 표현 규칙

현재 결과를 설명할 때는 다음처럼 구분한다.

- **E25 detector 성능:** E25 checkpoint와 `0.40/0.15/0.10` threshold의 결과.
- **E25 production 성능:** E25 + 현재 position-only tracker + UNKNOWN stationary + Map-CV.
- **Base4 Q1 연구 성능:** P0--P9의 별도 temporal/velocity checkpoint 및 연구 mode 결과.

“P7이 좋아졌으므로 E25 production도 그만큼 좋아졌다”거나 “P8 frame fix가 들어갔으므로
E25가 9D velocity를 사용한다”라고 쓰지 않는다. E25 후단의 정량 개선으로 주장할 수 있는
것은 같은 E25 detection으로 paired 평가해 production에 채택한 변경만 해당한다.

## 관련 문서

- Production 현재 계약:
  [`../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md`](../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)
- P0--P9 전체 ledger:
  [`260923_BASE4_Q1_EXECUTION_ROADMAP.md`](260923_BASE4_Q1_EXECUTION_ROADMAP.md)
- E25 tracker noise·motion-model A/B:
  [`260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md`](260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md)
- P8 frame/output 실험:
  [`260924_BASE4_Q1_P8_LONGITUDINAL_OUTPUT_RESULTS.md`](260924_BASE4_Q1_P8_LONGITUDINAL_OUTPUT_RESULTS.md)
- P9 uncertainty fusion 실험:
  [`260924_BASE4_Q1_P9_UNCERTAINTY_FUSION_RESULTS.md`](260924_BASE4_Q1_P9_UNCERTAINTY_FUSION_RESULTS.md)
- 현재 정적 장애물 진단:
  [`260924_STATIC_OBSTACLE_TRACKING_DIAGNOSIS.md`](260924_STATIC_OBSTACLE_TRACKING_DIAGNOSIS.md)
