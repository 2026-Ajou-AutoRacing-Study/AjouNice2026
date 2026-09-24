# Base4 Q1-P7 Free-intercept Motion 결과

> 날짜: 2026-09-24.
> 상태: Detector 연구 baseline 갱신, tracker 직접 융합은 기각.

## 목적

P6는 현재 frame의 query center를 정확한 trajectory intercept로 고정하고 과거 center와의
pair slope를 `dt^2`로 결합했다. 그러나 현재 center에도 decoder localization noise가
있으므로 그 오차가 모든 pair에 공통으로 들어간다. P7은 현재와 과거 query center 전체에
대해 다음 직선을 동시에 적합한다.

`position(t) = intercept + velocity * t`

현재 center를 고정하지 않고 intercept와 velocity를 함께 구해 한 frame의 center noise가
속도로 증폭되는 현상을 줄인다. 학습 가중치와 spatial output은 바꾸지 않으며, P3 E1
checkpoint에 재학습 없이 적용했다.

## 구현과 검증

- config: `bevfusion_base4_q1_p7_free_intercept_4cam_lidar_velocity.py`
- `direct_velocity_fit_intercept=true`
- 현재 observation weight는 과거 observation 평균 weight의 `1.0`배다.
- 기존 P6와 이전 checkpoint 호환을 위해 기능 기본값은 `false`다.
- true velocity `2 m/s`, current center에만 `+0.2 m` noise를 넣은 tensor test에서
  fixed-intercept보다 free-intercept 결과가 정답에 가까움을 확인했다.
- Docker CUDA query-memory unit test는 `QUERY_MEMORY_P0_OK`로 통과했다.

## 재학습 없는 hardcase val

| 설정 | speed MAE | mAVE | mATE | Vehicle center mAP |
| --- | ---: | ---: | ---: | ---: |
| P6 fixed-intercept WLS2 | `0.7383` | `0.7987` | `0.2931` | `0.9106` |
| P7 free-intercept | **`0.6757`** | **`0.7393`** | `0.2930` | `0.9106` |

P7은 P6 대비 speed MAE를 약 `8.5%` 낮췄고 spatial 성능은 유지했다.

## 개발 rosbag detector A/B

threshold는 Vehicle/Pedestrian/Obstacle `0.40/0.15/0.10`으로 동일하다.

| Bag | 설정 | center mAP | F1 | speed MAE | bias | error-step p50 / p95 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| highway1 | P6 | `0.8395` | `0.9635` | `0.872` | `-0.104` | `0.454 / 1.492` |
| highway1 | P7 | `0.8408` | `0.9635` | **`0.811`** | `-0.366` | **`0.358` / `1.547`** |
| highway merge | P6 | `0.6423` | `0.7973` | `0.829` | `+0.241` | `0.426 / 2.511` |
| highway merge | P7 | `0.6423` | `0.7973` | **`0.779`** | **`+0.187`** | **`0.356 / 1.977`** |

P7은 두 bag 모두 평균과 p50을 개선했다. highway1 p95는 `0.055 m/s` 악화했지만 합류
p95는 크게 개선했다. paired inference latency는 5060 Ti 한-PC 조건에서 highway1
mean/p50/p95 `91.05/90.91/94.21 ms`, 합류 `95.37/90.25/119.35 ms`였다. MORAI와 분리된
4090 production PC에서 다시 측정하기 전까지 latency gate 통과로 보지 않는다.

## 등속 과적합 감사

GT speed의 인접 frame 변화로 재구성한 가속도 구간별 종방향 speed MAE는 다음과 같다.
이 slice는 GT acceleration의 근사치이므로 최종 held-out을 대체하지는 않는다.

| Bag | 설정 | steady `<=0.5` | moderate `0.5--2` | high `>2 m/s²` |
| --- | --- | ---: | ---: | ---: |
| highway1 | P6 | `0.798` | `0.874` | `0.858` |
| highway1 | P7 | `0.829` | **`0.788`** | **`0.775`** |
| highway merge | P6 | `0.978` | `0.709` | **`0.698`** |
| highway merge | P7 | **`0.813`** | **`0.661`** | `0.704` |

P7은 highway1의 moderate/high와 합류의 steady/moderate에서 개선했다. 합류 high는
`0.006 m/s` 차이로 사실상 동일하다. 따라서 현재 이득을 등속 구간 오버피팅으로만
설명할 근거는 없다. 다만 실제 로터리 가속과 급가속 합류를 포함한 새 held-out bag은
여전히 필요하다.

## Detector와 tracker 상보성

동일 GT sample에서 P6, P7과 position-only tracker를 비교했다.

| Bag | P6 | P7 | position tracker | sample별 oracle best |
| --- | ---: | ---: | ---: | ---: |
| highway1 | `0.858` | `0.795` | **`0.430`** | `0.258` |
| highway merge | `0.638` | **`0.618`** | `0.782` | `0.332` |

highway1에서는 mature track의 position history가 강하고, 합류에서는 P7이 tracker보다
좋다. highway1에서도 track age `<=1 s`에는 P7을 쓰고 이후 tracker를 쓰는 offline
selector가 `0.409 m/s`로 tracker 단독 `0.430 m/s`보다 좋아졌다. 이는 두 estimator가
상보적이라는 근거지만, GT sample 기반 selector이므로 그대로 production 로직으로 쓰지
않는다.

## 초기 1초 EKF 직접 융합 실험

실제 ROS pipeline에 mode 3을 추가해 track 생성 후 1초까지만 detector `vx, vy`를 EKF에
넣었다. 기본값은 계속 mode 0이며 실험 옵션만 추가했다.

| Bag | 설정 | F1 | center p50 | speed MAE | 2D AVE | stable minFDE-K 1/2/3s |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| highway1 | position-only | **`0.9804`** | **`0.280`** | `0.451` | **`0.595`** | **`0.374 / 0.588 / 0.815`** |
| highway1 | P7 early 1s | `0.9710` | `0.289` | **`0.431`** | `0.603` | `0.398 / 0.636 / 0.894` |
| highway merge | position-only | **`0.8855`** | **`0.590`** | `0.891` | **`1.047`** | **`1.246 / 1.885 / 2.600`** |
| highway merge | P7 early 1s | `0.8832` | `0.695` | **`0.826`** | `1.093` | `1.273 / 1.925 / 2.651` |

종방향 speed는 개선됐지만 center, lateral error, 2D AVE와 stable prediction이 함께
악화했다. 따라서 `vx, vy` 직접 EKF update는 production에 채택하지 않는다.

## 판정과 다음 단계

1. Q1 detector 연구 baseline은 P6에서 P7로 갱신한다.
2. Production은 E25와 position-only tracker mode 0을 유지한다.
3. mode 3은 기본 off인 실험 옵션으로만 보존한다.
4. 다음 P8은 detector의 2D velocity 전체를 EKF 상태에 주입하지 않는다.
   - track heading/lane tangent 기준 종방향 성분만 사용한다.
   - 초기 track 또는 tracker innovation이 큰 구간에만 조건부로 사용한다.
   - lateral velocity와 spatial state는 position tracker가 유지한다.
5. 급가속 합류와 로터리 가속을 포함한 새 held-out bag에서 acceleration slice와
   Map-CV를 검증하기 전 production 승격은 금지한다.
