# Base4 Q1-P9 종방향 불확실성 결합 결과

> 날짜: 2026-09-24.
>
> 상태: covariance 전달 계약 복구, 초기 구간 가능성 확인, continuous 결합 기각.

## 구현

`DetectedObject.twist_with_covariance`에는 detector 속도 분산이 실려 있었지만 tracker는
이를 읽지 않고 고정 노이즈만 사용했다. P9은 이 covariance를 tracker measurement와
track에 전달하고 mode 5에서 다음 역분산 가중치를 사용한다.

```text
detector_weight = tracker_longitudinal_variance /
                  (tracker_longitudinal_variance + detector_variance)
```

- 종방향 출력만 결합한다.
- lateral velocity와 EKF spatial state는 position tracker를 유지한다.
- Vehicle/Truck, fresh detector observation, longitudinal innovation gate만 허용한다.
- 기본값은 계속 mode 0이다.

현재 BEVFusion bridge가 싣는 detector covariance는 query별 추정값이 아니라 전역 설정
`detector_velocity_noise_std_mps`에서 만든 값이다. 따라서 이번 단계는 uncertainty
전달·결합 경로를 완성한 P9-A이며, temporal fit residual/span을 query별 covariance로
변환하는 P9-B는 새 held-out bag 이후 판단한다.

## 검증

- tracker build PASS.
- detection/center-motion velocity test 포함 `30 tests`, failure/error/skip 0.
- mode 5가 detector variance를 보존하면서 EKF velocity state를 직접 초기화하지 않는
  단위시험을 추가했다.

## 기존 개발 bag 회귀 결과

P7 detections를 재사용했으며 새 detector 추론은 하지 않았다. 현재 빌드에서 mode 0과
mode 5를 다시 실행한 paired 결과다. 두 bag은 개발에 반복 사용했으므로 모델 선택
증거가 아니라 regression evidence다.

| Bag | 설정 | speed MAE | 2D AVE | all FDE 1/2/3s | stable FDE 1/2/3s |
| --- | --- | ---: | ---: | ---: | ---: |
| highway1 | mode 0 | `0.445` | `0.742` | `0.824/1.971/3.694` | **`0.487/0.678/0.905`** |
| highway1 | mode 5, 초기 1초 | **`0.440`** | **`0.735`** | **`0.776/1.805/3.377`** | `0.502/0.703/0.941` |
| merge | mode 0 | `0.880` | `1.337` | `1.679/3.377/5.708` | `1.192/1.812/2.503` |
| merge | mode 5, 초기 1초 | **`0.858`** | **`1.306`** | **`1.625/3.287/5.603`** | `1.185/1.830/2.552` |

속도와 all-FDE는 양쪽 bag에서 좋아졌지만 stable FDE는 일관되게 좋아지지 않았다. 또한
merge 실행은 tracker output frame 수가 `941 vs 956`으로 달라 작은 차이는 재생
scheduler 민감도와 분리하기 어렵다.

## Continuous 결합 반증

track-age 제한을 제거하고 covariance만으로 mature track까지 계속 결합한 highway1
A/B는 다음처럼 명확히 악화했다.

- speed MAE: `0.445 → 0.695 m/s`
- 2D AVE: `0.742 → 0.950 m/s`
- stable FDE@1/2/3s: `0.487/0.678/0.905 → 0.878/1.633/2.478 m`

현재 EKF covariance는 실제 속도 오차 calibration을 보장하지 않고, P7 detector도 mature
position track보다 항상 정확하지 않다. 따라서 covariance 숫자만 믿는 무제한 continuous
fusion은 기각한다. mode 5는 초기 1초 연구 옵션으로 제한하고 production mode 0을 유지한다.

## 다음 gate

새 가속·감속 held-out bag이 오면 먼저 mode 0, mode 4, mode 5를 같은 출력 cadence로
평가한다. 그 결과가 양쪽 motion slice에서 일관될 때만 query별 temporal uncertainty
(fit residual, temporal span, 유효 memory 수)를 ROS covariance로 전달하는 P9-B를 연다.
