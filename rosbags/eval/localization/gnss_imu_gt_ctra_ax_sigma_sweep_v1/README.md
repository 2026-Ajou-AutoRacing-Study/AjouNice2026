# IMU ax covariance sweep — GT vx tuned

## 비교 조건

- raw bag:
  `rosbags/raw_sensor/competition_route_full_highspeed.bag`
- 전체 구간: 약 `155.7 s`, `2249 m`
- GT longitudinal velocity 표준편차: `0.01 m/s`
- GT heading 보정: `-1.347 deg`
- IMU ax: 모든 조건에서 ON
- 비교 표준편차:
  `0.01 / 0.05 / 0.10 m/s²`
- 나머지 EKF, 센서, GNSS 음영 차단 및 재생 조건은 동일

`sigma_0p01`은 최신 최종 v2 bag을 재사용했다. `sigma_0p05`와
`sigma_0p10`은 동일 코드와 `rosbag play -r 2.0`으로 새로 생성했다.

## 결과 파일

```text
sigma_0p01/results/full.csv
sigma_0p01/results/full.txt

sigma_0p05/
  gnss_imu_gt_ctra_vx_0p01_ax_sigma_0p05_full.bag
  results/full.csv
  results/full.txt

sigma_0p10/
  gnss_imu_gt_ctra_vx_0p01_ax_sigma_0p10_full.bag
  results/full.csv
  results/full.txt
```

## 전체 구간 공통 timestamp 비교

| 지표 | σ=0.01 | σ=0.05 | σ=0.10 |
| --- | ---: | ---: | ---: |
| XY mean | **0.1267 m** | 0.1275 m | 0.1269 m |
| XY RMSE | 0.1653 m | 0.1613 m | **0.1607 m** |
| 종방향 MAE | 0.1085 m | 0.1013 m | **0.0998 m** |
| 횡방향 MAE | 0.0437 m | 0.0428 m | **0.0426 m** |
| `linear.x` bias | **0.00035 m/s** | 0.00039 m/s | 0.00043 m/s |
| `linear.x` MAE | 0.00763 m/s | **0.00711 m/s** | 0.00717 m/s |
| `linear.x` RMSE | 0.02406 m/s | 0.02121 m/s | **0.01966 m/s** |
| `linear.x` max | 0.475 m/s | 0.435 m/s | **0.310 m/s** |
| speed MAE | 0.01064 m/s | **0.01013 m/s** | 0.01019 m/s |
| speed RMSE | 0.02643 m/s | 0.02395 m/s | **0.02263 m/s** |

위치 지표의 수 cm 미만 차이는 동일 설정 반복 실행 변동과 겹치므로
ax covariance의 확정적인 효과로 해석하지 않는다. 속도 지표는 세 조건의
주요 비교 대상으로 사용한다.

## 주행 조건별 속도 MAE

| 구간 | 지표 | σ=0.01 | σ=0.05 | σ=0.10 |
| --- | --- | ---: | ---: | ---: |
| 가감속 `|a|>0.5` | `vx` | 0.01642 | 0.01517 | **0.01500** |
| 가감속 `|a|>0.5` | speed | 0.01749 | 0.01627 | **0.01612** |
| 강한 가감속 `|a|>1` | `vx` | 0.01708 | 0.01584 | **0.01552** |
| 정속 `|a|<0.1` | `vx` | 0.00074 | **0.00070** | 0.00078 |
| GNSS 음영 7.1초 | `vx` | **0.00012** | 0.00012 | 0.00015 |
| GNSS 음영 7.1초 | speed | 0.00014 | **0.00013** | 0.00015 |

## 속도 연속성

| 지표 | σ=0.01 | σ=0.05 | σ=0.10 |
| --- | ---: | ---: | ---: |
| `vx` 오차 frame-step MAE | 0.01046 | **0.01029** | 0.01030 |
| `vx` 오차 frame-step RMS | 0.02830 | 0.02493 | **0.02445** |
| state `vx` 변화량 p95 | 0.07812 | 0.07604 | **0.07570** |
| state `vx` 변화량 max | 0.620 | **0.517** | 0.517 |

`0.01`은 IMU ax를 강하게 따라가므로 일부 순간 가속도 spike가 state
속도에 더 직접 반영됐다. `0.05`와 `0.10`은 GT vx를 유지하면서 이러한
spike를 완화했다.

## Filtered acceleration

`/localization/acceleration.linear.x`와
`/Ego_topic.acceleration.x`를 timestamp 보간 비교했다.

| 지표 | σ=0.01 | σ=0.05 | σ=0.10 |
| --- | ---: | ---: | ---: |
| acceleration MAE | **0.177 m/s²** | 0.268 m/s² | 0.315 m/s² |
| acceleration RMSE | **0.703 m/s²** | 0.883 m/s² | 0.977 m/s² |
| p95 절대 오차 | **0.519 m/s²** | 0.749 m/s² | 1.064 m/s² |
| 상관계수 | **0.911** | 0.851 | 0.814 |

`0.01`은 acceleration output의 측정 추종성이 가장 좋다. 반대로
`0.10`은 acceleration state를 더 부드럽게 만들지만 실제 가속도 변화도
더 많이 약화한다.

## 결론

현재 구조의 균형값으로 **`0.05 m/s²`를 권장**한다.

- 전체 `vx` 및 speed MAE가 세 조건 중 가장 작다.
- `0.01`보다 속도 RMSE와 rare step이 감소한다.
- 정속 및 GNSS 음영 구간의 속도 지표가 가장 좋다.
- `0.10`보다 IMU acceleration 정보를 더 많이 보존한다.

`0.10`은 강한 가감속의 RMSE와 최대 속도 오차를 더 줄이므로 tail
robustness를 최우선으로 할 때 대안이 된다. 그러나 `0.05`와 `0.10`의
평균 속도 차이는 매우 작고 단일 bag의 실행 변동보다 작을 수 있다.

현재 저장소 기본값은 sweep 결과 승인 전까지 `0.01 m/s²`를 유지한다.
