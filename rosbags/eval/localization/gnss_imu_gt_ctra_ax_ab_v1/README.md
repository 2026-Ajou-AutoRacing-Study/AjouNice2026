# IMU ax ON/OFF A/B — GT vx tuned

## 비교 조건

두 조건은 다음을 동일하게 유지했다.

- raw bag:
  `rosbags/raw_sensor/competition_route_full_highspeed.bag`
- 전체 구간: 약 `155.7 s`, `2249 m`
- GT longitudinal velocity 표준편차: `0.01 m/s`
- GT heading 보정: `-1.347 deg`
- IMU `wz`, GT yaw/vx, GNSS x/y 설정
- `frequency: 50`, `predict_to_current_time: true`
- 동일 코드와 `rosbag play -r 2.0`

유일한 알고리즘 차이는 다음과 같다.

| 조건 | IMU `linear_acceleration.x` |
| --- | --- |
| ax ON | 융합, 표준편차 `0.01 m/s²` |
| ax OFF | `imu0_config`에서 비활성화 |

ax ON bag은 최신 v2 최종 결과를 사용했다.

```text
rosbags/eval/localization/
  gnss_imu_gt_ctra_heading_corrected_vx_tuned_full_v2/
    gnss_imu_gt_ctra_heading_corrected_vx_tuned_full_v2.bag
```

ax OFF 결과는 다음 경로에 있다.

```text
ax_off/
  gnss_imu_gt_ctra_vx_0p01_ax_off_full.bag
  results/full.csv
  results/full.txt
```

ax ON의 같은 평가 결과는 `ax_on/results/`에도 저장했다.

## 전체 구간 결과

서로 다른 출력 sample 시각의 영향을 줄이기 위해 ax ON state 시각에
ax OFF 결과를 선형보간하여 비교했다.

| 지표 | ax ON | ax OFF | 판단 |
| --- | ---: | ---: | --- |
| XY mean | 0.1267 m | 0.1276 m | 사실상 동일 |
| XY RMSE | 0.1653 m | 0.1611 m | 실행 변동 범위 |
| 종방향 MAE | 0.1085 m | 0.0998 m | OFF가 작지만 반복 실행 변동과 겹침 |
| 횡방향 MAE | 0.0437 m | 0.0427 m | 사실상 동일 |
| `linear.x` MAE | **0.00762 m/s** | 0.01088 m/s | **ON이 30.0% 작음** |
| `linear.x` RMSE | **0.02405 m/s** | 0.02523 m/s | ON이 4.7% 작음 |
| `linear.x` max | 0.475 m/s | **0.433 m/s** | OFF가 rare max에 유리 |
| speed MAE | **0.01063 m/s** | 0.01399 m/s | **ON이 24.0% 작음** |
| speed RMSE | **0.02643 m/s** | 0.02809 m/s | ON이 5.9% 작음 |

## 주행 조건별 속도

| 구간 | 지표 | ax ON | ax OFF | ON 효과 |
| --- | --- | ---: | ---: | ---: |
| 가감속 `|a|>0.5` | `vx` MAE | 0.01641 | 0.02014 | **18.5% 감소** |
| 가감속 `|a|>0.5` | speed MAE | 0.01747 | 0.02129 | **17.9% 감소** |
| 강한 가감속 `|a|>1` | `vx` MAE | 0.01708 | 0.02124 | **19.6% 감소** |
| 정속 `|a|<0.1` | `vx` MAE | 0.00074 | 0.00326 | **77.2% 감소** |
| GNSS 음영 7.1초 | `vx` MAE | 0.00012 | 0.00022 | **46.6% 감소** |

## Filtered acceleration

`/localization/acceleration.linear.x`를 `/Ego_topic.acceleration.x`와
timestamp 보간 비교했다.

| 지표 | ax ON | ax OFF |
| --- | ---: | ---: |
| acceleration MAE | **0.177 m/s²** | 0.570 m/s² |
| acceleration RMSE | **0.703 m/s²** | 1.253 m/s² |
| p95 절대 오차 | **0.519 m/s²** | 2.097 m/s² |
| 상관계수 | **0.911** | 0.666 |

ax OFF에서도 GT vx와 위치 update의 교차공분산으로 acceleration state가
간접 추정되지만, 직접 IMU ax를 융합한 결과보다 정확하지 않았다.

## 연속성

| 지표 | ax ON | ax OFF |
| --- | ---: | ---: |
| `vx` 오차 frame-step MAE | **0.01046 m/s** | 0.01164 m/s |
| `vx` 오차 frame-step RMS | 0.02830 m/s | **0.02567 m/s** |
| state `vx` 변화량 p95 | 0.07812 m/s | **0.07655 m/s** |
| state `vx` 변화량 max | 0.620 m/s | **0.495 m/s** |

ON은 일반적인 frame-step 오차가 더 작지만 일부 rare spike는 더 크다.

## 반복 실행 변동 확인

동일한 ax ON 설정을 두 번 실행해 비교했을 때도 position 및 yaw 지표가
다음 정도 변했다.

- XY RMSE: 약 `2.5%`
- 종방향 MAE: 약 `5.7%`
- yaw MAE: 약 `10.8%`

따라서 ON/OFF 위치 지표의 약 `1~8%` 차이를 ax의 인과 효과로 단정하지
않는다. 반면 `vx`와 speed MAE의 ON 우위는 반복 실행 변동보다 크고,
가감속·정속·음영 구간에서 같은 방향으로 확인됐다.

## 결론

현재 GT vx `0.01 m/s` 설정에서도 IMU ax를 유지한다.

- GT vx 사이 50 Hz propagation의 속도 정확도가 개선된다.
- 가감속뿐 아니라 정속·GNSS 음영 구간에서도 `vx`가 개선된다.
- filtered acceleration 품질이 크게 개선된다.
- 전체 XY와 횡방향 위치 정확도는 실질적으로 악화되지 않는다.

다만 `ax sigma=0.01 m/s²`가 최적인지는 이 ON/OFF 시험만으로 확정할 수
없다. rare spike를 줄여야 한다면 다음 단계에서
`0.01 / 0.05 / 0.10 m/s²`를 비교한다.
