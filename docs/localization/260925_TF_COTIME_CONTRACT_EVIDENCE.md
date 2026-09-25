# Local/Global EKF TF timestamp 정합 수정

상태: **EVIDENCE / IMPLEMENTED**
검증일: 2026-09-25

## 결론

기존 final-state publisher는 Global EKF message timestamp와 무관하게 가장 최근의
`odom -> base_link` TF를 사용해 `map -> odom`을 만들었다. 두 50 Hz EKF callback의
위상이 달라 실제 폐루프 bag에서 Local pose는 Global pose보다 다음만큼 오래됐다.

| bag | Local signed age p50 | 속도×age 위치 차이 p50 / p95 |
|---|---:|---:|
| production 장주행 | `16.53 ms` | `13.81 / 28.88 cm` |
| production 짧은 반복 | `10.60 ms` | `7.48 / 17.40 cm` |
| 기각 Q/R | `2.76 ms` | `2.56 / 4.51 cm` |

이는 `/localization/kinematic_state` 자체의 pose 오차가 아니라 TF graph 내부의
시간 불일치다. 같은 timestamp에서 다음 등식이 닫히지 않았다.

```text
(map -> odom) x (odom -> base_link) == map-frame kinematic_state
```

## 수정

`gnss_imu_kinematic_state_publisher.py`가 Local odometry를 직접 구독하고, 최신 Local
pose를 Global message timestamp까지 body twist로 짧게 외삽한 뒤 `map -> odom`을
합성한다. Global kinematic state 값, EKF Q/R, heading, GNSS와 UDP 시간 정책은
변경하지 않았다.

- Local age 허용값: `50 ms`.
- pose/twist frame 계약이 다르면 거부.
- age가 허용값을 넘으면 잘못된 TF를 발행하지 않고 기다림.
- controller용 smooth state가 이미 사용한 것과 같은 co-time 원리다.

## 동일 bag 전후 결과

Production 짧은 반복 bag을 1×로 전체 replay했다.

| TF closure | 기존 기록 | 수정 replay |
|---|---:|---:|
| position p50 | `7.494 cm` | **수치 정밀도 수준** |
| position p95 | `17.397 cm` | **0.071 cm** |
| position max | `19.383 cm` | **0.575 cm** |
| yaw p95 | `0.350°` | **0.047°** |
| kinematic state rate | 약 `50 Hz` | `49.918 Hz` |

새 평가기의 `tf_contract`가 이 closure를 자동 보고한다. 위치 p95 잔차는 rosbag에
기록된 두 state 사이의 선형 보간 및 아주 짧은 회전 운동 근사 오차 수준이다.

## 영향과 gate

이 수정은 TF를 통해 센서·객체·지도 좌표를 조회하는 소비자의 순간 정합을 개선한다.
Planning이 직접 읽는 kinematic pose는 바뀌지 않으므로 route/controller 동작을
임의로 튜닝하는 수정이 아니다. 그래도 실차 승인 전 다음을 확인한다.

1. 새 MR 폐루프에서 `tf_contract.position_closure_error_m.p95 < 1 cm`.
2. `/localization/kinematic_state` 약 50 Hz 유지 및 Local age 경고 없음.
3. LiDAR/Object를 map에 표시했을 때 차체·지도 정합 회귀 없음.
4. GPS 음영과 checkpoint 재배치에서도 TF tree가 단절되지 않음.

