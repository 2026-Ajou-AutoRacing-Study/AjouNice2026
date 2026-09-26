# Competition yaw-rate와 control yaw smoothing 후속 실험

상태: **EVIDENCE / REJECTED PRODUCTION CHANGES**  
검증일: 2026-09-25

## 결론

현재 production의 IMU yaw-rate와 Smooth control state `translation/yaw tau
0.5/0.5 s`를 유지한다.

- Competition packet `angular.z`는 ROS rad/s가 아니라 clockwise deg/s였다.
  ROS `base_link` yaw-rate로 쓰려면 `-pi/180` 변환이 필요하다.
- 변환 후 Competition yaw-rate는 heading 미분과 잘 맞지만 IMU를 대체한 전체 EKF
  replay에서 의미 있는 성능 우위가 없었다.
- `yaw tau=0`을 기존 alignment 원점에 단순 적용하면 회전이 차량 현재 위치가 아닌
  odom 원점 주위에서 일어나 XY가 크게 흔들린다.
- 차량 현재 위치를 회전 중심으로 바로잡은 SE(2) 후보도 `yaw tau=0`에서 이득이 없었다.

## 평가 계약

- 입력: `260925_194856_smooth_tau0p5_full_route_no_actors.bag`.
- MR, `gt_source=competition`, evaluator heading `-1.347 deg`.
- production Q/R: GNSS sigma `0.10 m`, Global Qxy `0.0005`.
- replay rate `1x`, output state 약 `50 Hz`.
- Competition 후보는 IMU yaw-rate를 대체하고 기존 IMU acceleration만 유지했다.
  두 yaw-rate를 중복 fusion하지 않았다.
- 첫 adapter 실행은 covariance tuple 쓰기 오류로 후보 topic이 0건이었다. 이 결과는
  전부 폐기했다. 수정 실행은 후보 IMU `7,859`건과 state `7,860`건을 확인했다.

## raw yaw-rate 계약 감사

두 최신 전체주행 bag에서 native Competition angular.z와 IMU의 비율 중앙값은 약
`-57.34`였다. 이는 `-180/pi`와 일치한다. `-deg2rad` 변환 후 결과는 다음과 같다.

| bag | 신호 | heading 미분 best lag | correlation | p95 error |
|---|---|---:|---:|---:|
| Smooth 폐루프 | IMU | `+22 ms` | `0.999862` | `0.00637 rad/s` |
| Smooth 폐루프 | Competition | `-4 ms` | `0.999856` | `0.00658 rad/s` |
| Global 폐루프 | IMU | `+20 ms` | `0.999854` | `0.00670 rad/s` |
| Global 폐루프 | Competition | `-4 ms` | `0.999882` | `0.00605 rad/s` |

Competition과 IMU 사이 상관계수도 `0.99995` 이상이었다. 독립 센서 두 개가 아니라
같은 simulator state의 서로 다른 출력으로 판단한다.

## 전체 EKF 대체 A/B

GPS 음영을 제외한 이동 구간이다.

| 지표 | IMU yaw-rate | Competition yaw-rate | 판단 |
|---|---:|---:|---|
| 횡 오차 p50 / p95 | `1.696 / 5.025 cm` | `1.704 / 5.023 cm` | 동등 |
| 종 오차 p50 / p95 | `0.935 / 3.038 cm` | `0.935 / 3.064 cm` | 동등 |
| yaw 오차 mean / p95 | `0.00708 / 0.02986 deg` | `0.00720 / 0.02938 deg` | mixed |
| 횡 jump p50 / p95 | `0.204 / 1.265 cm` | `0.207 / 1.268 cm` | 동등 |
| 종 jump p50 / p95 | `0.868 / 3.527 cm` | `0.870 / 3.437 cm` | mixed |

Competition 입력의 약 20 ms 빠른 위상은 EKF output을 유의미하게 개선하지 않았다.
새 adapter node와 runner는 규정·센서 변경 회귀 진단용으로 남기되 production launch에는
연결하지 않는다.

## yaw tau 분리 결과

기존 alignment x/y/yaw를 독립 필터링하면서 yaw tau만 0으로 만들면 횡 p95가
Smooth bag에서 `5.13 -> 15.25 cm`, Global bag에서 `5.62 -> 17.99 cm`로 악화했다.
이는 odom 원점에서의 alignment 회전이 차량 XY까지 이동시키기 때문이다.

차량 현재 pose를 회전 중심으로 삼도록 SE(2) correction을 다시 구성한 후보에서는
이 결함은 제거됐다. 그러나 translation tau `0.5 s`에서 yaw tau `0`은 두 bag 모두
tau `0.5 s`보다 yaw p95와 종합 지표가 좋아지지 않았다. 따라서 구조 변경과 production
parameter 변경을 모두 기각한다.

## 재현

```bash
python3 scripts/evaluation/localization/analyze_competition_yaw_rate.py INPUT.bag

LOCALIZATION_YAW_RATE_AB_OUTPUT_DIR=/tmp/yaw_rate_ab \
scripts/evaluation/localization/run_competition_yaw_rate_ab.sh INPUT.bag
```
