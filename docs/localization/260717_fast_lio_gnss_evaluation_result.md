# 260717 FAST-LIO + GNSS 평가 결과

## 1. 평가 대상

이번 문서는 FAST-LIO + GNSS provider를 기존 `raw1.bag` 데이터로 실행한 뒤
기록한 결과의 **성능 평가만** 다룬다. 구현 방법과 실행 절차는 다루지 않는다.

- 평가 bag: `rosbags/eval/fast_lio_gnss_full_0717_2004.bag`
- 평가 state: `/localization/kinematic_state`
- GT: `/Ego_topic`
- 평가 샘플: 2,014개
- 평가 시간: 103.780초
- GT 주행거리: 316.815 m
- state 기준점: `base_link`

평가 원본은 다음 두 파일이다.

- `rosbags/eval/fast_lio_gnss_full_0717_2004_state_eval.txt`
- `rosbags/eval/fast_lio_gnss_full_0717_2004_state_eval.csv`

## 2. 결론

GNSS global correction을 적용한 결과, **위치 성능은 실험용 provider로서 매우
크게 개선됐다.** 평균 XY 오차는 0.194 m, 최대 오차는 0.722 m이며 전체
구간에서 1 m를 넘는 위치 오차가 없었다. FAST-LIO 단독 결과에서 나타났던
주행 후반의 수 m 단위 누적 위치 오차는 제거된 것으로 판단한다.

반면 `/localization/kinematic_state`의 **속도는 아직 planning/control에 그대로
사용하기 어렵다.** 속력 MAE가 0.747 m/s이고, 정지 GT 구간에서도 최대 약
1.75 m/s가 출력됐다. GNSS로 보정된 pose는 정지 위치를 비교적 잘 유지하지만
state 내부의 velocity는 이를 따라가지 못하고 있다.

Yaw는 평균 약 1.08도의 체계적인 편차가 남았고, yaw rate는 한 번의 짧은
spike를 제외하면 양호했다. 따라서 현재 결과는 다음과 같이 요약할 수 있다.

> Global position correction은 성공했지만, velocity와 yaw까지 완성된 최종
> localization state라고 보기는 어렵다.

## 3. 위치 평가

| 항목 | 결과 |
|---|---:|
| Ego path length | 316.815 m |
| FAST-LIO path length | 322.213 m |
| Path length ratio | 1.01704 |
| Path length error | +5.397 m |
| XY 평균 오차 | 0.194 m |
| XY RMSE | 0.253 m |
| XY 최대 오차 | 0.722 m |
| XY 최종 오차 | 0.323 m |
| 최종오차/주행거리 | 0.102% |
| 1 m 초과 오차 구간 | 0개 |

XY 오차 분포는 다음과 같다.

| 백분위 | XY 오차 |
|---|---:|
| 50% | 0.100 m |
| 90% | 0.479 m |
| 95% | 0.542 m |
| 99% | 0.620 m |

최대 XY 오차는 상대시간 17.760초에서 0.722 m였다. 전체 샘플 중 XY 오차가
0.5 m를 넘은 비율은 약 8.19%였지만 1 m를 넘은 샘플은 없었다.

축별 signed error 평균은 다음과 같다.

- X: -0.012 m
- Y: +0.181 m
- Z: +0.065 m

X 방향 편향은 거의 없지만 Y 방향으로 약 0.18 m의 잔여 편향이 있다. 경로가
GT의 한쪽으로 치우쳐 보인다면 이 값과 연결해 확인할 필요가 있다. 다만 오차가
주행거리와 함께 계속 증가하는 형태는 아니므로, 이번 결과만 보면 큰 누적
drift보다는 좌표 정렬·기준점·GNSS 보정의 잔여 offset에 더 가깝다.

FAST-LIO 경로가 GT보다 5.397 m 길다는 것은 최종 위치가 5.397 m 벗어났다는
뜻이 아니다. 최종 위치 오차는 0.323 m이며, 경로 길이 차이는 작은 위치 진동과
반복적인 GNSS correction이 누적된 결과로 해석하는 것이 타당하다.

## 4. FAST-LIO 단독 결과와 비교

기존 동일 raw 데이터의 FAST-LIO 단독 평가와 비교하면 다음과 같다.

| 항목 | FAST-LIO 단독 | FAST-LIO + GNSS | 변화 |
|---|---:|---:|---:|
| XY 평균 오차 | 4.388 m | 0.194 m | 약 95.6% 감소 |
| XY RMSE | 4.957 m | 0.253 m | 약 94.9% 감소 |
| XY 최대 오차 | 7.568 m | 0.722 m | 약 90.5% 감소 |
| XY 최종 오차 | 7.483 m | 0.323 m | 약 95.7% 감소 |
| 최종오차/주행거리 | 2.363% | 0.102% | 2.261%p 감소 |
| Yaw 평균 절대오차 | 0.131 deg | 1.083 deg | 악화 |

이 비교에서 GNSS가 위치 drift를 억제한 효과는 명확하다. 반면 position-only
GNSS correction은 heading을 직접 관측하지 않으므로 yaw 성능까지 함께
개선하지는 못했다.

## 5. Yaw 평가

| 항목 | 결과 |
|---|---:|
| 초기 정렬 yaw | -0.022 deg |
| Yaw 평균 절대오차 | 1.083 deg |
| Yaw 최대 절대오차 | 1.914 deg |
| Yaw signed bias | -1.054 deg |

Yaw 절대오차의 중앙값은 1.150도이고, 전체 샘플 중 약 79.6%가 1도를 넘었다.
최대 오차는 상대시간 33.060초에서 -1.914도였다.

평균 절대오차 1.083도와 signed bias -1.054도가 거의 같으므로, 무작위로
흔들리는 오차보다는 **대부분 한 방향으로 약 1도 돌아간 체계적 편차**에
가깝다. 시작 정렬값은 -0.022도로 작지만 주행 이후 편차가 형성되므로 다음
항목을 별도로 검토해야 한다.

- FAST-LIO yaw drift
- LiDAR–IMU 및 `body`–`base_link` 회전 extrinsic
- 초기 heading 정렬 방식
- Ego heading과 state timestamp의 기준 차이
- 단일 GNSS의 position-only correction 한계

단일 GNSS 위치만으로는 정지 상태의 절대 heading을 직접 관측할 수 없으므로,
GNSS를 추가했다는 이유만으로 yaw도 자동으로 보정될 것으로 기대하면 안 된다.

## 6. 선속도 평가

| 항목 | Bias | MAE | RMSE | 최대 절대오차 |
|---|---:|---:|---:|---:|
| `linear.x` | -0.587 m/s | 0.704 m/s | 0.755 m/s | 1.074 m/s |
| `linear.y` | +0.307 m/s | 0.431 m/s | 0.738 m/s | 2.071 m/s |
| `linear.z` | +0.255 m/s | 0.407 m/s | 0.496 m/s | 0.876 m/s |
| speed | +0.355 m/s | 0.747 m/s | 0.812 m/s | 1.651 m/s |

속력 MAE 0.747 m/s는 약 2.69 km/h이고, 최대 오차 1.651 m/s는 약
5.94 km/h이다. 전체 샘플의 약 80.1%에서 속력 오차가 설정 임계값
0.5 m/s를 넘었다.

최대 속력 오차는 상대시간 27.340초에서 발생했다.

- Ego speed: 0.098 m/s
- State speed: 1.749 m/s
- Speed error: +1.651 m/s

따라서 몇 개의 순간 spike만으로 평균이 커진 것이 아니라, 상당 구간에서
지속적으로 속도 차이가 존재한다. `linear.x`의 -0.587 m/s bias와
`linear.y`, `linear.z`의 양의 bias도 있어 단순한 zero-mean noise로 보기
어렵다.

10초 단위로 보아도 속력 MAE는 특히 20–30초 구간에서 1.028 m/s였고,
60–70초 구간에서 0.944 m/s였다. 평가기가 검출한 0.5 m/s 초과 구간도
10개이며, 그중 여러 구간이 수십 초 동안 이어졌다.

## 7. 정지 구간 평가

GT 기준 정지 구간은 총 3개이며 952개 샘플, 전체의 약 47.3%를 차지했다.

| 구간 | 시간 | State net 이동 | State 누적 이동 | State 평균 속력 | State 최대 속력 |
|---|---:|---:|---:|---:|---:|
| 1 | 0.000–3.520 s | 0.024 m | 0.090 m | 0.042 m/s | 0.822 m/s |
| 2 | 27.240–32.480 s | 0.348 m | 1.262 m | 1.580 m/s | 1.750 m/s |
| 3 | 52.440–92.560 s | 0.311 m | 4.400 m | 0.885 m/s | 1.033 m/s |

정지 샘플에서 state speed 중앙값은 약 0.885 m/s이고 최대값은 약
1.750 m/s였다. 정지 구간 속력 MAE는 약 0.884 m/s로, 이동 구간의 약
0.625 m/s보다 더 컸다.

여기서 `net 이동`과 `누적 이동`은 구분해야 한다. 세 번째 구간에서 최종
위치는 시작점으로부터 0.311 m만 달라졌지만, 중간의 작은 위치 진동을 모두
합한 누적 경로는 4.400 m이다. 따라서 차량이 한 방향으로 4.4 m drift했다는
뜻은 아니며, GNSS correction 전후의 pose jitter가 누적된 값이다.

더 중요한 문제는 pose의 net 이동은 작지만 published speed가 계속 약
0.9 m/s라는 점이다. 이는 GNSS로 보정된 pose와 estimator velocity가 서로
일관되지 않음을 보여준다. 정지 판정, 정지선 처리, TTC 및 제어 입력에 이
속도를 그대로 사용하면 문제가 생길 수 있다.

## 8. 각속도 평가

GT yaw rate는 연속된 Ego heading을 unwrap한 뒤 7개 샘플 local regression으로
계산했다.

| 항목 | 결과 |
|---|---:|
| `angular.z` bias | -0.0003 rad/s |
| `angular.z` MAE | 0.0018 rad/s |
| `angular.z` RMSE | 0.0076 rad/s |
| `angular.z` 최대 절대오차 | 0.1959 rad/s |

MAE 0.0018 rad/s는 약 0.103 deg/s로 전반적인 yaw-rate 추정은 양호하다.
0.1 rad/s 임계값을 넘은 구간은 14.180–14.280초의 한 구간뿐이며, 최대값은
14.240초에서 발생한 0.1959 rad/s였다. 전체 샘플 중 임계값 초과 비율은 약
0.15%이므로 지속적인 각속도 문제보다는 짧은 spike로 보는 것이 적절하다.

Ego GT가 yaw heading만 제공하므로 `angular.x`, `angular.y`는 평가하지 않았다.

## 9. 시간 동기화 품질

평가는 각 state timestamp 사이의 두 Ego 샘플을 찾아 GT pose, heading,
velocity를 해당 시각으로 보간했다.

| 항목 | 결과 |
|---|---:|
| Exact match | 0개 |
| Interpolated match | 2,014개 |
| 범위 밖 reject | 0개 |
| 큰 gap reject | 0개 |
| 제거된 중복 Ego timestamp | 2,904개 |
| 가장 가까운 Ego 원본 샘플 시간차 평균 | 0.0058 s |
| 가장 가까운 Ego 원본 샘플 시간차 최대 | 0.0190 s |

모든 state 샘플을 Ego 시계열 범위 안에서 보간했고 reject된 샘플이 없다.
가까운 원본 GT까지의 시간차도 최대 19 ms여서 이번 결과의 큰 속도 오차를
시간 매칭 문제만으로 설명하기는 어렵다.

중복 timestamp 2,904개는 평가기가 동일한 Ego header stamp를 가진 샘플을
정리한 수치다. 중복을 제거한 뒤 양쪽 샘플 사이를 보간했기 때문에 `exact=0`,
`interpolated=2014`가 나오는 것 자체는 평가 실패를 의미하지 않는다.

## 10. 현재 사용 가능 범위와 후속 평가 우선순위

현재 결과를 기준으로 한 판단은 다음과 같다.

- **Global pose:** 시제품 및 planning 연동 시험에 사용할 수 있는 수준이다.
- **Yaw:** 약 1도의 한 방향 bias 원인을 확인한 뒤 사용하는 것이 좋다.
- **Linear velocity:** 현재 값 그대로 최종 planning/control 입력에 쓰기에는
  오차와 정지 시 잔류 속도가 크다.
- **Yaw rate:** 한 번의 spike 처리를 제외하면 비교적 신뢰할 수 있다.
- **Covariance:** 실제 MORAI noise 조건을 반영하지 않은 상태이므로 이번
  결과만으로 대회 환경 성능을 확정할 수 없다.

후속 평가 우선순위는 다음과 같다.

1. Corrected pose를 미분한 속도와 published twist를 같은 timestamp에서 비교
2. 정지 상태 velocity 보정 또는 ZUPT 적용 전후 비교
3. Yaw signed error의 시간 변화와 extrinsic·초기 heading 영향 분리
4. GNSS covariance 및 position covariance floor 변경에 따른 pose jitter 비교
5. MORAI GPS/IMU/LiDAR noise를 적용한 반복 주행 평가
6. 더 긴 경로와 회전이 많은 경로에서 global correction 안정성 확인

현재 가장 중요한 후속 작업은 위치 오차를 더 줄이는 것보다 **GNSS로 보정된
pose와 published velocity가 서로 일관되게 움직이도록 만드는 것**이다.
