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

## 11. GNSS timestamp 보정 후 재평가

GNSS 측정 시각과 FAST-LIO 예측 시각을 일치시키도록 수정한 뒤 같은 raw
데이터를 다시 처리했다. 앞 절의 결과는 수정 전 결과로 유지하며, 이 절에서
timestamp 보정 후 결과와 변화를 별도로 정리한다.

### 11.1 평가 대상

- 평가 bag: `rosbags/eval/fast_lio_gnss_full_0717_2211.bag`
- 평가 state: `/localization/kinematic_state`
- GT: `/Ego_topic`
- 평가 샘플: 2,014개
- 평가 시간: 103.780초
- GT 주행거리: 316.815 m
- state 기준점: `base_link`

평가 원본은 다음 두 파일이다.

- `rosbags/eval/fast_lio_gnss_timestamp_fix_eval_1.txt`
- `rosbags/eval/fast_lio_gnss_timestamp_fix_eval_1.csv`

### 11.2 핵심 결론

Timestamp 보정 후 XY 평균·RMSE·최종 오차와 종방향 오차가 모두 감소했다.
특히 최종 위치 오차는 0.323 m에서 0.152 m로 약 52.9% 감소했다. 따라서
과거 GNSS 위치를 LiDAR scan-end 시각의 현재 위치처럼 적용하던 것이 기존
종방향 지연성 오차의 주요 원인 중 하나였다고 판단할 수 있다.

반면 FAST-LIO 경로 길이 오차와 정지 중 누적 이동량은 크게 증가했다. 이는
절대 위치 정확도가 나빠졌다는 뜻이 아니라, GNSS가 pose를 반복적으로
당겨오는 동안 estimator 내부 velocity가 정지 상태에서도 0으로 수렴하지
않아 작은 pose 진동이 누적된 결과다. Timestamp 보정은 위치 시각 불일치를
해결했지만 velocity-state 및 정지 상태 문제까지 해결한 것은 아니다.

### 11.3 위치 평가

| 항목 | 보정 후 결과 |
|---|---:|
| Ego path length | 316.815 m |
| FAST-LIO path length | 332.273 m |
| Path length ratio | 1.04879 |
| Path length error | +15.457 m |
| XY 평균 오차 | 0.142 m |
| XY RMSE | 0.159 m |
| XY 최대 오차 | 0.561 m |
| XY 최종 오차 | 0.152 m |
| 최종오차/주행거리 | 0.048% |
| 1 m 초과 오차 구간 | 0개 |

수정 전후 위치 결과는 다음과 같다.

| 항목 | Timestamp 보정 전 | Timestamp 보정 후 | 변화 |
|---|---:|---:|---:|
| XY 평균 오차 | 0.194 m | 0.142 m | 약 26.8% 감소 |
| XY RMSE | 0.253 m | 0.159 m | 약 37.2% 감소 |
| XY 최대 오차 | 0.722 m | 0.561 m | 약 22.3% 감소 |
| XY 최종 오차 | 0.323 m | 0.152 m | 약 52.9% 감소 |
| 최종오차/주행거리 | 0.102% | 0.048% | 0.054%p 감소 |
| Path length error | +5.397 m | +15.457 m | 10.060 m 증가 |

절대 위치 오차는 전반적으로 개선됐지만, 누적 경로 길이 오차는 악화됐다.
그러므로 `path length error`를 위치 drift와 동일시하면 안 된다. 현재 경로
길이 증가는 정지 및 저속 구간의 pose jitter를 더 크게 반영한다.

### 11.4 종방향·횡방향 위치 오차

오차는 `state - GT`로 정의했다. 종방향 양수는 GT보다 앞, 음수는 GT보다
뒤이며, 횡방향 양수는 차량 진행 방향 기준 왼쪽, 음수는 오른쪽이다.

| 방향 | Bias | MAE | RMSE | 최대 절대오차 |
|---|---:|---:|---:|---:|
| 종방향 | -0.080 m | 0.088 m | 0.104 m | 0.561 m |
| 횡방향 | -0.028 m | 0.099 m | 0.120 m | 0.402 m |

- 제곱 위치 오차 기여도: 종방향 42.8%, 횡방향 57.2%
- 최종 종방향 오차: -0.152 m
- 최종 횡방향 오차: -0.009 m
- 최종 XY 오차: 0.152 m

최종 시점에는 GT보다 약 15.2 cm 뒤에 있으며 횡방향 오차는 약 0.9 cm로
거의 없다. 수정 전에는 전체 제곱 위치 오차 중 종방향 기여도가 약 92.8%로
지배적이었지만, 수정 후에는 종·횡방향 기여도가 42.8%와 57.2%로 바뀌었다.
즉 기존의 속도 의존적 종방향 지연은 대부분 제거됐고, 남은 위치 오차에서는
횡방향 성분도 비슷한 중요도를 갖는다.

### 11.5 Yaw 평가

| 항목 | 보정 전 | 보정 후 |
|---|---:|---:|
| 초기 정렬 yaw | -0.022 deg | -0.022 deg |
| Yaw 평균 절대오차 | 1.083 deg | 1.070 deg |
| Yaw 최대 절대오차 | 1.914 deg | 1.807 deg |

Yaw는 소폭 개선됐지만 약 1도의 편차가 여전히 남아 있다. Timestamp 보정은
GNSS position innovation의 시각을 맞추는 변경이며, 단일 안테나 GNSS로
heading을 직접 보정하는 변경이 아니므로 yaw 문제는 별도로 해결해야 한다.

### 11.6 선속도 평가

| 항목 | Bias | MAE | RMSE | 최대 절대오차 |
|---|---:|---:|---:|---:|
| `linear.x` | -0.552 m/s | 0.612 m/s | 0.663 m/s | 0.937 m/s |
| `linear.y` | +0.238 m/s | 0.566 m/s | 0.811 m/s | 2.058 m/s |
| `linear.z` | +0.246 m/s | 0.352 m/s | 0.439 m/s | 0.787 m/s |
| speed | +0.324 m/s | 0.672 m/s | 0.767 m/s | 1.769 m/s |

속력 MAE는 수정 전 0.747 m/s에서 0.672 m/s로 약 10.0% 감소했지만 여전히
큰 편이다. 최대 속력 오차는 오히려 1.651 m/s에서 1.769 m/s로 증가했다.
0.5 m/s 임계값을 넘는 구간도 8개이며, 여러 구간이 수십 초 동안 지속된다.
따라서 timestamp 보정 후에도 published velocity를 최종 planning/control
입력으로 확정하기에는 부족하다.

### 11.7 정지 구간 평가

| 구간 | 시간 | State net 이동 | State 누적 이동 | State 평균 속력 | State 최대 속력 |
|---|---:|---:|---:|---:|---:|
| 1 | 0.000–3.520 s | 0.020 m | 0.293 m | 0.042 m/s | 0.822 m/s |
| 2 | 27.240–32.480 s | 0.351 m | 3.038 m | 1.727 m/s | 1.869 m/s |
| 3 | 52.440–92.560 s | 0.328 m | 9.342 m | 0.821 m/s | 0.911 m/s |

정지 시작점과 끝점의 순변위는 각 구간에서 0.020–0.351 m 수준이지만 작은
위치 진동을 합한 누적 이동은 총 12.673 m이다. 수정 전 세 구간의 누적 이동
합계 5.752 m보다 약 2.2배 증가했다. 이는 차량이 한 방향으로 12.673 m
drift했다는 뜻이 아니라, 정지 중 반복적인 pose 보정과 진동이 경로 길이에
누적됐다는 뜻이다.

특히 두 번째 정지 구간의 평균 published speed는 1.727 m/s인데 net 이동은
0.351 m에 불과하다. Pose와 velocity state 사이의 불일치가 여전히 크므로,
다음 단계에서는 robust stationary detection과 ZUPT 또는 velocity-state
correction을 검토해야 한다.

### 11.8 각속도 평가

| 항목 | 결과 |
|---|---:|
| `angular.z` bias | -0.0003 rad/s |
| `angular.z` MAE | 0.0018 rad/s |
| `angular.z` RMSE | 0.0076 rad/s |
| `angular.z` 최대 절대오차 | 0.1958 rad/s |

0.1 rad/s 임계값을 넘은 구간은 14.180–14.280초의 한 구간뿐이다. 수정 전과
거의 같으므로 timestamp 보정은 yaw-rate 추정에 의미 있는 악영향을 주지
않았다.

### 11.9 최종 판단

- **Global pose:** 수정 전보다 개선됐으며 planning 통합 시험에 사용할 수 있다.
- **종방향 위치:** 기존 지연성 오차가 크게 줄어 timestamp 보정 효과가 확인됐다.
- **횡방향 위치:** 절대값은 작지만 현재 남은 위치 오차에서 차지하는 비중이 커졌다.
- **Yaw:** 약 1도 오차가 남아 있어 별도 분석이 필요하다.
- **Linear velocity:** 정지 중 잔류 속도와 pose 불일치가 해결되지 않았다.
- **Path smoothness:** GNSS correction에 따른 정지 중 pose jitter가 증가했다.
- **Yaw rate:** 짧은 spike 하나를 제외하면 양호하다.

따라서 timestamp 보정은 유지해야 한다. 다음 개선 우선순위는 timestamp를
다시 조정하는 것이 아니라, estimator velocity state와 정지 상태 pose를
일관되게 만드는 ZUPT·velocity 보정이다.
