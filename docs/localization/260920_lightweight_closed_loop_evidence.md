# 경량 GT-object 폐루프 Localization 근거

상태: **EVIDENCE**
평가일: 2026-09-20

## 결론

카메라·LiDAR·신호등 인지를 끄고 MORAI GT object만 detector 대체 입력으로 사용한
폐루프의 정상 GNSS·이동 구간에서 production Q/R인 `GNSS sigma=0.10 m`, Global
`Qxy=0.0005`는 50 Hz localization과 낮은 종·횡 오차 및 frame jump를 보였다. 그러나
마지막에는 GPS 음영 안에서 차량이 정지했고, 당시 구현은 stale GNSS z 때문에
controller-facing state를 약 27.31초 동안 발행하지 못했다. 따라서 이 bag을
“localization이 끝까지 50 Hz로 완주한 결과”로 해석하지 않는다. 같은 경량 환경의
`sigma=0.05 m`, `Qxy=0.05` preset은 별도 주행임을 감안해도 위치 jump와 조향률이
크게 나빠 production으로 채택하지 않는다.

이 결과는 현장 LAN-to-LAN 승인 자체는 아니다. 주행 구간이 완전히 같지 않고,
2026-09-20에 반영한 새 Pure Pursuit steering rate limiter와 deadband 적용 전 bag이다.

## 실험 계약

```text
MORAI GPS + IMU + Competition/Ego/Object
  -> production GNSS+IMU+GT-motion localization
MORAI Object GT
  -> canonical detection -> production tracker -> Map-CV
traffic-light inference/planner plugin OFF
camera/LiDAR generation OFF
RViz + planning + controller ON
```

대회 fallback 계약이 아니다. `/Object_topic`과 `/Ego_topic`은 로컬 부하 분리용으로만
사용했고 localization 입력 motion은 `/Competition_topic`이다. 분석은
`scripts/evaluation/localization/analyze_localization_stability_bag.py`로 moving speed
`>1.0 m/s` 구간을 평가했다.

## Production Q/R 장주행

입력 bag:

```text
rosbags/eval/localization/local_mr_stability/
  260919_231906_q0005_r010_gt_object_lightweight_no_traffic.bag
```

| 항목 | 결과 |
|---|---:|
| 전체 / moving 시간 | `242.72 / 156.05 s` |
| Competition / GPS / localization active rate / trajectory | `49.81 / 29.89 / 50.00 / 9.83 Hz` |
| 횡 오차 abs p50 / p95 | `1.79 / 5.56 cm` |
| 종 오차 abs p50 / p95 | `1.77 / 6.28 cm` |
| 횡 frame jump abs p50 / p95 | `0.21 / 1.31 cm` |
| 종 frame jump abs p50 / p95 | `1.88 / 7.53 cm` |
| yaw 오차 abs p50 / p95 | `0.007 / 0.067 deg` |
| 조향률 abs p50 / p95 | `0.082 / 0.541 rad/s` |
| 유효 조향 부호 반전 | `0.500 /s` |

후속 oracle timing 감사에서 `/Ego_topic`의 종방향 프레임 운동학 residual p95가
`12.08 cm`로 확인됐다. 따라서 이 표의 종 frame jump `7.53 cm`는 localization
단독 연속성 지표가 아니며 채택 근거에서 제외한다. 횡 frame jump, 위치 오차, yaw,
실제 조향 지표는 유효하다.

### 마지막 GPS 음영과 정지

- zero-GNSS 시작부터 bag 종료까지: `27.746 s`.
- 음영 진입 속도: 약 `16.47 m/s`.
- 음영 구간의 약 `90.2%`는 속도 `0.2 m/s` 미만.
- 마지막 `20 s`는 사실상 완전 정지.
- Global EKF `/localization/odom`은 bag 끝까지 `50 Hz`로 발행.
- 당시 `/localization/kinematic_state`는 음영 진입 약 `0.44 s` 뒤 중단되고,
  bag 종료 `27.31 s` 전부터 message가 없다.
- Planning trajectory와 control command는 계속 발행됐지만 localization state가
  끊겼으므로, 정지 원인을 판단 알고리즘 단독 문제로 확정할 수 없다.

이 구간은 GPS가 복귀하기 전에 bag이 종료되므로 재획득 correction 평가에는 쓰지
않는다. 반면 GPS 음영 진입 후 controller-facing state 연속성 회귀에는 강한 실제
폐루프 근거다.

여기서 localization `50.00 Hz`는 message가 존재하는 active 구간의 주기다. bag 전체
연속성을 뜻하지 않는다. 속도 `>1 m/s`의 `7,772` matched sample 역시 state가 존재했던
구간만 포함한다.

현장 baseline fusion의 조향률 abs p95 `1.59 rad/s`보다 작지만, detector 부하,
route, planning/control revision이 다르므로 이를 Q/R만의 `66%` 개선으로 해석하지
않는다. 이 bag의 강한 결론은 GPS 음영 진입 전의 경량 정상 시간축에서 production
estimator가 저율화되지 않고 안정적으로 동작했다는 것이다. Bag 전체 완주나 GPS 음영
대응을 입증하는 결과로 확대 해석하지 않는다.

## 실험 preset 비교

`sigma=0.05 m`, `Qxy=0.05` bag은 103.53초, moving 83.66초이며 production
장주행과 동일 route slice는 아니다.

| moving p95 | production Q/R 장주행 | `sigma=0.05, Qxy=0.05` |
|---|---:|---:|
| 횡 위치 오차 | `5.56 cm` | `7.06 cm` |
| 종 위치 오차 | `6.28 cm` | `8.92 cm` |
| 횡 frame jump | `1.31 cm` | `5.15 cm` |
| 종 frame jump | `7.53 cm` | `9.63 cm` |
| yaw 오차 | `0.067 deg` | `0.090 deg` |
| 조향률 | `0.541 rad/s` | `1.248 rad/s` |
| 조향 부호 반전 | `0.500 /s` | `1.434 /s` |

실험 preset은 GNSS cell transition을 훨씬 강하게 controller-facing state로 전달했다.
폐루프 정성 관측과 정량 지표가 모두 production Q/R 유지 결론과 일치한다.

## 짧은 반복 확인

다음 production Q/R bag은 52.20초, moving 32.59초의 짧은 반복이다.

```text
260920_003403_q0005_r010_gt_object_lightweight_no_traffic_v2_test.bag
```

- localization `49.999 Hz`.
- 횡/종 위치 오차 p95 `5.42/5.36 cm`.
- 횡/종 frame jump p95 `1.26/6.99 cm`.
- 조향률 p95 `0.601 rad/s`.

장주행과 localization 수치가 유사해 repeatability를 보조한다. 다만 이 bag도 최신
조향 limiter pull 전에 생성됐으므로 controller 변경 검증으로 사용하지 않는다.

## 남은 승인 조건

1. 최신 controller(`min lookahead=3.5 m`, steering rate `40 deg/s`, deadband
   `0.2 deg`)와 production Q/R을 같은 LAN-to-LAN 주행에서 검증한다.
2. raw Competition/GPS/IMU, EKF 중간 상태, trajectory와 steering을 함께 기록한다.
3. GPS 음영 진입·복귀는 정상 GNSS 구간과 별도 slice로 평가한다.
4. `GNSS z age` 경고가 센서 종료 뒤에만 발생했는지 확인한다. 음영 중 output 중단으로
   재현되면 z 유지 정책을 별도 수정한다.
5. 위 검증 전에는 quantization-aware update나 local-control-state 구조로 확장하지
   않는다.
