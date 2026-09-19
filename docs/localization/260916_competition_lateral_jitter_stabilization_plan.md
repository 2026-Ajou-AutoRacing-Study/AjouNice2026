# 대회 측위 횡방향 흔들림과 제어 채터링 안정화 계획

마지막 갱신: 2026-09-19

> **최신 판정:** 이 문서 이후 GNSS 양자화, 현장 `velocity.x`, 로컬 MORAI
> 실시간율을 추가 검증했다. 현재 원인과 실행 우선순위는
> [`260918_competition_localization_critical_issue_status.md`](260918_competition_localization_critical_issue_status.md)를
> 우선한다. 특히 로컬 full-stack bag의 종방향 과적분을 production
> `velocity.x` bias로 해석하지 않는다. 2026-09-19에 GNSS sigma `0.10 m`와
> Global Qxy `0.0005` 조합을 production 기본값으로 채택했다. 정량 근거는
> [`260919_quantized_gnss_qr_tuning_evidence.md`](260919_quantized_gnss_qr_tuning_evidence.md)를
> 따른다.

## 1. 목적

현장점검에서 관측한 다음 현상을 대회 전에 분리하고 해결한다.

- 직선 주행에서도 차량의 map-frame 위치가 종·횡방향으로 미세하게 흔들린다.
- 특히 횡방향 흔들림과 함께 조향 명령이 좌우로 반복되는 채터링이 보인다.
- 원인이 GNSS 원시값, timestamp, EKF update, planning trajectory 또는 controller 중
  어디에서 시작되는지 아직 정량적으로 확정하지 않았다.

이 문제는 눈대중으로 controller gain만 낮추지 않는다. 동일 입력 bag에서 원인이
처음 나타나는 단계를 찾고, 측위 수정과 제어 수정을 분리해 A/B한다.

## 2. 현재 production 측위 계약

대회 production 후보는 GNSS+IMU+Competition GT CTRA-like provider다.

```text
/Competition_topic heading, velocity.x
  + /sensor/imu angular_velocity.z, linear_acceleration.x
  + /sensor/gps x/y
  -> local/global robot_localization EKF
  -> /localization/kinematic_state
```

유지하기로 확정한 기준은 다음과 같다.

- 대회 GT source: `competition`.
- heading correction: 활성화.
- heading offset: `-1.347 deg`.
- `publish_sensor_tf=false`로 sensing과 static sensor TF 중복을 피한다.
- duplicate/out-of-order input timestamp는 거부한다.
- local/global EKF는 계속 `two_d_mode=true`를 사용한다.

공통 alias는 다음 두 진입점으로 분리했다. generic launch의 기본값은 기존 bag과 직접
실행 호환성을 위해 `ego`로 남기되, 운영자는 인자를 생략한 raw launch를 production
명령으로 사용하지 않는다.

- `localization`: 대회 production, `gt_source=competition`을 명시적으로 고정.
- `localization_mrp`: MRP(2026 개인용 launcher), `gt_source=ego`를 명시적으로 고정.

두 alias 모두 heading `-1.347 deg`와 `publish_sensor_tf=false`를 명시한다. 하나의
provider만 실행한다. `/Ego_topic` 직접 GT provider는 대회 fallback이 아니다.
duplicate timestamp 차단은 여러 localization provider의 동시 발행을 막는 기능이
아니다.

## 3. 현재 설정에서 가장 유력한 원인

이 계획을 작성했을 때의 noise-off profile은 GNSS 수평 표준편차를 `0.05 m`로
두었다. 2026-09-19 Q/R 재평가 후 현재 production은 `0.10 m`를 사용한다.

```yaml
gnss:
  horizontal_stddev_m: 0.05
```

따라서 GPS 좌표가 양자화되어 일정 시간 정지했다가 계단식으로 변하거나 순간 jump가
있으면 Global EKF가 그 측정을 강하게 따라갈 수 있다. GT heading과 종방향 속도가
매끄러워도 map-frame x/y correction이 횡방향 pose 흔들림을 만들 수 있다.

현재 우선순위 가설은 다음과 같다.

1. `0.05 m` GNSS covariance가 실제 현장 GNSS의 양자화·jitter보다 지나치게 작다.
2. GPS의 계단식 위치 변화 또는 outlier가 그대로 Global EKF innovation으로 들어간다.
3. 센서 receipt-time은 통일했지만 packet 간격과 callback latency 변화가 남아 있다.
4. Localization은 안정적이지만 Planning trajectory 자체가 frame마다 흔들린다.
5. Pose와 trajectory는 안정적이지만 controller lookahead/gain이 작은 오차에 과민하다.

Heading `-1.347 deg`는 고정 편향 보정이다. 값이 틀리면 한쪽으로 지속적으로 치우치는
오차를 만들 가능성이 크며, 좌우로 번갈아 나타나는 고주파 채터링의 1순위 원인은 아니다.

## 4. 진단용 기록 계약

직선, 완만한 곡선, 정지 후 출발을 포함하는 동일 주행에서 다음 토픽을 함께 기록한다.

```text
/sensor/gps
/sensor/imu
/Competition_topic
/localization/gnss/odom
/localization/odometry/local
/localization/odom
/localization/kinematic_state
/tf
/tf_static
/planning/trajectory
/control/command/control_cmd
/control/command/steering
/purepursuit_autoware/debug/lateral_error
/purepursuit_autoware/debug/lookahead_distance
/purepursuit_autoware/debug/target_curvature
/purepursuit_autoware/debug/limited_steering_angle
```

가능하면 같은 구간의 lane centerline과 Ego GT도 보존한다. 위치 오차는 map x/y만 보지
않고 lane tangent 기준 longitudinal/lateral 성분으로 변환한다.

단계별 판정은 다음과 같다.

| 처음 이상이 보이는 위치 | 우선 원인 |
|---|---|
| `/sensor/gps` | 원시 GNSS 양자화·jump·packet timing |
| `/localization/gnss/odom` | 좌표변환·offset·timestamp |
| Global odom만 흔들리고 local odom은 안정 | GNSS Kalman update gain |
| Localization은 안정, trajectory가 흔들림 | Planning path 재계산 |
| Pose와 trajectory는 안정, steering만 흔들림 | Controller tuning |

다음 값을 최소한 보고한다.

- GPS x/y increment의 plateau 길이와 jump 분포.
- GNSS update 직전·직후 Global EKF pose 변화.
- lane 기준 lateral error mean/std/p95/max.
- lateral velocity와 lateral jerk의 p95/max.
- steering command std, peak-to-peak, 초당 부호 반전 횟수.
- localization lateral jump와 steering 변화의 시차 상관관계.
- 곡선에서의 정상 추종 lag와 직선 채터링을 분리한 결과.

## 5. 수정 실험 우선순위

### L0. 과거 baseline 재현 — 완료

- GNSS horizontal stddev `0.05 m`인 과거 baseline.
- 현장과 같은 UDP timestamp 경로.
- 현재 controller 설정.
- 같은 bag 또는 반복 가능한 같은 주행을 모든 A/B에 사용한다.

### L1. GNSS measurement covariance 및 Global Qxy sweep — 완료

첫 수정 실험이며 다른 파라미터는 고정한다.

```text
0.05 m  historical baseline
0.10 m
0.20 m  first candidate
0.30 m
```

횡방향 채터링만 줄고 장기 절대 위치, 곡선 추종 및 GNSS 음영 복구가 손상되지 않는
값을 선택한다. 2026-09-19 정상 시간축 replay까지 마친 결과 `0.10 m`와 Global
Qxy `0.0005`를 production 기본값으로 채택했다. 현장 closed-loop 검증은 남아 있다.

### L2. GNSS innovation/outlier gate

L1만으로 불연속 jump가 남을 때 수행한다.

- `robot_localization` pose rejection threshold A/B.
- 시간 간격, GT vx와 heading으로 설명할 수 없는 position innovation 거부.
- GNSS 음영 복귀 직후 큰 correction을 한 번에 적용하지 않는 정책 검토.
- 거부 횟수와 최대 연속 거부 시간을 반드시 로그로 남긴다.

정상적인 급회전 또는 checkpoint 재배치를 outlier로 오인하지 않도록 별도 처리한다.

### L3. Global correction smoothing 또는 local/global 역할 분리

L1과 L2 이후에도 Global EKF correction이 제어에 직접 전달되어 문제가 남을 때만
검토한다.

- Planning과 route에는 global map pose를 사용한다.
- 빠른 제어 feedback에는 smooth local odometry 사용을 검토한다.
- map trajectory를 local frame으로 변환할 때 동일 시각 TF 계약을 지킨다.

이 단계는 frame contract 변경 범위가 크므로 대회 직전 임시 수정으로 수행하지 않는다.

### L4. Controller 완화

Localization과 trajectory가 충분히 안정적인 것이 확인된 뒤에만 수행한다.

- 속도별 lookahead distance.
- steering command rate/jerk 제한.
- lateral error 또는 curvature smoothing.
- gain과 saturation A/B.

최종 pose에 단순 LPF를 먼저 넣지 않는다. 곡선에서 phase lag가 생기고 TF와 state가
불일치할 수 있다. GPS를 높은 주기로 선형 보간하는 것도 새 정보를 추가하지 않으며,
현재 EKF는 이미 `predict_to_current_time=true`, 50 Hz로 중간 상태를 예측한다.

## 6. 채택 기준

수정안은 직선 조향이 눈으로 덜 흔들린다는 이유만으로 채택하지 않는다.

- 직선 lateral error std/p95와 steering sign reversal이 baseline보다 감소한다.
- 곡선 lateral error p95와 최대값이 악화되지 않는다.
- lane 중심에 대한 지속 bias가 증가하지 않는다.
- GNSS 음영 진입·유지·복귀에서 pose jump가 허용 범위 안이다.
- 정지·재출발과 checkpoint 재배치에서 과거 상태를 길게 유지하지 않는다.
- `/localization/kinematic_state`, TF, planning trajectory의 timestamp가 단조 증가한다.
- 제어 주기와 localization 50 Hz 출력이 유지된다.

수치 threshold는 현장 baseline bag을 분석한 뒤 고정한다. baseline 없이 임의 threshold를
먼저 정하지 않는다.

## 7. 실행 순서

1. 대회와 동일한 네트워크·timestamp 설정으로 짧은 진단 bag을 확보한다.
2. L0에서 원인이 처음 나타나는 topic을 확인한다.
3. 같은 bag으로 L1 `0.05/0.10/0.20/0.30 m`를 재생 평가한다.
4. L1 최적값으로 live 직선·곡선·정지 출발을 확인한다.
5. 불연속 outlier가 남을 때만 L2를 수행한다.
6. Localization이 안정된 이후에만 Planning과 Controller를 각각 A/B한다.
7. 채택값과 rollback 명령을 competition runbook에 반영한다.

## 8. 아직 확정하지 않은 사항

- 현장 GPS가 실제로 몇 cm 단위로 양자화되는지.
- 채터링의 주성분이 GPS jump인지 Planning trajectory 변화인지.
- 채택 Q/R이 실제 LAN-to-LAN closed-loop에서 조향 채터링을 얼마나 낮추는지.
- outlier gate가 필요한지.
- control 입력을 local odometry로 분리할 가치가 있는지.

위 항목은 진단 bag과 동일 조건 A/B 전에는 사실로 표현하지 않는다.

## 9. 2026-09-16 full-route baseline과 L1 결과

입력 bag은 `260916_150411_full_route_mr_baseline.bag`이다. 약 522초이며 정상 종료된
LZ4 bag이다. MORAI, BEVFusion, planning, control을 한 PC에서 함께 실행해 센서 주기가
낮아졌지만, 현장 sensor-local bag과 다음 핵심 특성이 거의 같아 localization A/B에
사용할 수 있다.

- 정상 GNSS 구간 lateral error p95: local `7.46 cm`, 현장 `8.01 cm`.
- lateral frame jump p95: local `5.40 cm`, 현장 `5.24 cm`.
- steering-rate abs p95: local `1.60 rad/s`, 현장 `1.59 rad/s`.
- 현장 GT localization의 steering-rate abs p95는 `0.34 rad/s`였다.

따라서 문제의 주성분은 큰 지속 편향보다는 GNSS update를 강하게 따라가는 고주파
global correction이다. 공분산 sweep은 perception/planning/controller를 제외하고 동일
GPS·IMU·GT 입력으로 localization만 실제 시간 1배속 재계산했다. `0.05 m` replay가 live
결과를 1 mm 이내 수준으로 재현한 것을 확인한 뒤 비교했다.

| GNSS horizontal sigma | lateral error p95 | lateral jump p95 | longitudinal error p95 | longitudinal jump p95 |
|---:|---:|---:|---:|---:|
| `0.05 m` | `7.48 cm` | `5.46 cm` | `34.14 cm` | `17.68 cm` |
| `0.10 m` | `6.77 cm` | `3.49 cm` | `55.08 cm` | `16.33 cm` |
| `0.20 m` | `7.03 cm` | `2.26 cm` | `108.37 cm` | `15.59 cm` |
| `0.30 m` | `8.96 cm` | `1.76 cm` | `163.75 cm` | `15.18 cm` |

판정은 다음과 같다.

- `0.10 m`는 lateral jump p95를 약 36% 낮추면서 lateral error도 개선한 유일한 균형점이다.
- `0.20 m` 이상은 횡점프는 더 낮지만 종방향 lag가 1 m 이상이어서 탈락한다.
- 이 초기 sweep 이후 정상 시간축 `/Competition_topic` bag에서 Global Qxy까지 함께
  재탐색했고, sigma `0.10 m` + Qxy `0.0005`를 production 기본값으로 채택했다.
  다음 단계는 live 직선·곡선·정지/출발과 controller 지표 확인이다.
- GPS 음영에서 `/localization/gnss/odom`이 약 `8.17 s` 끊겼고 복귀 순간 baseline은
  longitudinal 약 `9.62 m`, lateral 약 `0.53 m`를 한 frame에 보정했다. `0.10 m`도 각각
  약 `9.51 m`, `0.65 m`로 근본 해결되지 않는다. 정상구간 covariance와 음영 복귀
  재정합 정책은 별도 문제로 유지한다.

초기 sweep 두 번은 고정 ROS master port 재사용으로 orphan process의 `/clock`이 섞여
폐기했다. sweep script는 실행마다 고유 port를 사용하고, 새 roscore가 실제로 시작되지
않으면 즉시 실패하도록 수정했다.
