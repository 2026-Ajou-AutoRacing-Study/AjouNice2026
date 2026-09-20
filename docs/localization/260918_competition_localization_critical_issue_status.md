# 대회 Localization 중대 이슈 현황과 해결 기준

마지막 갱신: 2026-09-20

이 문서는 대회용 GNSS+IMU+GT motion localization에서 관측한 위치 흔들림과
제어 채터링의 최신 사실, 반증된 가설, 남은 위험 및 다음 실험 계약을 한곳에
고정한다. 과거 채팅이나 단일 실험 결과보다 이 문서를 우선한다.

## 1. 현재 심각도와 목표

현장 주행에서 GT localization은 부드러웠지만 현재 fusion localization은 직선에서도
조향이 좌우로 반복되는 채터링을 보였다. 현장 bag에서 moving steering-rate absolute
p95는 다음과 같았다.

| 상태 | steering-rate absolute p95 |
|---|---:|
| GNSS+IMU+GT motion localization | 약 `1.59 rad/s` |
| MORAI GT localization | 약 `0.34 rad/s` |

이는 RViz 미관 문제가 아니라 제어 안정성 문제다. 목표는 종·횡 위치 정확도를 유지하면서
GNSS update가 만드는 고주파 pose correction이 조향 명령으로 전달되지 않게 하는 것이다.

## 2. Production 측위 계약

```text
/Competition_topic heading, velocity.x
  + /sensor/imu angular_velocity.z, linear_acceleration.x
  + /sensor/gps x/y
  -> local/global robot_localization EKF
  -> /localization/kinematic_state
  -> planning / controller
```

고정된 운영 계약은 다음과 같다.

- 대회 alias: `localization`, `gt_source=competition`.
- MRP alias: `localization_mrp`, `gt_source=ego`.
- heading correction: `-1.347 deg`.
- local/global EKF: `two_d_mode=true`.
- `publish_sensor_tf=false`.
- 대회 UDP: `/use_sim_time=false`, ROS host receipt-time policy 유지.
- 대회에서는 `/Ego_topic`과 `/Object_topic`을 기대하지 않는다.
- 대회 운영자는 localization provider를 정확히 하나만 실행한다.
- GNSS horizontal sigma: `0.10 m`.
- Global EKF x/y process-noise diagonal: `0.0005`.
- `predict_to_current_time=true`.
- 개발 중 최초 start pose는 `/Ego_topic`을 사용하고 대회 전 공식 interface로
  교체한다.

대회 운영진에게 GNSS와 IMU 모두 인위적인 랜덤 노이즈를 넣지 않는다는 답변을
확인했다. 그러나 GNSS의 좌표 양자화와 GPS 음영은 랜덤 노이즈와 별개로 남는다.

## 3. 확정된 정상 GNSS 구간 문제

### 3.1 원시 GNSS 양자화

MORAI NMEA GNSS의 위도·경도 격자는 약 `1.6667e-6 deg`다. K-City 위도에서
환산하면 다음과 같다.

| 축 | 한 격자 |
|---|---:|
| 동서 | 약 `14.8 cm` |
| 남북 | 약 `18.5 cm` |
| 두 축 동시 전이 | 약 `23.7 cm` |

무잡음이라는 말은 좌표가 연속적이라는 뜻이 아니다. 실제 값은 같은 격자에 머물다가
다음 격자로 계단식 전이한다. 단일 GNSS 샘플만으로는 격자 내부의 실제 sub-cell 위치를
복원할 수 없다.

### 3.2 Global EKF가 계단 입력을 강하게 따라감

과거 baseline은 GNSS horizontal sigma를 `0.05 m`로 사용했다. 정상 구간에서 EKF
50 Hz 출력 interval을 GNSS update 유무로 나눈 결과는 다음과 같다.

| 횡방향 error jump | `0.05 m` | `0.10 m` |
|---|---:|---:|
| prediction-only MAE | `0.142 cm` | `0.121 cm` |
| GNSS update MAE | `2.94 cm` | `1.98 cm` |
| GNSS update p95 | `8.27 cm` | `5.18 cm` |

`0.05 m`에서 GNSS update 순간 횡 jump는 prediction-only보다 약 21배 크다.
GNSS innovation이 다음 EKF correction에 반영되는 경험적 비율과 상관도 다음처럼
측정됐다.

| GNSS sigma | 경험적 correction 비율 | innovation-correction 상관계수 |
|---|---:|---:|
| `0.05 m` | 약 `0.677` | `0.971` |
| `0.10 m` | 약 `0.473` | `0.976` |

따라서 정상 GNSS 구간의 횡방향 채터링 주원인은 IMU나 motion prediction 자체의
랜덤 흔들림이 아니라, 시간 상관된 양자화 GNSS를 반복해서 강하게 반영하는 Global
EKF correction이다.

2026-09-19에는 GNSS sigma `0.10 m`와 Global EKF x/y process noise `0.0005`를
결합한 Q/R 후보를 정상 시간축 `/Competition_topic` bag에서 1x 재검증했다. 횡 jump
p95는 `5.432 -> 1.444 cm`, 종 jump p95는 `7.234 -> 2.416 cm`로 감소했고 종·횡
위치 오차도 함께 개선되어 production 기본값으로 채택했다. 상세 표와 한계는
[`260919_quantized_gnss_qr_tuning_evidence.md`](260919_quantized_gnss_qr_tuning_evidence.md)를
따른다.

### 3.3 Controller가 Global correction을 직접 봄

Local EKF는 GNSS가 없는 smooth `odom -> base_link` 상태를 만들지만, 현재 controller가
소비하는 `/localization/kinematic_state`는 Global EKF 결과다. 따라서 dual-EKF를
사용해도 제어 입력 관점에서는 GNSS correction이 격리되지 않는다.

```text
quantized GNSS update
  -> Global EKF map pose correction
  -> /localization/kinematic_state
  -> lateral error 변화
  -> steering correction / chatter
```

## 4. `velocity.x bias` 결론 정정

로컬 full-stack bag 하나에서는 Ego 위치 이동거리가 `2139.41 m`, `velocity.x`
적분거리가 `2608.36 m`여서 겉보기 과대 적분이 `21.9%`였다. 이를 처음에는
`velocity.x` 자체의 bias로 해석했지만 현장 LAN bag으로 반증했다.

| 현장 bag | 위치 이동거리 | `velocity.x` 적분거리 | 비율 |
|---|---:|---:|---:|
| localization 1 | `2264.56 m` | `2264.78 m` | `0.999905` |
| localization 2 | `2265.40 m` | `2265.42 m` | `0.999994` |
| localization 3 | `2248.55 m` | `2248.54 m` | `1.000006` |
| GT 주행 | `2195.38 m` | `2195.80 m` | `0.999806` |

판정은 다음과 같다.

- 현장 `/Ego_topic.velocity.x`는 위치 이동과 사실상 정확히 일치한다.
- 로컬 bag의 21.9% 차이는 `velocity.x` 고정 bias가 아니다.
- 로컬에서 MORAI, BEVFusion, planning을 한 PC/GPU에서 실행해 MORAI가 wall time의
  약 `0.82` 비율로 진행했는데, UDP header는 ROS receipt wall time을 사용한 시간축
  불일치로 판단한다.
- 같은 로컬 bag의 `/Competition_topic`과 `/Ego_topic` heading 및 `velocity.x`는
  샘플별 최대 차이가 정확히 0이었다.
- 현장 bag에는 `/Competition_topic`이 기록되지 않았으므로 다음 현장 기록에서 직접
  최종 확인한다.

따라서 production에서 `velocity.x`에 `0.9`, `0.85` 등의 scale을 곱하지 않는다.
과거 로컬 bag에서 관측한 GPS 음영 중 약 10 m 종방향 drift도 production velocity
bias의 증거로 사용하지 않는다.

## 5. Timestamp와 로컬 MORAI 실시간율

현재 UDP receiver는 패킷의 remote `sec/nsec` 대신 `ros::Time::now()`를 메시지
header에 사용한다. 이는 현장 MORAI PC와 알고리즘 PC의 clock offset 때문에 발생한
TF·freshness 문제를 해결한 production 정책이다.

```text
대회: /use_sim_time=false + ROS host receipt time
```

이 정책을 통째로 원복하지 않는다. 단순 원복 시 Ego/Competition/IMU/Camera는 remote
clock을 쓸 수 있지만 GPS와 LiDAR가 다른 시간축에 남아 혼합 clock 문제가 재발할 수
있다.

로컬 한-PC 개발에서는 다음 두 경로를 순서대로 검증한다.

1. BEVFusion 대신 MORAI GT detection을 사용해 GPU 경쟁을 제거한다.
2. MORAI `/clock`이 실제 physics simulation time이고 충분히 빠르면
   `/use_sim_time=true`인 로컬 전용 runtime을 검토한다.

단순히 `/clock` 토픽을 기록하는 것만으로는 시간축이 바뀌지 않는다. 모든 노드를
`/use_sim_time=true` 설정 이후 시작해야 `ros::Time::now()`가 simulation time을
반환한다. `/clock` rate가 낮으면 duplicate stamp가 생길 수 있으므로 먼저 측정한다.

추가해야 할 비침투 진단은 다음과 같다.

```text
/morai/remote_clock
/morai/time_sync_status
  - remote sec/nsec
  - ROS receipt time
  - remote-receipt offset
  - delta_remote / delta_receipt (effective real-time factor)
  - monotonicity, packet gap
```

진단 단계에서는 canonical `/clock`을 새로 소유하지 않는다.

## 6. GPS 음영과 복귀

GPS가 `(0, 0)`으로 들어오는 음영 구간은 adapter가 거부한다. 하지만 정상 좌표가
다시 들어온 첫 샘플을 일반 GNSS update와 동일하게 처리한다.

현재 없는 기능은 다음과 같다.

- reacquisition 상태기계.
- 복귀 직후 covariance inflation.
- 큰 innovation gradual realignment.
- 일반 음영 복귀와 checkpoint 순간이동 구분.
- correction rate/step 제한과 진단 로그.

로컬 과부하 bag의 약 9.5~10 m 복귀 jump는 시간축 artifact에 오염되어 있으므로
production 크기 근거로 사용하지 않는다. GPS 음영 정책은 real-time factor가 정상인
새 bag에서 별도 평가한다.

## 7. 현재 해결 우선순위

### P0. 시간축이 정상인 localization test bed 확보 — 완료

BEVFusion을 끄고 MORAI GT detection adapter를 사용한다.

```text
/Object_topic
  -> morai_gt_detection
  -> canonical detections
  -> 기존 EKF tracking
  -> UNKNOWN stationary policy
  -> 기존 Map-CV
  -> planning / control
```

GT adapter는 detector velocity를 전달하지 않으므로 tracker와 prediction은 production과
동일한 동작을 유지한다. 현재 분리 실행 계약에서 `planning`은 planning node만
소유하므로 `od_gt`, `tracking`, `planning`을 각각 한 번씩 실행한다.

테스트 bed 승인 조건:

- `Ego position 누적 이동거리 / integral(velocity.x dt)`가 `0.98~1.02`.
- localization, planning, control topic rate가 유지됨.
- detection publisher가 `morai_gt_detection` 하나임.
- `/Object_topic`은 이 로컬 실험에서만 사용하며 대회 fallback으로 취급하지 않음.

2026-09-20 경량 폐루프에서 production Q/R 장주행은 localization `50.00 Hz`, 횡/종
위치 오차 p95 `5.56/6.28 cm`, 횡/종 frame jump p95 `1.31/7.53 cm`, 조향률 p95
`0.541 rad/s`를 기록했다. `sigma=0.05 m`, `Qxy=0.05` preset은 횡 frame jump
`5.15 cm`, 조향률 `1.248 rad/s`로 악화했다. production Q/R을 유지한다. 상세 근거와
비교 한계는
[`260920_lightweight_closed_loop_evidence.md`](260920_lightweight_closed_loop_evidence.md)를
따른다.

한 PC 로컬 test bed에서는 다음 경량 계약을 사용한다.

```text
MORAI Sensor_localization_gt_lightweight.json
  -> GPS + IMU only (Camera/LiDAR 생성 안 함)
Ego Network
  -> Ego + Competition + Object + control 유지
od_gt -> tracker/Map-CV -> planner -> controller
Traffic-light inference OFF
TrafficLightModulePlugin OFF
RViz ON
```

실행 진입점은 `udp_localization_light`, `sensing_localization_light`, `localization`,
`od_gt`, `tracking`, `planning enable_traffic_light_planner:=false`, `planviz`,
`traj_fol_a`다. `enable_traffic_light_planner`의 기본값은 계속 `true`이므로 대회
production 판단 동작은 바뀌지 않는다. 이 구성은 신호를 의도적으로 무시하므로
localization/control 부하 실험에서만 사용한다.

### Closed-loop localization preset

기본 `localization`은 sigma `0.10 m`, Global Qxy `0.0005`를 계속 사용한다. 과거
폐루프에서 상대적으로 양호했던 두 조합은 다음 별도 alias로만 재현한다.

| alias | GNSS horizontal sigma | Global Qxy | 상태 |
|---|---:|---:|---|
| `localization` | 0.10 m | 0.0005 | production 기본값 |
| `localization_sigma_0p10_qxy_0p05` | 0.10 m | 0.05 | closed-loop 실험 |
| `localization_sigma_0p05_qxy_0p05` | 0.05 m | 0.05 | closed-loop 실험 |

실험 alias는 estimator 구조, `/Competition_topic`, heading correction `-1.347°`를
바꾸지 않는다. 한 주행에는 하나의 provider만 실행하고 동일 route·controller·MORAI
실시간율 조건으로 비교한다.

### P1. 채택 Q/R의 현장 closed-loop 확인

정상 시간축 offline A/B와 로컬 경량 폐루프는 완료했으며 Q/R을 production 기본값으로
유지한다. 다음 LAN-to-LAN 주행에서는 새 Pure Pursuit limiter까지 포함한 controller
설정을 고정하고 baseline 현장 bag의 steering-rate absolute p95 `1.59 rad/s`보다
감소하는지 확인한다. 새 bag에는 raw Competition, GPS, IMU와 EKF 중간 토픽을 반드시
함께 기록한다.

### P2. 필요할 때만 explicit quantization-aware GNSS update

채택 Q/R 후에도 현장 채터링이 남을 때만 GNSS random denoising이 아니라 양자화
계약을 명시적으로 모델링한다.

- 동일 cell의 반복 관측을 독립적인 새 정보로 계속 누적하지 않는다.
- 예측 위치가 현재 GNSS cell 범위 안이면 correction을 생략하거나 최소화한다.
- cell 범위를 벗어났을 때 초과분만 제한적으로 보정한다.
- cell transition에서 pose step, innovation, 적용 correction을 기록한다.

대략적인 cell half-width는 동서 `7.4 cm`, 남북 `9.3 cm`다. 실제 구현은 위경도
양자화 축과 map projection을 반영해야 하며, 차량 heading 기준 임의 deadband로
대체하지 않는다.

### P3. GPS reacquisition 정책

P0~P2 정상 구간을 해결한 뒤 음영 복귀를 분리한다. 큰 innovation을 무조건 거부하면
진짜 checkpoint 재배치나 장기 drift를 복구하지 못하므로 상태기계와 timeout이 필요하다.

### P4. Global/local 제어 상태 분리

위 조치 후에도 correction이 제어기를 흔들면 planning에는 global map pose를 유지하고
빠른 제어 feedback에는 smooth local state를 사용하는 구조를 검토한다. 이는 frame과
trajectory 변환 계약 변경이 크므로 마지막 구조 변경이다. Competition Vehicle Status의
확장된 motion 정보가 이 방향을 강화하지만, 현재도 heading과 `velocity.x`는 이미 사용
중이므로 production을 즉시 바꾸지는 않는다. 구체적인 frame 계약과 A/B는
[`260920_competition_status_and_local_control_frame_plan.md`](260920_competition_status_and_local_control_frame_plan.md)를
따른다.

### P5. Controller tuning

Localization pose와 planning trajectory가 안정된 뒤에만 lookahead, steering rate,
gain을 조정한다. localization jump를 controller gain으로 덮지 않는다.

## 8. 하지 않을 것

- production UDP receipt-time 정책을 근거 없이 제거하지 않는다.
- `velocity.x`에 임의 scale을 적용하지 않는다.
- GNSS/IMU가 무잡음이라는 이유로 covariance를 0으로 설정하지 않는다.
- 최종 pose에 단순 LPF를 먼저 적용하지 않는다.
- 과부하된 로컬 bag의 종방향 오차를 production sensor bias로 해석하지 않는다.
- `/Ego_topic` 또는 `/Object_topic`을 대회 fallback으로 사용하지 않는다.
- GPS 음영 복귀의 큰 innovation을 무기한 거부하지 않는다.

## 9. 다음 기록 계약

다음 local GT-object full-stack bag에는 최소한 다음을 포함한다.

```text
/clock                         # 존재할 때
/morai/remote_clock            # 진단 구현 후
/morai/time_sync_status        # 진단 구현 후
/Ego_topic
/Competition_topic             # 존재할 때
/Object_topic
/sensor/gps
/sensor/imu
/localization/gnss/odom
/localization/odometry/local
/localization/odom
/localization/kinematic_state
/tf
/tf_static
/perception/object_recognition/detection/objects
/perception/object_recognition/tracking/objects
/perception/object_recognition/objects
/planning/trajectory
/control/command/steering
/ctrl_cmd
/purepursuit_autoware/debug/lateral_error
/purepursuit_autoware/debug/lookahead_distance
/purepursuit_autoware/debug/target_curvature
/purepursuit_autoware/debug/limited_steering_angle
```

정상 직선, 완만한 곡선, 정지·재출발, GPS 음영 진입·복귀를 구간별로 분리해 보고한다.

## 10. 증거 위치

- 현황 이전 계획:
  `docs/localization/legacy/260916_competition_lateral_jitter_stabilization_plan.md`
- 최신 경량 폐루프 근거:
  `docs/localization/260920_lightweight_closed_loop_evidence.md`
- UDP timestamp 계약:
  `docs/localization/morai_udp_receipt_time.md`
- 현장 bag과 분석:
  `rosbags/eval/localization/260915_field_inspection/`
- 로컬 baseline 및 `0.10 m` bag:
  `rosbags/eval/localization/local_mr_stability/`
- covariance replay:
  `rosbags/eval/localization/gnss_covariance_sweep_260916_154603/`
- 분석 코드:
  `scripts/evaluation/localization/analyze_localization_stability_bag.py`

## 11. 현재 한 줄 결론

정상 구간 채터링의 확정 원인은 무잡음이지만 약 15~19 cm로 양자화된 GNSS를 Global
EKF가 강하게 따라가고 그 correction을 controller가 직접 소비하는 구조다. 로컬에서
관측한 21.9% `velocity.x` 과적분은 production velocity bias가 아니라 한-PC 과부하와
receipt-time이 결합된 시간축 artifact다. 경량 GT-object test bed에서는 production
Q/R이 안정적으로 동작했으며, 다음 gate는 최신 controller를 포함한 LAN-to-LAN
closed-loop와 GPS 음영 복귀 검증이다.
