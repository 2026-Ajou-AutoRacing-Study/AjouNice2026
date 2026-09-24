# Localization baseline 1차 감사

상태: **EVIDENCE**

감사일: 2026-09-24

기준 commit: `0f6f2c1d`

## 결론

Localization은 전혀 개선되지 않은 상태가 아니다. GNSS 양자화 원인 규명과 production
Q/R 채택으로 정상 구간 pose jump는 크게 줄었고, 경량 폐루프도 50 Hz로 완주했다.
다만 지금까지의 개선은 주로 Q/R 튜닝이며, GPS 음영에서 output 연속성, 재획득,
controller용 smooth local state라는 구조 문제는 아직 남아 있다.

가장 먼저 해결할 항목은 GPS 음영 중 stale GNSS z 때문에 최종 state 전체가 중단되는
경로다. 그 다음에 reacquisition과 local-control frame을 분리해서 검증한다.

## 현재 구현 감사

### 잘 되어 있는 부분

- MR/competition과 MRP heading 계약이 alias로 분리돼 있다.
- raw motion adapter는 heading correction을 한 번만 적용한다.
- GPS `(0,0)`, invalid status, duplicate/out-of-order stamp를 거부한다.
- Local EKF는 GNSS 없이 heading/vx/yaw-rate/ax로 smooth odometry를 만든다.
- Global EKF는 quantized GNSS를 낮은 Qxy와 sigma 0.10 m로 완만하게 반영한다.
- TF owner가 분리돼 있고 provider 중복 실행 금지 계약이 있다.

### 구조적 위험

1. `gnss_imu_kinematic_state_publisher.py`는 GNSS z age가 `0.5 s`를 넘으면
   `/localization/kinematic_state` 전체를 발행하지 않는다.
2. GPS 음영 종료 첫 유효 GNSS를 일반 update로 즉시 반영하며 gradual realignment가 없다.
3. Controller는 Local EKF가 아닌 Global EKF pose를 직접 소비한다.
4. Competition production도 initial pose는 개발 편의상 `/Ego_topic`에 의존한다.
5. Competition `angular_velocity.z`와 IMU yaw-rate의 frame·부호·상관성 A/B가 없다.

## 2026-09-24 평가기 재검증

현재 `analyze_localization_stability_bag.py`로 기존 결과를 다시 계산했다.

| bag | 횡 오차 p50/p95 | 횡 jump p50/p95 | 종 오차 p50/p95 | 종 jump p50/p95 | steering-rate p50/p95 |
|---|---:|---:|---:|---:|---:|
| production Q/R 짧은 반복 | `1.50/5.42 cm` | `0.17/1.26 cm` | `1.44/5.36 cm` | `1.72/6.99 cm` | `0.000/0.601 rad/s` |
| 기각 sigma 0.05/Qxy 0.05 | `2.88/7.06 cm` | `0.38/5.15 cm` | `2.96/8.92 cm` | `2.75/9.63 cm` | `0.181/1.248 rad/s` |
| 현장 fusion run 3 | `2.66/8.01 cm` | `0.35/5.24 cm` | `4.43/13.42 cm` | `1.52/7.81 cm` | `0.165/1.589 rad/s` |
| 현장 GT | `0/0 cm` | `0/0 cm` | `0/0 cm` | `0/0 cm` | `0.010/0.340 rad/s` |

서로 route, controller, 부하가 다른 bag이므로 행 간 차이를 단일 Q/R 효과로 해석하지
않는다. 이 표의 강한 결론은 다음 두 가지다.

1. 현재 평가기가 기존 production/기각 preset의 상대 경향을 재현한다.
2. 현장 fusion과 GT 사이에는 controller-facing 안정성 차이가 실제로 존재한다.

현장 GT bag은 evaluator heading offset `0 deg`에서 yaw p95가 수치 정밀도 수준이다.
잘못된 기본 `-1.347 deg`를 적용하면 정확히 `1.347 deg`의 가짜 yaw 오차가 생겼다.

## GPS 음영 bag 감사

| bag | GPS sample | zero sample | 연속 zero-GNSS |
|---|---:|---:|---:|
| `gps_blank1.bag` | 596 | 162 | 약 `5.55 s` |
| `gps_blank2.bag` | 960 | 315 | 약 `10.93 s` |

두 bag 모두 `/Ego_topic`, GPS, IMU는 있지만 `/Competition_topic`은 없다. 따라서 MRP
계약인 `gt_source=ego`, heading offset `0 deg`로 state-continuity A/B를 수행한다.
대회 입력 완전 재현은 아니지만 stale-z output 중단을 검출하기에는 충분하다.

## 개선 순서

1. **P0-A: GNSS z hold A/B**
   - GPS 음영에서도 planar state를 계속 발행하고 마지막 유효 z만 유지.
   - `gps_blank1/2`로 output gap, pose jump, NaN을 검증.
2. **P0-B: reacquisition 상태기계**
   - 복귀 첫 update의 innovation과 correction step을 측정.
   - covariance inflation 또는 제한된 realignment를 각각 분리 A/B.
3. **P0-C: controller-local state prototype**
   - local pose와 map trajectory를 같은 control-local frame으로 변환.
   - 단순 topic remap은 하지 않음.
4. **P1: start/reset 계약**
   - 공식 start pose 및 checkpoint teleport interface 확정.
5. **P1: Competition yaw-rate 통제 실험**
   - IMU 대체와 residual 진단만 비교하고 중복 fuse하지 않음.

Production 기본값은 위 gate 전까지 바꾸지 않는다.
