# Competition Localization 현재 상태

상태: **CURRENT**
마지막 갱신: 2026-09-24

## 한 줄 결론

현재 production은 GNSS 양자화에 맞춘 Q/R로 정상 GNSS 구간의 pose jump를 크게
줄였지만, controller가 Global EKF state를 직접 소비하고 GPS 음영에서 GNSS z가
stale해지면 최종 state가 중단될 수 있는 구조적 위험이 남아 있다.

## Production 계약

| 항목 | MR·실대회 | MRP |
|---|---|---|
| alias | `localization` | `localization_mrp` |
| motion source | `/Competition_topic` | `/Ego_topic` |
| heading correction | `-1.347 deg` | `0 deg` |
| GNSS horizontal sigma | `0.10 m` | `0.10 m` |
| Global EKF Qxy | `0.0005` | `0.0005` |
| controller-facing state | `/localization/kinematic_state` | 동일 |

공통 계약:

- local/global EKF `two_d_mode=true`.
- `predict_to_current_time=true`.
- `publish_sensor_tf=false`.
- UDP는 ROS host receipt-time 정책을 유지한다.
- localization provider는 한 번에 하나만 실행한다.
- 대회에서 `/Ego_topic`과 `/Object_topic`을 기대하지 않는다.
- 개발 중 initial pose는 `/Ego_topic`을 사용하지만, 대회 전 공식 start-pose
  interface로 교체해야 한다.

## 현재 데이터 흐름

```text
/Competition_topic heading/vx ----> GT motion adapter ----+
/sensor/imu yaw-rate/ax ----------> covariance adapter ---+--> Local EKF
                                                          +--> Global EKF
/sensor/gps quantized x/y --------> GNSS odometry --------+

Local EKF  : odom -> base_link, GNSS가 직접 들어가지 않는 smooth motion
Global EKF : map pose, GNSS x/y correction 포함
Publisher  : Global pose + 최신 GNSS z -> /localization/kinematic_state
Controller : /localization/kinematic_state 직접 소비
```

## 해결된 범위

1. MORAI GNSS는 무잡음이어도 동서 약 `14.8 cm`, 남북 약 `18.5 cm`로 양자화됨을
   확인했다.
2. 기존 sigma `0.05 m`, Qxy `0.05`가 cell transition을 과도하게 따라가는 주원인임을
   확인했다.
3. sigma `0.10 m`, Qxy `0.0005`는 동일 raw bag replay에서 횡 frame-jump p95를
   `5.432 -> 1.444 cm`, 종 frame-jump p95를 `7.234 -> 2.416 cm`로 줄였다.
4. 경량 폐루프에서 production Q/R은 localization `50 Hz`, 횡 frame-jump p95
   `1.31 cm`, steering-rate p95 `0.54 rad/s` 수준을 재현했다.
5. 로컬 full-stack의 velocity 적분 불일치는 production `velocity.x` bias가 아니라
   MORAI 비실시간율과 receipt wall-time 혼합 artifact로 판정했다.

## 남은 위험과 우선순위

### P0-A. GPS 음영 중 최종 state 연속성

`morai_gps_to_local_odometry.py`는 `(0,0)` GNSS를 정상적으로 거부한다. 그러나
`gnss_imu_kinematic_state_publisher.py`는 최신 GNSS z age가 `0.5 s`를 넘으면 Global
EKF odometry 전체를 거부한다. 보유 GPS 음영 bag은 zero-GNSS가 각각 약 `5.55 s`,
`10.93 s` 지속되므로 현 구조에서는 controller-facing state가 끊길 가능성이 높다.

첫 개선은 x/y/yaw/twist를 계속 발행하면서 z만 마지막 유효값으로 hold하는 제한된
A/B다. 음영 종료 후 reacquisition 정책과 섞지 않는다.

### P0-B. GPS 재획득과 checkpoint 순간이동 구분

현재는 음영 종료 첫 GNSS를 일반 update로 즉시 반영한다. 다음 기능은 없다.

- reacquisition 상태기계.
- 복귀 직후 covariance inflation.
- correction step/rate limit.
- 정상 복귀와 checkpoint teleport 구분.

P0-A로 출력 연속성을 먼저 보장한 뒤 별도 실험한다.

### P0-C. Controller가 Global correction을 직접 소비

Local EKF는 이미 smooth state를 만들지만 controller는 Global EKF pose를 받는다.
장기 구조 후보는 controller 전용 local state와 같은 frame으로 변환한 trajectory다.
`/localization/odometry/local`만 controller에 단순 remap하면 map trajectory와 frame이
달라지므로 금지한다.

### P1. 공식 start pose

Competition Status에는 global position이 없다. 현재 initial pose publisher가
`/Ego_topic`을 사용하는 개발 편의 계약은 실대회에서 성립하지 않는다. 대회가 제공하는
start pose interface와 reset/checkpoint 계약을 확정해야 한다.

### P1. Competition yaw-rate A/B

현재는 Competition heading/vx와 IMU yaw-rate/ax를 사용한다. Competition
`angular_velocity.z`는 IMU와 상관된 신호일 수 있으므로 동시에 독립 측정처럼 fuse하지
않는다. IMU yaw-rate 대체 또는 residual 진단으로만 통제 실험한다.

## Production 변경 gate

```text
동일 raw bag offline A/B
-> od_gt 경량 폐루프
-> GPS 음영 진입·유지·복귀
-> LAN-to-LAN 동일 controller
-> production alias/config 변경
```

Offline 위치 오차 하나만 좋아졌다는 이유로 기본값을 바꾸지 않는다. 횡·종 오차,
frame jump, yaw, state rate, steering-rate와 sign flip을 함께 본다.

## 변경하지 않을 항목

- heading correction `-1.347 deg`.
- production Q/R을 추가 sweep의 출발점 없이 임의 수정하는 것.
- UDP receipt-time 정책.
- `velocity.x` 임의 scale.
- local state만 controller에 바로 remap하는 것.
- perception P8/P9 결과를 localization 보정 근거로 사용하는 것.
