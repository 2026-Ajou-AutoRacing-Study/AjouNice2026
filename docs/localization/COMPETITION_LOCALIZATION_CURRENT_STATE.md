# Competition Localization 현재 상태

상태: **CURRENT**
마지막 갱신: 2026-09-24

## 한 줄 결론

현재 production은 GNSS 양자화에 맞춘 Q/R로 정상 GNSS 구간의 pose jump를 줄였고,
GPS 음영 중에도 마지막 유효 z만 유지하며 planar state를 계속 발행한다. 남은 핵심
구조 과제는 GPS 재획득 correction과 controller가 Global EKF state를 직접 소비하는
점이다.

## Production 계약

| 항목 | MR·실대회 | MRP |
|---|---|---|
| alias | `localization` | `localization_mrp` |
| motion source | `/Competition_topic` | `/Ego_topic` |
| heading correction | `-1.347 deg` | `0 deg` |
| initial pose | 공식 시작 position 제공 전 첫 `/Ego_topic` pose 임시 사용 | 동일 |
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
- `/Ego_topic`은 현재 초기 pose 한 번에만 쓰는 개발용 임시 입력이다. 공식 시작
  position interface를 받으면 교체한다.

## 현재 데이터 흐름

```text
/Competition_topic heading/vx ----> GT motion adapter ----+
/sensor/imu yaw-rate/ax ----------> covariance adapter ---+--> Local EKF
                                                          +--> Global EKF
/sensor/gps quantized x/y --------> GNSS odometry --------+

Local EKF  : odom -> base_link, GNSS가 직접 들어가지 않는 smooth motion
Global EKF : map pose, GNSS x/y correction 포함
Publisher  : Global pose + 최신/마지막 유효 GNSS z -> /localization/kinematic_state
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
6. GPS 음영 `5.64 s`, `11.01 s` replay에서도 마지막 유효 z를 유지하면서
   `/localization/kinematic_state`를 각각 `50.00 Hz`, `50.01 Hz`로 연속 발행했다.
7. 현재 `/Ego_topic` 임시 초기값 계약으로 수행한 Competition 전체 경로 replay에서
   `49.71 Hz` state와 기존 정확도 수준을 유지했다.

## 남은 위험과 우선순위

### 해결. GPS 음영 중 최종 state 연속성

`morai_gps_to_local_odometry.py`는 `(0,0)` GNSS를 거부한다. 최초 유효 z를 받은 뒤
GNSS가 stale해지면 publisher는 마지막 유효 z만 유지하고 Global EKF의 x/y/yaw/twist를
계속 발행한다. 유효 GNSS가 돌아오면 z hold를 해제한다. 최초 z조차 없는 startup은
기존처럼 준비되지 않은 상태로 판단한다.

### P0-A. GPS 재획득과 checkpoint 순간이동 구분

현재는 음영 종료 첫 GNSS를 일반 update로 즉시 반영한다. 다음 기능은 없다.

- reacquisition 상태기계.
- 복귀 직후 covariance inflation.
- correction step/rate limit.
- 정상 복귀와 checkpoint teleport 구분.

현재 low-Q replay에서는 재획득 직후 폭발적 correction이 관찰되지 않아 임의의
covariance inflation은 넣지 않았다. GPS 음영 미션 폐루프와 checkpoint teleport를
분리해 추가 검증한다.

### P0-B. Controller가 Global correction을 직접 소비

Local EKF는 이미 smooth state를 만들지만 controller는 Global EKF pose를 받는다.
장기 구조 후보는 controller 전용 local state와 같은 frame으로 변환한 trajectory다.
`/localization/odometry/local`만 controller에 단순 remap하면 map trajectory와 frame이
달라지므로 금지한다.

### P1. 공식 start pose 교체와 checkpoint reset

Competition Status에는 global position이 없다. 현재 첫 `/Ego_topic` pose는 주최 측의
공식 시작 position interface가 아직 확정되지 않아 초기값 용도로만 쓰는 의도적인 임시
계약이며 문제나 fallback이 아니다. 공식 interface가 제공되면 이 입력만 교체한다.
사고 후 checkpoint 순간이동을 자동 검출해 두 EKF를 reset하는 계약도 아직 없다.

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
