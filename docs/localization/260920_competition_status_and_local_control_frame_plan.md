# Competition Status 확장과 Local-control frame 검토

상태: **PLAN**

작성일: 2026-09-20

## 결론

업데이트된 대회 규정은 Competition Vehicle Status가 차량 자세, 종방향 속도,
각속도와 제어 입력을 제공한다고 명시한다. 이는 GNSS 없이도 짧은 시간 동안 연속적인
local odometry를 유지할 근거를 강화한다. 하지만 현재 production estimator도 이미
`/Competition_topic`의 `heading`과 `velocity.x`를 사용하므로 즉시 EKF 구조나 기본
Q/R을 바꾸지는 않는다.

가장 가치 있는 후속 구조는 다음과 같다.

```text
Global planning/perception:
  quantized GNSS + motion state -> 정확한 map pose

Fast controller feedback:
  Competition yaw/vx + yaw-rate/IMU -> 연속적인 odom pose

Global correction:
  local pose를 직접 계단 이동시키지 않고 local↔global 정합을 제한된 속도로 보정
```

단, controller pose만 `odom`으로 바꾸면 안 된다. 현재 planning trajectory는 `map`
좌표이므로 controller가 소비하는 pose와 trajectory를 같은 frame으로 맞추는 계약이
반드시 함께 필요하다.

## 규정상 사용 가능한 정보

Competition Vehicle Status에서 localization에 유용한 항목은 다음이다.

- 차량 자세 정보.
- `vel_x`.
- 각속도, 특히 planar localization의 `ang_vel_z`.
- 제어 입력은 진단이나 motion-model 보조에 사용할 수 있으나 직접 pose 측정값은 아니다.

다음 값은 제공되지 않는다. 메시지 schema에 필드가 존재하거나 0으로 보이더라도
유효 측정으로 fuse하지 않는다.

- `pos_x`, `pos_y`, `pos_z`.
- `vel_y`, `vel_z`.
- `accel_x`, `accel_y`, `accel_z`.
- tire force, slip angle, cornering stiffness 계열.

따라서 Competition Status만으로 global x/y를 정할 수 없고 GNSS 또는 다른 global
anchor는 여전히 필요하다. GNSS 양자화 문제도 사라지지 않는다.

## 현재 코드와의 차이

현재 `morai_gt_motion_adapter.py`는 Competition Status에서 다음 두 값만 변환한다.

| 입력 | 현재 사용 | 용도 |
|---|---|---|
| `heading` | 사용 | absolute planar yaw |
| `velocity.x` | 사용 | signed longitudinal velocity |
| `angular_velocity.z` | 미사용 | 현재는 `/sensor/imu` yaw-rate 사용 |
| roll/pitch | 미사용 | production `two_d_mode=true` |
| 제어 입력 | 미사용 | estimator measurement가 아님 |

Local EKF는 GNSS 없이 Competition yaw/vx와 IMU yaw-rate/longitudinal acceleration으로
`odom -> base_link`를 만든다. Global EKF는 같은 motion 입력에 quantized GNSS x/y를
추가한다. 그러나 controller가 받는 `/localization/kinematic_state`는 Global EKF의
`map` pose이므로 GNSS correction이 여전히 제어 feedback에 직접 보인다.

## 전략에 미치는 영향

### 즉시 유지

- production alias와 heading correction `-1.347 deg`.
- GNSS horizontal sigma `0.10 m`, Global Qxy `0.0005`.
- local/global EKF `two_d_mode=true`.
- 현장에서 검증한 UDP receipt-time 정책.
- Competition `heading + velocity.x`, IMU yaw-rate/ax 조합.

### 새로 가능해진 통제 실험

Competition `angular_velocity.z`를 바로 추가 fuse하지 않는다. 같은 simulator state에서
파생된 heading, yaw-rate와 IMU가 강하게 상관될 수 있어 독립 측정처럼 중복 fuse하면
covariance가 과도하게 작아질 수 있다. 다음을 동일 bag에서 비교한다.

1. 현재 production: Competition heading/vx + IMU yaw-rate/ax.
2. Competition motion: Competition heading/vx/yaw-rate + IMU ax.
3. 진단용 residual: Competition yaw-rate와 IMU yaw-rate를 fuse하지 않고 차이만 기록.

규정 문구만으로 orientation·angular velocity의 단위, 부호, body/world frame과 timestamp
품질을 확정하지 않는다. 현장 또는 공식 샘플에서 기존 corrected heading과 yaw가
일치하는지, `ang_vel_z`가 IMU z와 일치하는지를 먼저 확인한다.

## Local-control frame 구현 원칙

현재 dual-EKF의 smooth local branch를 실제 제어에 활용하는 것은 GNSS cell transition이
조향 채터링으로 전달되는 경로를 끊는 구조적 개선이다. 구현 시 다음 계약을 지킨다.

1. Planner, Lanelet, stop line, perception은 정확한 global `map` pose를 유지한다.
2. Controller에는 연속적인 local pose를 별도 topic으로 제공한다.
3. `map` trajectory를 controller가 사용할 동일 local frame으로 변환하거나, 이에
   동등한 controller 전용 상대 좌표 계약을 둔다.
4. global correction은 rate/step 제한을 두고 적용한다.
5. GPS 음영 복귀는 gradual realignment 대상으로 다룬다.
6. checkpoint 순간이동은 느리게 따라가지 않고 명시적으로 local/global state를 reset한다.

Canonical `map -> odom`을 무조건 low-pass하면 map 기반 perception과 stop-line 정합도
지연될 수 있다. 그러므로 첫 구현은 전체 TF를 흐리기보다 controller 전용 local-state와
trajectory 변환으로 범위를 제한한다.

## 승인 기준

현재 global-control을 baseline으로 다음을 동일 route와 동일 controller에서 비교한다.

- lateral/longitudinal tracking error p50/p95.
- steering-rate p50/p95와 steering sign flip rate.
- GNSS cell transition 직후 controller-facing pose/path step.
- 정지선과 Lanelet global 정합.
- GPS 음영 진입·복귀 시간과 최대 correction.
- checkpoint 재배치 후 정상 route 복구 여부.

이 A/B 전에는 production controller topic을 local pose로 단순 remap하지 않는다.
