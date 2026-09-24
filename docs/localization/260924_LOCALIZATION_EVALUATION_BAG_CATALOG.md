# Localization 평가 rosbag catalog

상태: **CURRENT**
감사일: 2026-09-24

## 선정 원칙

bag을 다음 네 등급으로 나눈다.

1. `REPLAY`: raw estimator 입력이 있어 같은 입력으로 localization을 다시 실행 가능.
2. `CLOSED_LOOP`: localization, planning, control 결과가 있어 폐루프 안정성 평가 가능.
3. `FIELD_RESULT_ONLY`: 현장 결과는 강하지만 raw GPS/IMU/Competition이 없어 재실행 불가.
4. `DIAGNOSTIC_ONLY`: 시간축·부하·route가 오염돼 production 채택 근거로 사용 불가.

파일명으로 MR/MRP와 heading offset을 추측하지 않는다. 아래 provenance를 우선한다.

## 핵심 평가 세트

| 등급 | bag | heading oracle | 용도 |
|---|---|---:|---|
| REPLAY-A | `rosbags/raw_sensor_w_cam/competition_full_run/260822_005713_competition_full_route_raw_hz_run01.bag` | `-1.347 deg` | `/Competition_topic`, GPS, IMU, Ego가 모두 있는 99초 동일입력 offline A/B |
| REPLAY-B1 | `rosbags/raw_sensor/gps_blank1.bag` | replay `0 deg` | 2026-07-29 구형 로컬 raw, 취득 alias 불명. 약 5.58초 zero-GNSS의 기계적 연속성·재획득 replay 전용 |
| REPLAY-B2 | `rosbags/raw_sensor/gps_blank2.bag` | replay `0 deg` | 2026-07-29 구형 로컬 raw, 취득 alias 불명. 약 10.95초 zero-GNSS의 기계적 연속성·재획득 replay 전용 |
| REPLAY+CLOSED-LOOP-A | `rosbags/eval/localization/local_mr_stability/260919_231906_q0005_r010_gt_object_lightweight_no_traffic.bag` | `-1.347 deg` | 최신 경량 폐루프. 마지막 27.75초 GPS 음영 안에서 정지하며 구형 final state가 27.31초 중단됨. 음영 복귀는 없음 |
| CLOSED-LOOP-B | `rosbags/eval/localization/local_mr_stability/260920_003403_q0005_r010_gt_object_lightweight_no_traffic_v2_test.bag` | `-1.347 deg` | production Q/R 짧은 반복 smoke |
| NEGATIVE-CONTROL | `rosbags/eval/localization/local_mr_stability/260919_233559_sigma0p05_qxy0p05_gt_object_lightweight_no_traffic.bag` | `-1.347 deg` | 기각 Q/R이 악화하는지 평가기 회귀 확인 |
| FIELD-FUSION | `rosbags/eval/localization/260915_field_inspection/planning_rviz_0915_sensor_local_3.bag` | `-1.347 deg` | LAN-to-LAN fusion 채터링 결과 비교, 재실행 불가 |
| FIELD-GT | `rosbags/eval/localization/260915_field_inspection/planning_rviz_0915_gt_local.bag` | `0 deg` | LAN-to-LAN GT controller 기준, 재실행 불가 |

`FIELD-GT`에 MR 평가기의 기본 `-1.347 deg`를 사용하면 정확히 `1.347 deg`의 가짜 yaw
오차가 생긴다. `--heading-offset-deg 0`에서 yaw 오차는 수치 정밀도 수준으로 돌아온다.

## 보조 bag

- `planning_rviz_0915_sensor_local_1.bag`, `_2.bag`
  - 현장 반복성 확인용. raw estimator 입력이 없어 Q/R 재실행에는 사용하지 않는다.
- `260916_150411_full_route_mr_baseline.bag`
- `260917_134456_full_route_mr_sigma0p10.bag`
  - 로컬 full-stack 비교 자료. MORAI real-time factor와 software revision이 달라
    production velocity 또는 Q/R의 단독 인과 근거로 사용하지 않는다.

## 각 실험에서 반드시 기록할 provenance

```text
bag path:
acquisition: MR | MRP | LAN_FIELD | OFFLINE_REPLAY
alias:
gt_source: competition | ego
estimator heading correction:
evaluator heading offset:
time policy: receipt_wall | sim_time | recorded_header
real-time factor:
localization commit:
controller commit:
changed variable:
fixed variables:
```

위 항목을 복구할 수 없는 bag은 절대 yaw와 횡·종 오차의 채택 근거에서 제외한다.
heading과 무관한 topic rate나 raw position jump는 제한적으로 사용할 수 있다.

## 다음 현장/폐루프 녹화 필수 토픽

최소 다음 입력과 출력을 함께 기록한다.

```text
/Competition_topic
/competition/raw_twist       # native UDP receiver의 Competition linear/angular velocity
/Ego_topic                 # 개발/검증 oracle, 대회 제공 여부와 별개
/sensor/gps
/sensor/imu
/localization/gt/orientation
/localization/gt/twist
/localization/imu
/localization/gnss/odom
/localization/odometry/local
/localization/odom
/localization/kinematic_state
/planning/trajectory
/purepursuit_autoware/debug/lateral_error
/control/command/steering
/ctrl_cmd
/tf
/tf_static
```

가능하면 remote timestamp와 receipt timestamp의 차이를 담는 진단 topic도 추가한다.
카메라/LiDAR는 localization-only 실험에는 기록하지 않는다.
