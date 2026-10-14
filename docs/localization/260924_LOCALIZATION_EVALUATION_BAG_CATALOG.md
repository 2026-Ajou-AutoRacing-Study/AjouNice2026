# Localization 평가 rosbag catalog

상태: **CURRENT**
최초 감사: 2026-09-24; 전체 코스 후보 검증 입력 추가: 2026-10-15.

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

## 전체 코스 회귀 입력 (2026-10-15 추가)

- 최신 팀장님 E24 풀스택 원본:
  `rosbags/eval/perception/261014_teammate_fullstack/full_baseline_00_sample_20261014_012302_01/planbag_full_baseline_00_sample_P_20261014_012302.bag`
  - **REPLAY + CLOSED_LOOP**. bag 약 187.84초, 실제 주행 약 161.82초,
    최고속도 약 28.57m/s. 원본 events의 controller 시작/주행 종료로 평가 구간을 제한한다.
  - GPS 4,157, IMU 7,679, Competition 7,643, Ego 7,642 메시지가 있어 같은 원시 입력으로
    현재 estimator를 재실행할 수 있다. recorded static TF와 실제 파라미터도 함께 확인한다.
  - 실제 adapter `source=competition`, heading correction `-1.347°`,
    `use_sim_time=false`, sigma=0.10m/Qxy=0.0005이며 IMU ay mask는 OFF였다.
  - `/competition/raw_twist`와 내부 Global/Local odom은 원본에 없다. 해당 원본의 내부
    ay/vy를 관측했다고 쓰지 않는다. 재실행에서 내부 상태를 새로 녹화한다.
  - 원본 BEVFusion/판단/제어 결과와 후보 estimator replay를 구분한다. 원래 Ego 움직임을
    재사용한 replay는 후보의 실제 완주/조향/회피 개선을 증명하지 않는다.
- 과거 actor-free 전체 코스 두 bag은 별도 재현·회귀 보조 입력이다.
  `local_mr_stability/260925_194856_smooth_tau0p5_full_route_no_actors.bag`,
  `local_mr_stability/260925_200740_global_full_route_no_actors.bag`
  - 최신 E24 bag 또는 새 주행과 software/제어 속도 정책이 같다고 가정하지 않는다.

전체 코스 soft 횡속도 prior 검증과 독립 IMU ay ON 진단의 판정은
[`261015_GPS_SHADOW_CLOSED_LOOP_IMPROVEMENT.md`](261015_GPS_SHADOW_CLOSED_LOOP_IMPROVEMENT.md)에
누적하며 개별 replay/bag/그래프는 `planner_logs/analysis/261015_gps_shadow_fullcourse/`에 둔다.

같은 날 개선 Smooth의 동일입력 검증에는 soft 실행 `034133`(무객체 중단),
`012811`(무객체 완주), `004111`(박스 1개), `002129`(박스 2개)와 위 E24의
`261015_gps_shadow_fullcourse/lateral_ready/estimator.bag`를 사용했다. 첫 네 실행은
`planner_logs/validation/loc_*_261015_<시각>/`의 원본 case bag/snapshot/events가 기준이다.
이 입력은 모두 recorded soft Global/Local/Ego를 재사용한 **Smooth-only 비교**이며,
새 Smooth의 차량 폐루프나 GT pose 대조로 분류하지 않는다. 목록·정확한 경로·출력은
`planner_logs/analysis/261015_soft_production_smooth_improvement/candidate_results.json`에 있다.

## 새 production 실제 폐루프 입력 (2026-10-15)

`planner_logs/validation/` 아래 다음 실행의 `success/<case>/planbag_*.bag`다.
모두 **REPLAY + CLOSED_LOOP**, MR/Competition, estimator/oracle heading 각각−1.347°,
receipt-time, localization + od_gt + 실제 ROS1 planning/PP·PI이다. 원본 snapshot과
runtime publisher/param/단일 initialpose로 GT pose 대체가 아님을 확인했다.

| 실행 | 장면 | 실제 GPS invalid | 주행 결과 / 부하 주의 |
|---|---|---:|---|
| `loc_fullcourse_soft_production_261015_044421` | 무객체 전체 코스 | 7.270초 | PASS151.07초; RViz 궤적 변환 노드 조기 종료 |
| `loc_fullcourse_soft_production_261015_045214` | 같은 무객체 전체 코스 | 7.242초 | PASS150.95초; RViz/영상10fps split ON |
| `loc_agent_soft_lateral_pair_261015_045709` | 음영 박스2개 | 11.918초 | PASS31.81초; RViz OFF·영상ON |
| `loc_agent_soft_lateral_pair_261015_050237` | 같은 음영 박스2개 | 12.844초 | PASS32.65초; RViz/영상10fps split ON |

4회 모두 충돌0·state 약50Hz이며 raw GPS/IMU/Competition/Ego/Global/Local/Smooth/
prior/TF/경로/제어가 저장됐다. 실제 pair는 두 물체 모두 같은 쪽으로 회피했으므로
좌우 양방향 회피 검증으로 세지 않는다. 별도 주행 간 비교는 같은 입력의 인과 A/B가 아니다.
최신pair `050237`를 원본 초기pose와 일치시키고soft만 바꾼 격리1× replay는
estimator 원인 분리용이다. 기록 Ego/trajectory의 재사용을 후보의 실제 차량 재주행으로
부르지 않는다. 결과와 전체 경로는 누적EVIDENCE 및
`planner_logs/analysis/261015_soft_smooth_closed_loop_validation/`에서 확인한다.

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
/localization/constraints/lateral_velocity  # soft ON일 때 실제 count 확인
/localization/imu
/localization/gnss/odom
/localization/odometry/local
/localization/odom
/localization/kinematic_state
/localization/control_kinematic_state
/planning/trajectory
/purepursuit_autoware/debug/lateral_error
/control/command/steering
/ctrl_cmd
/tf
/tf_static
```

가능하면 remote timestamp와 receipt timestamp의 차이를 담는 진단 topic도 추가한다.
카메라/LiDAR는 localization-only 실험에는 기록하지 않는다.
