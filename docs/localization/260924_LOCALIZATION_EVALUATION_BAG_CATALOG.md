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

## 제공 Spawn 툴 실측 입력 (2026-10-15)

이동은 사용자 제공 실행 파일의gRPC SetTransform이며 SceneLoad emulation과 구분한다.
MR/Competition·heading−1.347°·경량GPS/IMU·무객체·soft/Smooth production·시간보상0초·
guardOFF·신호등OFF·영상/RViz ON이다. 각 teleport의 실제Ego 변화에서 보간을 분할한다.
재배포 수정안 START+CP1–14는 툴0–14와 모두 일치한다. 아래 원본 파일·JSON·실행명에
남은 CP13/15는 구번호(현행12/14)이며 원본은 고치지 않는다.
아래 입력은 모두 운영측 CP14 음영 범위 조정 **이전 로컬 환경**의 관측이다.
2026-10-15 운영측은 좌표·spawn을 유지하고 음영 범위를 조정하겠다고 답변했다.
새 환경은 별도 실행으로 검증하고 환경 버전/시나리오 hash를 기록하며 기존 결과에 덮어쓰지 않는다.

| 실행/산출물 | 등급·용도 | 결과·제한 |
|---|---|---|
| `planner_logs/validation/loc_official_spawn_probe_261015_131944/error/loc_fullcourse_no_actors_20261015_131945_01/` | REPLAY + 정차 teleport 대조 | 0–13 자연복구,14(수정안CP14, 구CP15)GPSzero·136.77m 미복구로 안전중단/P확인. 완주PASS 아님 |
| `planner_logs/analysis/261015_official_spawn_probe/packet_probe_20261015_132719_550832/` | DIAGNOSTIC_ONLY,raw recv trace+bag | 원시GPSzero·native시계연속·기어D를관측. 정차0→14→0,loc/planning/주행controller없음. ptrace부하로latencybenchmark금지 |
| `planner_logs/validation/loc_official_spawn_remaining_route_261015_132923/success/` | REPLAY+CLOSED_LOOP,수정안CP12(구CP13)재출발 | 툴12 약211m이동→XY/yaw회복→freshroute→stockPP/PI→목적지PASS/P. 배우·공식remaining-checkpoint상태복구검증아님 |

같은prefix의`131535`는CP1통과점을목적지로썼다가footprint검사에거절돼스폰전종료,
`131659`는첫0번의D/3D접지transient를즉시정지검사가잡은negative실행이다.
원본을보존하고성공집단에포함하지않는다. 상세표·분석도구는
`planner_logs/analysis/261015_official_spawn_probe/`와누적EVIDENCE를따른다.

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

2026-10-15 현장1/2/3+직접GT의33토픽을 전수 확인하고 최종state↔Ego 상대epoch를 역산했다.
fusion header기준최적lag는모두4ms, nuisance회귀는5.66/5.79/5.40ms,직접GT는0ms였다.
rawGPS가있는5bag calibration과구형Q/R대조로방법을검증했으나물리GPS지연의직접측정은아니다.
20ms급지속지연은자료와맞지않고추가runtime보상0초유지가합리적이다. 현장2의
state header→receipt p95약28ms는별도의하류지연이며GPSraw지연으로세지않는다.
누적EVIDENCE마지막절과`planner_logs/analysis/261015_field_gps_epoch_inverse/`를따른다.
새역산결과로FIELD_RESULT_ONLY를REPLAY등급이나현재productionLAN검증완료로승격하지않는다.

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

## 최종 hardening 신규 실행 (2026-10-15)

아래는 `planner_logs/validation/<execution>/success|error/<case>/planbag_*.bag`이다.
모두 MR/Competition, heading−1.347°, receipt-time, od_gt/실제 planning·PP/PI,
영상10fps/Planviz split ON, localization_analysis/safety ON이다.
baseline/full zero는 원래 validator CLI, stop/SceneLoad/known-prior 개입은 별도 개발
helper이며 직진·정속 제어기로 대체하지 않았다. execution의 snapshot/events/receipt가 기준이다.

2026-10-15 ‘공식 체크포인트는 음영 밖’은 후속 확인상 사용자 지도 그림 판단이며
복귀 시 유효 GNSS 보장은 아니다. 실제 제공 Spawn 툴14/수정안CP14(구CP15)에서도 로컬 raw GPS
zero를 재현했다. 아래 SceneLoad 실험과 새 gRPC Spawn 실측을 구분한다.
[공식 좌표 안내 백업](../competition/261015_공식_체크포인트_안내_보관.md)을 따른다.
아래 CP13/15는 구번호(현행12/14)이며, GPSzero는 로컬 개발 장면의 관측이지 실대회 음영 판정이 아니다.
해당 로컬 zero 원인은 미확인이고, 원본 bag·수치는 보존한다.

| execution | 등급·조건 | 판정·주의 |
|---|---|---|
| `loc_final_baseline_261015_055818` | REPLAY+CLOSED_LOOP, 짧은 무객체 기본값 | PASS; 실제 음영7.310초 |
| `loc_final_checkpoint_off_261015_061706` | CHECKPOINT_EMULATION, CP13 guard OFF | PASS; 정차/P→약114m 순간이동→GNSS 복구→새 route/재개 |
| `loc_final_checkpoint_on_261015_061917` | 같은 CP13 guard active | PASS; 실제 guard reset0회, 가속 회복 증명 아님; 마지막 absolute-age 강화 전 코드 |
| `loc_final_gps020_261015_062131` | REPLAY+CLOSED_LOOP, 짧은 GNSS20ms 후보 | PASS; 실제 음영7.346초, 기본값 아님 |
| `loc_final_shadow_hold_261015_062342` | CLOSED_LOOP_STOP_INTERVENTION, 기본값 | PASS; 실제 음영50.246초, Park hold30.068초 후 같은 PP/PI 재개 |
| `loc_final_full_zero_261015_062655` | REPLAY+CLOSED_LOOP, 전체 코스 기본0ms | PASS; 전체 코스7.302초 음영 |
| `loc_final_full_gps020_261015_063115` | PARTIAL_CLOSED_LOOP, 전체 코스20ms | ERROR/MANUAL_ABORT; 고주로 MPT/far-goal path 실패, 안전 종료. 완주로 세지 않음 |
| `loc_final_checkpoint_shadow_261015_064054` | NEGATIVE_CONTROL, 로컬 CP15 장면·새 위치 정보 없음 | SAFETY_FAILURE; GPSzero로 약138.65m 위치 차이, 재출발 차단·P 확인. 공식 체크포인트 음영 판정 아님 |
| `loc_final_checkpoint_known_261015_064654` | KNOWN_POSITION_PRIOR_EMULATION, 로컬 CP15 장면 | PASS; 명시적 checkpoint prior1회. 공식 운영 이벤트 아님 |
| `loc_final_checkpoint_known_261015_065447` | 위 조건 최신 helper 안전 검사 반복 | PASS; fresh heading/P를 발행 직전에 다시 검증 |

known-prior는 runtime 위치 입력을 Ego에서 가져오지 않는다. 실험자가 요청한
ScenarioLoad 목적지라는 명시적 prior를 사용하며, 독립 Ego는 안전/오차 확인용이다.
`064654`까지 target metadata의 `position_runtime_source` 문자열은 GNSS-only로 남지만
실제 `known_checkpoint_prior_receipt.json`과 bag의 추가initialpose가 우선 provenance다.
새 helper는 metadata도 두 모드로 구분한다. 원본 snapshot을 고쳐 쓰지 않는다.

동일 raw replay의 latest `044421` 대9/19 historical heldout과 synthetic pause는
`planner_logs/analysis/261015_localization_final_hardening/`에 있다. 임의 합성 hold의
누적 변위를 GT accuracy/새 폐루프라고 부르지 않는다. detailed 결과는 누적 EVIDENCE를 따른다.

## 실제 무카메라·무LiDAR 전체 코스 추가 (2026-10-15)

- 실행: `loc_fullcourse_true_lightweight_261015_112304`.
- bag: `planner_logs/validation/loc_fullcourse_true_lightweight_261015_112304/success/loc_fullcourse_no_actors_20261015_112306_01/planbag_loc_fullcourse_no_actors_P_20261015_112306.bag`.
- **REPLAY + CLOSED_LOOP**, MR/Competition, estimator/oracle 각각−1.347°, receipt-time,
  softσ0.20·vehicle-pivot Smooth0.5/0.1/0.5·cap0.3, motion projection0초.
- 사용자가 MR에 과거9/19와 동일한 `Sensor_localization_gt_lightweight.json`을 실제 Load했다.
  camera/LiDAR0개, GPS·IMU는 기존 full JSON과 동일. 새 센서 snapshot·actual launch 명령·
  live graph에서 센서 발행 부재/단일 제어를 확인했다. `include_sensors=false`만의 주장이 아니다.
- 같은 무객체 전체 코스·od_gt·실제 planning/PP·PI·RViz/영상10fps split ON,
  PASS153.71초·충돌0·실제 정지/P VERIFIED. GPS 음영7.276초 포함, replay/assessment 녹화 audit 모두 true.
- 원시 GPS lag0ms와 작은 종오차가9/19 경량 특성을 재현했다. 최신 normal GNSS 이동 비교와
  새 전체 이동/음영 지표는 누적 EVIDENCE 마지막 절 및
  `planner_logs/analysis/261015_true_lightweight_comparison/run112304/`를 따른다.
  기존 recorded 출력과 modern same-input 후보 replay의 모집단을 혼용하지 않는다.

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
