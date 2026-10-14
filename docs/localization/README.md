# Localization 문서 index

마지막 정리: 2026-10-15.

이 디렉터리의 문서는 날짜가 아니라 `CURRENT / EVIDENCE / PLAN / HISTORICAL`
상태로 판단한다. 코드와 문서가 다르면 현재 branch의 tracked launch/config/code를
우선한다.

## CURRENT

| 목적 | 문서 |
|---|---|
| Production 계약, 알려진 위험, 다음 승인 gate | [`COMPETITION_LOCALIZATION_CURRENT_STATE.md`](COMPETITION_LOCALIZATION_CURRENT_STATE.md) |
| 평가용 rosbag 등급과 heading provenance | [`260924_LOCALIZATION_EVALUATION_BAG_CATALOG.md`](260924_LOCALIZATION_EVALUATION_BAG_CATALOG.md) |
| 새 실험 기록 형식 | [`EXPERIMENT_TEMPLATE.md`](EXPERIMENT_TEMPLATE.md) |

현재 production estimator는 다음과 같다.

```text
/Competition_topic heading, velocity.x
+ /sensor/gps
+ /sensor/imu yaw-rate, longitudinal acceleration
+ soft body vy≈0 prior (sigma=0.20m/s, Global only)
-> local/global robot_localization EKF
-> /localization/kinematic_state (Planning/TF)
-> /localization/control_kinematic_state (Controller, vehicle-pivot Smooth long/lat/yaw tau=0.5/0.1/0.5 s)
```

- MR 및 실대회: `localization`, `/Competition_topic`, heading correction `-1.347 deg`.
- MRP: `localization_mrp`, `/Ego_topic`, heading correction `0 deg`.
  이는 현재 replay 설정이며 MRP 시나리오·지도 yaw 정합의 독립 검증은 아니다.
- GNSS horizontal sigma `0.10 m`, Global EKF Qxy `0.0005`.
- Global-only soft body `vy≈0` prior 기본 ON, σ `0.20m/s`.
- Smooth 횡 correction cap `0.3m/s`. 차량 움직임이 아니라 Global 위치 보정량만 제한한다.
- controller는 Smooth state `/localization/control_kinematic_state`를 사용한다.
  Planning과 TF는 Global `/localization/kinematic_state`를 유지한다.
- `predict_to_current_time=true`, `two_d_mode=true`, `publish_sensor_tf=false`.
- 대회에서 `/Ego_topic` 기반 provider는 fallback이 아니다.
- MR의 초기 yaw와 연속 yaw는 모두 보정된 `/Competition_topic.heading`을 쓴다.
  개발 기본값은 초기 **위치만** `/Ego_topic`에서 가져온다. 대회 공식 START
  위치는 `localization initial_position_source:=official_start`로 선택하며,
  MR 로컬 공식 START 대조는 통과했다(위 CURRENT 문서). 대회 LAN 시작 검증 전에는
  기본값으로 승격하지 않는다.
- localization A/B에는 `od_gt`를 사용해 detector/GPU 부하를 분리한다.
- 기존 폐루프 전체 코스 A/B에는 객체·spawn을 제거하고 GPS 음영 영역을 유지한
  `configs/scenario/localization_evaluation/localization_full_route_no_actors.json`을
  사용했다. 후속 수동 미션 관찰은 `full_course/workspace/`의
  `loc_fullcourse_static_shadow_hard_baseline.yaml`과
  `loc_fullcourse_static_shadow_hard_soft_lateral.yaml`로 동일 장애물/교통 장면을
  비교한다. 신호등은 OFF, Planviz/영상은 ON이다. 역사적 baseline YAML은
  `use_lateral_velocity_constraint=false`, `control_state_use_vehicle_pivot=false`로
  기존 방식을 명시하며, 새 기본값은 soft + 개선 Smooth다. 실제 주행 결과는 아래
  누적 EVIDENCE에서 구분한다.
- 새 기본값의 실주행용 개인 YAML은 같은 `full_course/workspace/`의
  `loc_fullcourse_soft_production.yaml`(무객체),
  `loc_fullcourse_static_shadow_hard_production.yaml`(기존 장애물 배치)이다.
  Planviz/영상 split/측위 bag/safety ON, 신호등 OFF, od_gt를 유지한다.
  무객체 새 production2회와 음영 pair2회는 실제 사용자 폐루프PASS를 확인했다.
  어려운 전체 미션배치 production은 별도이며 이4회 성공을 해당 미션 검증으로 확장하지 않는다.

## EVIDENCE

- [`261015_GPS_SHADOW_CLOSED_LOOP_IMPROVEMENT.md`](261015_GPS_SHADOW_CLOSED_LOOP_IMPROVEMENT.md)
  - 실제 Validator + od_gt 폐루프에서 Global 미관측 ay/vy 발산을 확인하고,
    Global-only soft 횡속도 제약을 동일입력 replay·장애물·일반 장면으로 비교한다.
    후속 od_gt 전체 코스 반복·고속 회귀와 IMU ay OFF의 모델 의미도 같은 문서에 누적한다.
    원본과 초기화/TF를 맞춘 동일 입력 비교, ay ON 기각 및 Smooth 잔여 손익을 구분한다.
    이후 사용자 승인으로 soft를 기본 ON으로 승격하고, 5개 동일입력 bag으로 검증한
    vehicle-pivot/종횡 분리 Smooth를 반영했다. 이전 후보 HOLD와 현재 승격을 구분한다.
    새 Smooth의 사용자 실제 MR 전체 코스2회·음영pair2회, 같은pair 입력soft ON/OFF
    인과 replay를 추가했다. 횡발산 억제와 평균/중앙값 개선, 남은 종오차·일부p95 악화를
    함께 기록한다. LAN·장시간 음영 정차·실제 반대 방향 회피·슬립 검증은 남아 있다.
- [`260924_LOCALIZATION_BASELINE_AUDIT.md`](260924_LOCALIZATION_BASELINE_AUDIT.md)
  - 현재 코드·bag·평가기의 1차 감사와 다음 개선 우선순위.
- [`260924_GNSS_OUTAGE_CONTINUITY_AND_CONTRACT_AUDIT.md`](260924_GNSS_OUTAGE_CONTINUITY_AND_CONTRACT_AUDIT.md)
  - GPS 음영 state 연속 발행과 frame·단위 계약 감사.
- [`260925_SMOOTH_CONTROL_STATE_OFFLINE_EVIDENCE.md`](260925_SMOOTH_CONTROL_STATE_OFFLINE_EVIDENCE.md)
  - Global 정확도와 Local 연속성을 분리한 controller용 state의 offline 사전 검증.
- [`260925_SMOOTH_CONTROL_STATE_OPEN_LOOP_EVIDENCE.md`](260925_SMOOTH_CONTROL_STATE_OPEN_LOOP_EVIDENCE.md)
  - 동일 trajectory에서 controller 입력만 바꾼 open-loop A/B.
- [`260925_SMOOTH_CONTROL_STATE_CLOSED_LOOP_EVIDENCE.md`](260925_SMOOTH_CONTROL_STATE_CLOSED_LOOP_EVIDENCE.md)
  - 전체 경로·GPS 음영 포함 폐루프 A/B와 Smooth state production 승격 근거.
- [`260925_QUANTIZED_GNSS_CONTROL_STATE_EVIDENCE.md`](260925_QUANTIZED_GNSS_CONTROL_STATE_EVIDENCE.md)
  - GNSS cell interval과 Global 종방향을 결합한 controller state 후보의 1× replay.
- [`260925_GNSS_QUANTIZATION_LATTICE_AUDIT.md`](260925_GNSS_QUANTIZATION_LATTICE_AUDIT.md)
  - 세 raw bag의 `1/600000°` 격자와 UTM cell 축 `-1.348°` 정합 감사.
- [`260925_MOTION_INPUT_ABLATION_EVIDENCE.md`](260925_MOTION_INPUT_ABLATION_EVIDENCE.md)
  - IMU yaw-rate/ax 제거 A/B와 production motion input 유지 근거.
- [`260925_FUSION_INPUT_CONTRACT_AUDIT.md`](260925_FUSION_INPUT_CONTRACT_AUDIT.md)
  - IMU yaw-rate의 부호·단위·시간 정합과 lever-arm 이중 보정 방지 감사.
- [`260925_COMPETITION_YAW_RATE_AND_YAW_SMOOTHING_EVIDENCE.md`](260925_COMPETITION_YAW_RATE_AND_YAW_SMOOTHING_EVIDENCE.md)
  - Competition yaw-rate 대체 A/B와 control yaw tau 변경 기각 근거.
- [`260925_EGO_ORACLE_TIMING_AUDIT.md`](260925_EGO_ORACLE_TIMING_AUDIT.md)
  - Ego pose의 프레임 시간 불균일과 종 frame-jump 해석 제한.
- [`260925_TF_COTIME_CONTRACT_EVIDENCE.md`](260925_TF_COTIME_CONTRACT_EVIDENCE.md)
  - Local/Global EKF 시각 불일치로 생긴 TF closure 오류와 수정 replay.
- [`260925_GNSS_OUTAGE_REACQUISITION_EVIDENCE.md`](260925_GNSS_OUTAGE_REACQUISITION_EVIDENCE.md)
  - 반복 가능한 8초 음영 replay와 큰 재획득 correction의 안전 reset.
- [`260925_CHECKPOINT_RELOCATION_GUARD_EVIDENCE.md`](260925_CHECKPOINT_RELOCATION_GUARD_EVIDENCE.md)
  - 기본 OFF/monitor-only 체크포인트 이동 감지 가드와 정상 bag 오탐 감사.
- [`260918_competition_localization_critical_issue_status.md`](260918_competition_localization_critical_issue_status.md)
  - 현장 채터링, GNSS 양자화, 시간축과 GPS 음영 문제의 누적 원인 분석.
- [`260920_lightweight_closed_loop_evidence.md`](260920_lightweight_closed_loop_evidence.md)
  - 경량 GT-object 폐루프의 production Q/R 및 기각 preset 결과.
- [`260919_quantized_gnss_qr_tuning_evidence.md`](260919_quantized_gnss_qr_tuning_evidence.md)
  - GNSS 양자화와 production Q/R 채택 근거.
- [`morai_udp_receipt_time.md`](morai_udp_receipt_time.md)
  - 현장 UDP receipt-time 정책과 변경 금지 근거.

## PLAN

- [`261014_GPS_SHADOW_VALIDATOR_AGENT_GUIDE.md`](261014_GPS_SHADOW_VALIDATOR_AGENT_GUIDE.md)
  - Validator 구현과 대조한 agent 전용 GPS 음영 실험 안내. 후보/config snapshot, 측위 전용 녹화/count audit, opt-in 준비·통신·실제 정지/P gate를 설명한다. 코드 지원과 실제 폐루프 결과·드리프트 개선·production 승격은 별개다.
- [`LOCALIZATION_PLANNING_CONTROL_EXPERIMENT_PLAN.md`](LOCALIZATION_PLANNING_CONTROL_EXPERIMENT_PLAN.md)
  - GNSS 입력 양자화 대응, odom-frame 경로·제어, 이후 판단·제어 공동 곡선 추종 A/B.
- [`260920_competition_status_and_local_control_frame_plan.md`](260920_competition_status_and_local_control_frame_plan.md)
  - Competition Status 확장과 controller용 smooth local-frame 구조.

PLAN의 구조나 파라미터는 production 기본값이 아니다. 기본 gate는 동일 bag offline
A/B, 경량 폐루프, GPS 음영, LAN-to-LAN이다. 사용자의 별도 즉시 승격 승인으로
예외 반영한 soft·개선 Smooth도 미완료 gate를 통과했다고 기록하지 않는다.

## 공식 실행 계약

`docs/common_alias.txt`를 source한 터미널에서 다음 진입점만 사용한다.

```bash
# MR 및 실대회
localization

# 과거 pre-soft+legacy Smooth를 정확히 대조할 때만
localization use_lateral_velocity_constraint:=false control_state_use_vehicle_pivot:=false

# 공식 START에서만: v85 공식 시작 위치 + 보정된 Competition 초기 yaw
localization initial_position_source:=official_start

# MRP
localization_mrp
```

`MR/MRP`는 실행 PC의 위치가 아니라 launcher와 topic 계약을 뜻한다. `-1.347 deg`는
UDP/ROS transport 보정이 아니라 MR heading convention 보정이다. 이미 보정된
`/localization/kinematic_state`에 다시 적용하지 않는다.

raw `/Ego_topic`을 oracle로 사용할 때만 실행 조건과 같은 offset을 oracle 쪽에
적용한다. `analyze_localization_stability_bag.py` 기본값은 MR용 `-1.347`이므로 MRP
bag은 반드시 다음 인자를 사용한다.

```bash
--heading-offset-deg 0
```

동시에 두 localization provider를 실행하지 않는다.

## HISTORICAL

[`legacy/`](legacy/)에는 Fast-LIO, 구형 GNSS+IMU baseline, `/Ego_topic` 직접 GT,
과거 설치·실행 절차와 완료된 계획만 둔다. 검색 결과에 먼저 나와도 checkpoint,
heading, Q/R, alias를 현재 기본값으로 재사용하지 않는다.
