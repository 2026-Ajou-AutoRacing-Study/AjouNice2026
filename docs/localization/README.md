# Localization 문서 index

마지막 정리: 2026-09-25.

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
-> local/global robot_localization EKF
-> /localization/kinematic_state
```

- MR 및 실대회: `localization`, `/Competition_topic`, heading correction `-1.347 deg`.
- MRP: `localization_mrp`, `/Ego_topic`, heading correction `0 deg`.
- GNSS horizontal sigma `0.10 m`, Global EKF Qxy `0.0005`.
- `predict_to_current_time=true`, `two_d_mode=true`, `publish_sensor_tf=false`.
- 대회에서 `/Ego_topic` 기반 provider는 fallback이 아니다.
- 공식 시작 position interface가 확정되기 전까지 초기 pose 한 번만
  `/Ego_topic`을 임시 사용한다. 연속 localization source나 대회 fallback이라는 뜻은
  아니다.
- localization A/B에는 `od_gt`를 사용해 detector/GPU 부하를 분리한다.

## EVIDENCE

- [`260924_LOCALIZATION_BASELINE_AUDIT.md`](260924_LOCALIZATION_BASELINE_AUDIT.md)
  - 현재 코드·bag·평가기의 1차 감사와 다음 개선 우선순위.
- [`260924_GNSS_OUTAGE_CONTINUITY_AND_CONTRACT_AUDIT.md`](260924_GNSS_OUTAGE_CONTINUITY_AND_CONTRACT_AUDIT.md)
  - GPS 음영 state 연속 발행과 frame·단위 계약 감사.
- [`260925_SMOOTH_CONTROL_STATE_OFFLINE_EVIDENCE.md`](260925_SMOOTH_CONTROL_STATE_OFFLINE_EVIDENCE.md)
  - Global 정확도와 Local 연속성을 분리한 controller용 state의 offline 사전 검증.
- [`260918_competition_localization_critical_issue_status.md`](260918_competition_localization_critical_issue_status.md)
  - 현장 채터링, GNSS 양자화, 시간축과 GPS 음영 문제의 누적 원인 분석.
- [`260920_lightweight_closed_loop_evidence.md`](260920_lightweight_closed_loop_evidence.md)
  - 경량 GT-object 폐루프의 production Q/R 및 기각 preset 결과.
- [`260919_quantized_gnss_qr_tuning_evidence.md`](260919_quantized_gnss_qr_tuning_evidence.md)
  - GNSS 양자화와 production Q/R 채택 근거.
- [`morai_udp_receipt_time.md`](morai_udp_receipt_time.md)
  - 현장 UDP receipt-time 정책과 변경 금지 근거.

## PLAN

- [`260920_competition_status_and_local_control_frame_plan.md`](260920_competition_status_and_local_control_frame_plan.md)
  - Competition Status 확장과 controller용 smooth local-frame 구조.

PLAN의 구조나 파라미터는 production 기본값이 아니다. 동일 bag offline A/B, 경량
폐루프, GPS 음영, LAN-to-LAN 순서로 검증한 뒤에만 CURRENT로 승격한다.

## 공식 실행 계약

`docs/common_alias.txt`를 source한 터미널에서 다음 진입점만 사용한다.

```bash
# MR 및 실대회
localization

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
