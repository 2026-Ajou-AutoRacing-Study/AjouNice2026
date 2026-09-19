# Localization 문서 진입점

이 디렉터리에서 현재 대회 production 계약은 하나다.

```text
/Competition_topic heading, velocity.x
+ /sensor/gps
+ /sensor/imu
-> GNSS+IMU+GT CTRA-like local/global EKF
-> /localization/kinematic_state
```

## 현재 읽어야 하는 문서

1. [`260918_competition_localization_critical_issue_status.md`](260918_competition_localization_critical_issue_status.md)
   - 현재 중대 이슈, 확정 원인, `velocity.x bias` 정정, 로컬 시간축과 해결 순서.
2. [`260916_competition_lateral_jitter_stabilization_plan.md`](260916_competition_lateral_jitter_stabilization_plan.md)
   - 현장 횡방향 흔들림, 조향 채터링, covariance A/B와 채택 기준.
3. [`260801_robot_localization_gnss_imu_gt_ctra_fusion.md`](260801_robot_localization_gnss_imu_gt_ctra_fusion.md)
   - 현재 estimator의 센서 역할, frame, timestamp, covariance와 검증 근거.
4. [`morai_udp_receipt_time.md`](morai_udp_receipt_time.md)
   - 현장 UDP receipt-time 정책.

## 공식 실행 계약

`docs/common_alias.txt`를 source한 터미널에서 다음만 production 진입점으로 사용한다.

```bash
# MR 및 실대회: /Competition_topic, heading -1.347 deg correction
localization

# MRP: /Ego_topic, 나머지 estimator 계약은 동일
localization_mrp
```

여기서 `MR`은 2025 대회용 launcher, `MRP`는 2026 개인용 launcher다. 실행하는
컴퓨터가 로컬인지 여부가 아니라 launcher가 제공하는 topic 계약으로 alias를 고른다.

대회 client에서는 `/Ego_topic`을 기대하지 않는다. 따라서
`localization_morai_gt`는 대회 fallback이 아니다. 대회에서는 `localization` 하나를
정상화하고, 입력 상실 시 안전 정지하는 정책을 사용한다.

동시에 두 localization provider를 실행하지 않는다. duplicate timestamp 거부는 서로
다른 provider가 같은 TF와 topic을 중복 발행하는 것을 방지하지 못한다.

## Legacy

[`legacy/`](legacy/)는 과거 1~7 설치·MORAI·rosbag·FAST-LIO 문서, 구형 GNSS+IMU
baseline, `/Ego_topic` 직접 GT, 과거 localization 구조 계획과 control 실행 기록이다.
현재 대회 명령이나 기본값을 결정할 때 검색 결과만 보고 재사용하지 않는다. 과거 실험
재현이 필요한 경우에만 읽는다.
