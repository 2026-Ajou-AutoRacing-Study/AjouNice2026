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
   - 현재 중대 이슈, 확정 원인, production 계약과 남은 현장 검증.
2. [`260920_lightweight_closed_loop_evidence.md`](260920_lightweight_closed_loop_evidence.md)
   - 경량 GT-object 폐루프에서 production Q/R과 실험 preset을 비교한 최신 근거.
3. [`260919_quantized_gnss_qr_tuning_evidence.md`](260919_quantized_gnss_qr_tuning_evidence.md)
   - GNSS 양자화 원인과 production Q/R 채택 근거.
4. [`morai_udp_receipt_time.md`](morai_udp_receipt_time.md)
   - 현장 UDP receipt-time 정책.
5. [`260920_competition_status_and_local_control_frame_plan.md`](260920_competition_status_and_local_control_frame_plan.md)
   - Competition Status 확장 영향과 controller용 local-frame 구조 검토.

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

현재 production estimator tuning은 GNSS horizontal sigma `0.10 m`, Global EKF
x/y process-noise diagonal `0.0005`다. `predict_to_current_time=true`를 유지한다.
개발 단계의 최초 start pose는 `/Ego_topic`을 사용하며, 대회 전 공식 start-pose
interface로 교체한다.

폐루프 A/B에서만 다음 명시적 preset을 사용할 수 있다. 둘 다 `/Competition_topic`,
heading correction `-1.347°` 등 production 입출력 계약은 유지하지만 Global Qxy를
`0.05`로 높인다. 기본 `localization` alias와 production YAML은 변경하지 않는다.

```bash
# GNSS sigma 0.10 m, Global Qxy 0.05
localization_sigma_0p10_qxy_0p05

# GNSS sigma 0.05 m, Global Qxy 0.05
localization_sigma_0p05_qxy_0p05
```

두 preset은 동시에 실행하지 않으며, 매 주행마다 사용한 alias를 rosbag run tag에
기록한다. 현장 결과가 좋았다는 정성 관측만으로 production 기본값을 변경하지 않는다.

## Legacy

[`legacy/`](legacy/)는 과거 1~7 설치·MORAI·rosbag·FAST-LIO 문서, 구형 GNSS+IMU
baseline, `/Ego_topic` 직접 GT, 8월 estimator 구현 기록, 완료된 9월 16일 안정화 계획과
과거 control 실행 기록이다.
현재 대회 명령이나 기본값을 결정할 때 검색 결과만 보고 재사용하지 않는다. 과거 실험
재현이 필요한 경우에만 읽는다.
