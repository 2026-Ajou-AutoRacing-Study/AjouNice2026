# Localization 현재 진입점

정리 기준: 2026-10-24 tracked launch/config/code. 이번 정리는 실행이나 새 주행 검증이 아니다.

읽는 순서:

1. [CURRENT: production 계약과 미완료 gate](COMPETITION_LOCALIZATION_CURRENT_STATE.md)
2. [평가 입력·heading provenance](BAG_CATALOG.md)
3. [현재 기본값을 지지하는 근거와 한계](CURRENT_EVIDENCE.md)
4. [UDP receipt-time 계약](morai_udp_receipt_time.md), [새 실험 형식](EXPERIMENT_TEMPLATE.md)

`docs/common_alias.txt`를 source한 터미널에서 provider 하나만 실행한다.

```bash
localization       # MR / competition: Competition heading/vx, heading -1.347 deg
localization_mrp   # MRP replay: Ego heading/vx, heading 0 deg
```

현재 체계는 GPS + IMU yaw-rate/ax + 선택한 status heading/vx의 Local/Global
`robot_localization` EKF다. Global-only soft `vy≈0` prior와 vehicle-pivot Smooth를
사용한다. Planning/TF는 `/localization/kinematic_state`, controller-facing 출력은
`/localization/control_kinematic_state`다. Q/R·heading·시간 정책을 문서 정리로 바꾸지 않는다.

실제 진입 코드:

- [competition bringup](../../base_ws/src/bringup/launch/localization_gnss_imu_gt_ctra.launch)
- [provider launch](../../base_ws/src/localization/localization_manager/launch/gnss_imu_gt_ctra.launch)
- [현재 config 구분](../../configs/localization/README.md)
- [현재 평가 도구](../../scripts/evaluation/localization/README.md)

[legacy/](legacy/)는 완료된 실험·기각/보류 후보·옛 provider·원본 snapshot 보관소다.
현재 근거가 역사적 문서를 인용해도 그 문서의 옛 기본값·명령 전체를 복원하지 않는다.
명시적 재현 요청이나 CURRENT에서 연결한 특정 근거 확인 전에는 legacy를 읽지 않는다.
코드와 설명이 다르면 현재 tracked launch/config/code가 우선이다.
