# Localization config 선택

현재 기준은 [CURRENT](../../docs/localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md)와
`bringup/localization_gnss_imu_gt_ctra.launch`의 실제 load 순서·launch 인자다.

Production 파일:

- `localization_manager.yaml`: frame/topic 및 초기화 공통 설정. compatibility provider
  section도 담고 있으므로 옛 section의 존재를 현재 provider 선택으로 해석하지 않는다.
- `gnss_imu_gt_noise_sensor6_ideal.yaml`: GNSS sigma0.10m, Global soft vy sigma0.20m/s.
- `robot_localization_gnss_imu_local.yaml`, `robot_localization_gnss_imu_global.yaml`:
  EKF base 설정, Global Qxy0.0005.
- `robot_localization_gnss_imu_gt_ctra_local.yaml`,
  `robot_localization_gnss_imu_gt_ctra_global.yaml`: production CTRA overlay.

`use_lateral_velocity_constraint`와 `lateral_velocity_stddev_mps` launch 인자가
noise profile과 Global overlay보다 우선하며 publisher/mask를 함께 확정한다.
구형 profile을 받았다고 prior 절반만 켜진 상태를 만들지 않는다.

Compatibility-only 파일은 공통 bringup에서 직접 참조하므로 경로를 유지했다:
`fast_lio_morai_sensor4_vlp16.yaml`, `fast_lio_msf_morai_sensor4_vlp16.yaml`,
`gnss_imu_noise_sensor6_ideal.yaml`. 이것들은 production 후보 추천이 아니다.

[legacy/](legacy/)에는 현재 launch/import에서 참조되지 않는 FAST-LIO 설치별 설정,
구형 Sensor4 noise, 기각 Q/R과 `.save` 원본을 보존한다. 실험 alias가 명시적으로 선택한
경우 외에는 production에 로드하지 않는다. 센서·네트워크 config는 이 정리에서 변경하지 않았다.
