# localization_manager

현재 provider는 GNSS + status heading/vx + IMU yaw-rate/ax의 Local/Global
`robot_localization` EKF와 Global-only soft vy prior, vehicle-pivot Smooth다.
실행은 `docs/common_alias.txt`의 `localization`(MR/Competition, heading−1.347°) 또는
`localization_mrp`(MRP/Ego, 0°) 중 하나만 선택한다.

현재 계약·근거·남은 gate는 [Localization router](../../../../docs/localization/README.md)와
[CURRENT](../../../../docs/localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md)를 따른다.
MRP의0°는 replay 설정이며 지도 yaw 독립 검증 완료를 뜻하지 않는다.

공통 출력은 map-frame base_link pose, base_link-frame twist를 담은
`/localization/kinematic_state`(`nav_msgs/Odometry`)다. Local EKF는 `odom -> base_link`,
최종 publisher는 Global stamp까지 외삽한 Local pose로 `map -> odom`을 소유한다.
GPS 음영에는 최초 유효 z 이후 마지막 z를 유지하고 planar state를 계속 발행한다.

bringup의 `localization_gnss_imu_gt_ctra.launch`는 controller-facing
`/localization/control_kinematic_state`도 발행한다. Smooth는 TF를 소유하지 않고
Planning/TF Global 출력을 바꾸지 않는다. 현재 longitudinal/lateral/yaw tau는
`0.5/0.1/0.5s`, vehicle pivot ON, 횡 **correction** cap `0.3m/s`다.
Local-only state를 map trajectory 소비자에 직접 remap하지 않는다.

`gnss_imu_gt_ctra.launch`와 현재 config는 보존한다. relocation guard는 OFF/monitor-only,
quantized control candidate는 OFF, GNSS motion compensation은0초다.
실험 node·test가 설치돼 있다는 사실은 production 승격 근거가 아니다.

Compatibility-only 경로도 유지한다: `fast_lio.launch`, `fast_lio_gnss.launch`,
`gnss_imu.launch`, `morai_gt.launch`와 이들의 Python 설치 항목. 공통 bringup·catkin·
테스트에서 참조하므로 이번 정리에서 이동하거나 설치 항목을 제거하지 않았다.
Fast-LIO/fast_lio_msf/livox submodule도 현행 build 의존성이며 production 선택은 아니다.
과거 실행법·후보 파라미터가 필요할 때만
[원본 package README](../../../../docs/localization/legacy/snapshots/261015_localization_manager_README.md)를
확인한다. 직접 Ego GT는 대회 fallback이 아니다.
