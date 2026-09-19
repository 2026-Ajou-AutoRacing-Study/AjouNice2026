# Legacy localization 기록

이 디렉터리는 현재 대회 production 계약이 아니다.

- 과거 FAST-LIO 및 FAST-LIO+GNSS 실험.
- GT heading/velocity를 쓰기 전의 GNSS+IMU baseline.
- `/Ego_topic` 직접 GT localization과 구형 `maptf_*` 실행법.
- 예전 control pipeline 문서.
- 루트 `docs/`에 있던 번호형 1~7 초기 설치·MORAI·rosbag·FAST-LIO 문서.
- `260804_localization_state_input_architecture_plan.md`: 채택 전 구조 검토 기록.

번호형 문서는 다음과 같이 모두 이 디렉터리에 보존한다.

1. `1_setup_ubuntu20_ros1.md`
2. `2_setup_morai_local.md`
3. `3_debug_lidar_imu.md`
4. `4_rosbag.md`
5. `5_GT_bridge.md`
6. `6_setup_fast_lio.md`
7. `7_fast_lio_bridge.md`

현재 production은 상위 [`README.md`](../README.md)와
[`260916_competition_lateral_jitter_stabilization_plan.md`](../260916_competition_lateral_jitter_stabilization_plan.md)를 따른다.

이 기록은 재현 근거를 보존하기 위해 삭제하지 않았다. 여기의 alias와 launch 명령을
현재 full stack에 동시에 섞지 않는다.
