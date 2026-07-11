# scripts
- setup/build/run/check에 사용할 스크립트들

## scripts list
- `setup_ws.sh`: ros용 workspace 전체 설정 스크립트
- `build_all.sh`: ros용 workspace 전체 빌드 스크립트
- `run_full_stack.sh`: 현재 sensing bringup 실행용 wrapper
  - (실제 대회에선 필요한 launch 조합에 맞춰 이 스크립트를 갱신하면 될 듯)
- `check_topics.sh`: 토픽 테스트용 스크립트
- `morai_status.sh`: MORAI/ROS1 런타임 상태 점검용 스크립트
  - `11311`, `9090`, LiDAR UDP 포트, 주요 ROS 노드, `/connected_clients`, 핵심 토픽 publisher/subscriber를 한 번에 확인
- `morai_reset.sh`: MORAI 연동 중 꼬인 ROS1 프로세스 정리용 스크립트
  - `bash scripts/morai_reset.sh stack`: roscore, rosbridge, sensing/map/rviz 관련 ROS 프로세스 정리
  - `bash scripts/morai_reset.sh bridge`: rosbridge/rosapi만 정리

## MORAI live bringup order
- 전체 ROS 스택이 꼬였거나 roscore/노드가 남아 있으면 먼저 `morai_reset`
- PC 쪽 rosbridge 서버를 `rb`로 실행
  - MORAI `ROS Connect`는 client이므로, `bringup/morai_bridge.launch` 또는 `rb`를 먼저 실행해야 함
  - bridge만 재시작하려면 `rb_restart`
- MORAI에서 `ROS Connect`를 다시 연결
- TF가 필요하면 `maptf_morai`
- 센서/RViz가 필요하면 `sensing`
- 상태 확인은 `morai_status`

## Time mode
- live MORAI 실행: `/use_sim_time=false`
- rosbag replay: `/use_sim_time=true` 후 `rosbag play --clock`

## Notes
- 현재(260511) 있는 `setup_ws.sh`, `build_all.sh`, `run_full_stack.sh,`, `check_topics.sh`은 **예시 파일들임.**
