# scripts

저장소 최상위의 `scripts/`는 실행 목적별로 정리한다. 모든 명령은 저장소
루트에서 실행하거나, 각 파일의 경로를 명시해서 실행한다.

## 디렉터리

| 경로 | 역할 | 대표 도구 |
| --- | --- | --- |
| `workspace/` | ROS workspace 초기화와 빌드 | `setup_ws.sh`, `build_all.sh` |
| `runtime/` | MORAI/ROS 실행, 상태 점검, 정리 | `launch_sensing.sh`, `morai_status.sh`, `morai_reset.sh` |
| `rosbag/record/` | live ROS 토픽 녹화 | `rosbag_raw.sh`, `rosbag_cam_raw.sh`, `rosbag_for_eval.sh` |
| `rosbag/repair/` | bag timestamp와 LiDAR point time 보정 | `fix_bag_header_stamp.py`, `convert_lidar_for_fastlio.py` |
| `rosbag/inspect/` | rosbag/PointCloud 입력 상태 확인 | `check_cloud_points.py` |
| `evaluation/fast_lio/` | FAST-LIO와 MORAI Ego GT 비교 | `eval_fast_lio_*.py` |
| `experiments/rosbag_legacy/` | 초기 rosbag 검증 실험 보존용 | 고정 경로 기반 legacy 스크립트 |

## 기본 실행 흐름

1. workspace 준비: `bash scripts/workspace/setup_ws.sh` 또는 `bash scripts/workspace/build_all.sh`
2. MORAI live 상태 확인/정리: `bash scripts/runtime/morai_status.sh`, `bash scripts/runtime/morai_reset.sh stack`
3. rosbag 녹화
   - raw 센서: `bash scripts/rosbag/record/rosbag_raw.sh`
   - raw 센서 + 6개 카메라: `bash scripts/rosbag/record/rosbag_cam_raw.sh`
4. FAST-LIO offline 입력 보정: [runbook](../docs/runbooks/fast_lio_rosbag_repair.md)을 따른다.

`launch_sensing.sh`는 이전 `run_full_stack.sh`의 이름을 바로잡은 것이다. 현재는
`bringup sensing.launch`만 실행하며, bridge나 map TF까지 함께 기동하지 않는다.

## 시간 모드

- live MORAI: `/use_sim_time=false`
- rosbag replay: `/use_sim_time=true`, `rosbag play --clock`
