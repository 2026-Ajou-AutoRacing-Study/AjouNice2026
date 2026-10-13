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
   - MR 전체 경로 localization·control 안정성 평가:
     `bash scripts/rosbag/record/rosbag_for_localization_stability_eval.sh full_route`
4. FAST-LIO offline 입력 보정: [runbook](../docs/runbooks/legacy/fast_lio_rosbag_repair.md)을 따른다.

`launch_sensing.sh`는 이전 `run_full_stack.sh`의 이름을 바로잡은 것이다. 현재는
`bringup sensing.launch`만 실행하며, bridge나 map TF까지 함께 기동하지 않는다.

## 시간 모드

- live MORAI: `/use_sim_time=false`
- rosbag replay: `/use_sim_time=true`, `rosbag play --clock`

## 후단과 노드 지연 분석

`scripts/evaluation/audit_fullstack_recording.py`는 validator의 bag·결과 JSON·events·
`ros_parameters.yaml`이 있는 한 run 폴더를 읽는다. 제어 시작부터 종료까지의 토픽 주기,
수신/header 차이, 충돌 메시지, 궤적 검사와 실제 planner 설정을 분리한다. 새 ROS master,
GPU 추론이나 MORAI를 실행하지 않는다. 분석 결과는 `planner_logs/analysis/<run_id>/`에 둔다.

```bash
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source planning_ws/install_isolated/setup.bash
python3 scripts/evaluation/audit_fullstack_recording.py --run "$RUN_DIR" --output "$ANALYSIS_DIR/audit_v1"
```

`profile_recorded_map_prediction.py`는 같은 입력에서 scalar/arc-index 계산의 ROS 메시지를
비교하고, 차량 전체의 예측·메시지 생성·직렬화 CPU 비용을 나눈다. 기본 실행은 고정된
lane-history 연구 후보도 검증하되 production을 변경하지 않는다. `--skip-candidate`는
계산 비교만 반복한다. 실행에는 인지 후단·평가 패키지의 Python 경로가 필요하다.

```bash
export PYTHONPATH="$PWD/base_ws/src/perception_object_pipeline/src:$PWD/base_ws/src/tracking_prediction_evaluation/src:$PWD/perception/detection3d/bevfusion/tools:$PYTHONPATH"
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/evaluation/profile_recorded_map_prediction.py \
  --run "$RUN_DIR" --output "$ANALYSIS_DIR/map_v1"
```

다음 실제 주행에서 GPU 입력 대기/전처리/추론과 planner 개별 비용을 더 보려면 기존 녹화와
별도로 아래 가벼운 trace를 쓸 수 있다. canonical 및 Base5 trial namespace를 모두 기록해
latency 토픽 remap 누락을 방지한다. 원본 센서는 추가하지 않는다. 발행되지 않는 debug
토픽을 생성하거나 노드를 재시작하지 않으므로 녹화 후 실제 topic count를 확인한다.

```bash
bash scripts/rosbag/record/rosbag_for_perception_latency_eval.sh --list  # 파일 생성·ROS 실행 없음
bash scripts/rosbag/record/rosbag_for_perception_latency_eval.sh --duration 60
```

수신 시각 차이는 callback CPU 비용이나 GPU 연산 시간과 같지 않다. tracker의 30 Hz 새
출력 header 역시 새 detector 관측이 아니다. C++ 포팅은 실제 비용과 출력 동등성을 확인한
노드부터 별도 A/B로 수행하며, 이 실행기들이 운영 정책을 자동 승격하지 않는다.

## 원시 LiDAR 패킷·카메라 녹화

51패킷 scan 경계와 first-packet stamp를 재검토할 때는 가벼운 latency trace만으로는
부족하다. `scripts/rosbag/record/record_lidar_packet_probe.py`는 raw VelodyneScan과
PointCloud2·4카메라 JPEG·Ego/Object GT·IMU를 짧게 함께 기록한다. 기존 900 RPM·51패킷
입력은 변경하지 않는다. `--list`는 ROS 없이 목록만 표시하고, `--check`는 live 수신만
확인한다. `--run`만 bag을 기록하며 ROS 노드 기동·파라미터 변경·차량 제어는 하지 않는다.

네트워크 연결 이후 자동 로드·운전·녹화까지 맡기는 경우에는
`run_lidar_packet_probe_auto.py --run`을 쓴다. 이 실행기는 기존 validator를 GT 모드로
재사용하며 별도 실험 설정을 생성한다. MORAI는 `ego_network_validator_local`에 연결하고
사용자 `rc`만 유지한다. 다른 캠페인과 동시에 실행하지 않는다. standalone recorder와
달리 이 자동 실행기는 시험 노드 기동·시나리오 로드·차량 제어를 수행한다.

시나리오 이름·로컬 UDP 설정·세 터미널 명령과 수동 운전 구간은
[benchmark 준비 절차](../configs/scenario/perception_benchmark/README.md#2026-10-14-원시-lidar-패킷-실험-준비)를 따른다.
완료한 bag을 같은 원본의 50/51/회전각 A/B에 쓰고, MORAI OFF 상태에서 후속 분석한다.
