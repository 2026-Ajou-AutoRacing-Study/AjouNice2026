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

### 팀 공유용 개발 녹화 (raw 센서 포함)

기존 풀스택을 켜고 별도 터미널에서 아래 명령을 실행한다. 이 도구는 노드를 기동하거나
차량·스택을 정지하지 않는다. `recorder.log`에 `Recording`이 나온 뒤 주행하고 Ctrl-C로 마감한다.

```bash
source scripts/workspace/source_aw149.sh
bash scripts/rosbag/record/rosbag_for_development.sh
# 선택: 5분 자동 마감
bash scripts/rosbag/record/rosbag_for_development.sh --duration 300
```

`rosbags/eval/development/dev_날짜_고유번호/`에 bag·recorder.log·metadata.json·recording_audit.json·
시작/종료 `ros_state_*.json`·Linux `host_algorithm.jsonl`을
함께 저장한다. 공유할 때 **폴더 전체**를 전달한다. 카메라는 실제 ROS 입력의 원본 CompressedImage
바이트를 그대로 담으며 재인코딩하지 않는다. LiDAR PointCloud와 원본 패킷·GPS·IMU,
GT·인지·측위·TF·단계별 계획 경로·제어·LC 안전 검사·Cruise 선택/입력 궤적도 요청한다.
이미지와 LiDAR 재추론에 필요한 원본을 기록하므로 용량·디스크 I/O 부담이 크다.
비압축 Image까지 필요할 때만 `--include-decoded-images`를 추가한다(중복 기록 가능).

기본 시작 여유는 10GiB이며 장시간 용량을 보장하는 수치는 아니다. 여유 2GiB 미만이면
녹화만 SIGINT로 마감하고 차량은 직접 정지해야 한다. 제한시간 생략 시 Ctrl-C까지 녹화한다.
`--output-dir /큰디스크/경로`로 저장 위치를 바꿀 수 있다. `--list`는 ROS/파일 변경 없이 목록만 출력한다.
종료 검사는 실제 메시지 수와 빠진 core/진단 토픽을 보고할 뿐 안전 주행·시간 동기화를 인증하지 않는다.
구버전 노드·미활성 기능의 진단은 없을 수 있다. 최신 빌드가 필요하며, advertise만 있고 발행하지 않는
Cruise `debug/obstacle_cruise/trajectory`는 필수 토픽으로 취급하지 않는다.

### LAN 두 PC를 함께 계측하기

2026-10-24 사용자 LAN 구성은 **Windows MORAI 송신 `10.42.0.252` → Linux 알고리즘 수신
`10.42.0.1`**이다. 현재 LAN JSON도 같은 주소를 사용한다. 이는 저장된 설정 대조이지 실제
인터페이스 IP 확인이 아니며, 이 녹화기는 IP·방화벽·시계·센서·차량 설정을 변경하지 않는다.
제어 명령은 반대 방향으로도 흐른다. 과거 현장 원본 JSON의 `192.168.0.*` 주소와 혼합하지 않는다.

다음 시험에서 **기존 노드를 시작하기 전**, 필요한 진단만 명시적으로 켠다.
주행 중 이 옵션 때문에 노드를 재시작하지 않는다. `bevfusion`, `tracking`은 공용 alias 기준이다.

```bash
# 각 원래 담당 터미널에서 실행할 시작 명령. 녹화기가 대신 실행하지 않는다.
bevfusion --publish-frame-timing
tracking publish_processing_diagnostics:=true publish_association_diagnostics:=true
```

BEVFusion 진단은 LiDAR callback/header age, camera 선택 차이, raw/prepared queue,
전처리·RPC/server·검출 발행 비용과 누적 skip counter를 담는다. tracker 선택형 association 진단은
입력 측정 stamp/index와 ID/UUID, 연결·새 ID·억제·coasting·삭제 이유를 담는다. 각 launch 기본은
계속 OFF이며, 새 tracker 진단에는 갱신된 submodule과 대상 PC 재빌드가 필요하다.
`measurement_objects`만 녹화해도 내부 연결 사유가 기록되는 것은 아니다.
Map-CV·motion-policy 진단과 이 association 옵션은 모델·threshold·3초 예측 정책을 바꾸지 않는다.
LC safety/Cruise selection은 해당 코드가 지원하고 실제 구간에서 발행해야 저장된다.

Linux의 **별도 녹화 터미널**:

```bash
source scripts/workspace/source_aw149.sh
# 선택: 토픽 광고/설정만 읽는다. 실제 메시지 저장 검사는 종료 후에 수행한다.
bash scripts/rosbag/record/rosbag_for_development.sh --check
# Windows와 같은 세션 이름을 쓴다. Ctrl-C까지 녹화하며 --duration 300도 가능하다.
bash scripts/rosbag/record/rosbag_for_development.sh --session-id lan_261024_01
```

Linux 호스트 계측은 기본 1초 간격이며 CPU 전체·코어별, RAM/swap, NIC 누적/증분,
Linux UDP 오류, LiDAR 수신 포트 2368의 socket queue/drop, GPU를 함께 기록한다.
`--udp-port 2368 --udp-port 9291`처럼 추가 포트를 선택할 수 있고, `--telemetry-pid PID`로
특정 프로세스 CPU/RSS를 기록한다. GPU는 5초 간격·조회 timeout 1초다.
`--no-host-gpu` 또는 `--no-host-telemetry`는 계측만 끈다. 미지원·권한 오류는 0으로 위장하지 않는다.
계측 subprocess 실패가 차량이나 bag을 중단하지는 않으며 종료 audit에서 실패 여부를 확인한다.

Windows에는 저장소 전체·ROS·Python 설치 없이
[`record_morai_host.ps1`](rosbag/record/record_morai_host.ps1) 한 파일을 전달한다.
일반 PowerShell에서 다음과 같이 실행한다. 기존 파일은 덮어쓰지 않는다.

```powershell
# 파일을 둔 폴더에서 먼저 30초 사전 확인. 출력 부모 폴더는 이미 존재해야 한다.
powershell.exe -NoProfile -File .\record_morai_host.ps1 -Output "$HOME\Desktop\morai_check.jsonl" -SessionId "lan_check" -Duration 30 -IncludeGpu
# 실제 녹화: Linux와 session-id를 맞추고 끝나면 Ctrl+C. MORAI는 종료하지 않는다.
powershell.exe -NoProfile -File .\record_morai_host.ps1 -Output "$HOME\Desktop\morai_lan_261024_01.jsonl" -SessionId "lan_261024_01" -IncludeGpu
```

Windows는 CPU/코어별·RAM·NIC·IPv4 UDP·선택 MORAI 프로세스 CPU/RSS를 기본 2초 간격으로
기록한다. 실제 프로세스 이름이 다르면 `-ProcessId 1234` 또는 `-ProcessName "실제이름*"`를 준다.
`-IncludeGpu`는 선택 사항이다. 실행 정책에 막히면 PC 소유자와 확인하며 전역 실행 정책을 바꾸지 않는다.
PowerShell 7 Linux parser의 구문 오류 0개와 provider를 대체한 JSONL 생성·마감·카운터 모의 시험을
확인했다. **Windows PowerShell 5.1 및 실제 Windows provider·Ctrl+C는 아직 실기 미검증이다.**
실제 Windows에서 30초 로그가 정상 종료되고 `sample`이 있는지 확인한 뒤 사용한다.
Linux 계측·녹화 및 모의 시험은 이 Windows 실기 검증을 대신하지 않는다.

녹화가 끝나면 Windows JSONL도 Linux의 녹화 폴더 안에 넣어 **폴더 전체**를 공유한다.
같은 session-id는 로그를 묶는 이름일 뿐 두 PC 시계 동기화나 물리 LAN 지연을 증명하지 않는다.
[Windows UDP 통계](https://learn.microsoft.com/en-us/dotnet/api/system.net.networkinformation.udpstatistics)와
[NIC 통계](https://learn.microsoft.com/en-us/powershell/module/netadapter/get-netadapterstatistics)는
호스트/인터페이스 전체 범위이며, Windows port-error discard를 Linux `RcvbufErrors`와 같은 뜻으로
해석하지 않는다. MORAI 내부 FPS·생성 시각·송신 packet sequence는 이 호스트 통계만으로는 알 수 없다.
정확한 송신 전/네트워크 손실 분리에는 송신측 packet/FPS 증거가 추가로 필요할 수 있다.

종료 audit는 raw 재추론·센서/측위·트래커 연결 trace·검출/후처리 시간·LC/Cruise 이유별로
**실제 메시지 유무**를 구분하고, 센서/검출 주기·gap·header age와 LiDAR 패킷 수 histogram을 낸다.
`recorded=true`는 메시지가 있다는 뜻이지 안전 주행·정확도·전체 미션 관측 인증이 아니다.
Header sequence gap도 ROS/recorder 손실이나 publisher 재시작을 포함할 수 있어 UDP 손실로 단정하지 않는다.
닫힌 기존 bag은 다음 명령으로 ROS master 없이 검사할 수 있다(결과는 stdout, 원본 불변).

```bash
bash scripts/rosbag/record/rosbag_for_development.sh --audit-only /경로/development.bag
```

기능 검증: `python3 -m unittest discover -s scripts/rosbag/record -p 'test_*record*py'`.
`smoke_development_recording.py --output-dir 새폴더`는 **별도 localhost master**의 합성 입력으로만
녹화·마감·원본 바이트 보존을 확인한다. 기존 ROS master·MORAI·차량을 사용하지 않으며,
실제 LAN/GPU/Windows/판단 폐루프 성능 시험이 아니다.

`prepare_e12_postprocessing.py`는 이미 동결한 E12/E24 bag 검출의 **모든** 입력을 실제 C++ EKF
코어로 재생하고, 평가 eligibility·GT 원래 index·0점 근거를 별도 보존한다. `analyze_e12_tracking_support.py`는
동일 중심 AP를 검출/track 출력에 적용한 proxy와 일시 0점·속도·잔류를 계산한다. AMOTA/IDF1이나
native val AP가 아니다. `analyze_e12_map_horizon.py`는 실제 predictor를 3/5/6초로 각각 호출하며
미래 GT도 eligible·연속 index·bounded interpolation을 요구한다. `probe_e12_coasting.py`는 별도
진단 사본의 0.75/1초 및 성숙 차량 후보만 시험한다. 새 검출·ROS master·생산 설정은 변경하지 않는다.
모든 도구는 이전 output을 덮어쓰지 않으며, 로컬 동결 자료가 있는 분석 PC용이지 팀 설치기가 아니다.
실행 전 `source scripts/workspace/source_aw149.sh`를 사용한다.
[E12 후단 결과와 명령](../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-tracking-map-horizon-261023)을 따른다.

`analyze_e12_motion_ablation.py`는 같은 E12 트랙의 Cartesian CV·실제 CTRV·Map-CV를 동일한
3초 GT 코호트로 비교하고, 종/횡 오차와 초기/재취득 층을 분리한다. GT 종속도만 교체하는 고정
분기 진단은 평가용이며 운영 입력이나 새 정책이 아니다. `qa_e12_motion_ablation.py`가 별도 수식으로
코호트·예측·원래 frozen Map 지표·GT 속도의 과거 창·성분 지표를 교차 확인한다.
[65절의 조건·재현 명령](../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-map-motion-ablation-261023)을 따른다.

`qualify_e12_mixed_horizon.py`는 같은 동결 트랙에서 고주로만 6초로 연장하는 후보의
첫 3초·Top-K·객체 필드 ROS 직렬화를 비교한다. `analyze_map_mixed_horizon_live.py`는
실제 기록 TrackedObjects를 이용한 동일 입력 대조와 주행 bag 지표를 별도로 계산한다.
`run_validator_route_probe.py CONFIG --require-lanelet ID ...`는 허가된 live 시험에서만
사용한다. 실제 선택 route에 요구 lanelet이 순서대로 있는지 **제어 시작 전** 확인하고
기존 validator의 정지/P·녹화·정리를 그대로 사용한다. production 기본값은 바꾸지 않는다.
[혼합 horizon 동작·복귀와 근거](../base_ws/src/perception_object_pipeline/docs/MIXED_PREDICTION_HORIZON.md)를 따른다.

`analyze_native_horizon_live.py --execution DIR [--execution DIR ...] --output DIR`는
닫힌 validator bag의 동일 TrackedObjects를 native 3초/6초로 각각 계산한다. 혼합 연장과 달리
Top-K·순위·confidence·첫 3초 변경을 허용하고 양방향 prefix 대응을 보고한다. 실제 31/61 pose,
입력 불변·기록 출력 정합과 주행 중 정지/제동·출력 공백·callback 비용을 구분한다.
`--output`은 새 `planner_logs/analysis/` 하위 경로를 사용한다. 별도 주행 간 차이는 NPC 생성 시점과
부하도 포함하므로 horizon만의 인과 효과나 예측 정확도 개선으로 해석하지 않는다.

동결한 E12 결과를 RViz에서 보려면 이 분석 PC에서
`bash scripts/evaluation/view_e12_postprocessing.sh H04 1.0`을 실행한다.
H03(다른 고주로)·R01(로터리)도 같은 인자로 선택한다. 시작은 일시정지이며 **명령을 친 터미널의
Space**로 재생/정지, Ctrl+C 또는 RViz 창 닫기로 종료한다. GT 자홍·검출 초록·트랙 청록(관측 누락 중
주황), 현행 3초 노랑이다. 기존 `object_pipeline_live.rviz`의 Lanelet 지도·IONIQ 5·4개 카메라 dock과
차량 추종 시점을 그대로 사용한다. 5초 후보 주황·연구용 prefix5 파랑·속도·설명은
`E12 comparison (optional)` 그룹을 켜서 본다. 기본은 이 그룹을 숨긴다.
별도 localhost master에서 원본 센서·저장 marker·기록된 Ego 기반 TF를 재생하고 공용 지도·차량
표시만 실행한다. MORAI·BEVFusion 추론·live 측위·제어는 켜지 않는다.
팀 설치기가 아니며 로컬 `rviz_common_v1/<case>/scene.json`과 원본 bag이 필요하다.

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

후단의 `publish_processing_diagnostics:=true`를 명시하면 Map-CV·UNKNOWN 정책의
callback/header age·메시지 생성·publish 비용 JSON도 보조 녹화에 포함된다. 기본 OFF이며
30Hz tracking header age를 마지막 detector 관측 age로 해석하지 않는다.
`profile_pipeline_cpu_offline.py`는 명시한 Git reference와 현재 소스를 6 bag 등 동일 입력에서
wire-exact 비교한다. [CPU 검토와 포팅 gate](../docs/bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#cpu)를 따른다.

`replay_pipeline_diagnostics_offline.py`는 설치된 UNKNOWN→Map-CV만 별도 localhost
master·`/offline_probe/*` 토픽에서 실행한다. 원본 tracking bag의 empty/차량/사람/UNKNOWN
표본, 기준 wire와 출력 subset, 진단 OFF/ON을 비교하고 종료 시 소유 프로세스를 정리한다.
UDP driver·GPU·planner·제어·MORAI는 켜지 않는다. 시험 clock은 원래 object header에서
생성하며 실제 bag clock/센서 age를 재현하지 않는다. 실행 전 AW149를 source하고
새 output 경로를 지정한다. `--rates 1`은 약 30Hz, 기본의 3×는 queue 포화 시험이다.

`profile_pipeline_cpu_map_gc.py`는 실제 설치된 Map-CV에 전체 연속 bag을 스트리밍한다.
입력/출력 전체를 child heap에 쌓지 않고 기본 GC 설정·bounded trace·원래 header cadence를
유지한다. 수신 wire, 자연 GC 겹침, RSS와 queue 포화를 비교한다. 시험 driver의 수신 지연에는
부모의 wire 검증까지 포함되므로 sensor age나 callback 시간으로 해석하지 않는다.

`prepare_base5_nearest_compile_continuous.py`와 `experiment_base5_nearest_compile.py`는
원본 입력 hash·stamp·제외 공백을 보존하고 연구 후보의 eager A/A·compile을 분리한다.
`audit_tracking_continuity_variants.py`와 `audit_tracking_continuity_variant_review.py`는
그 출력을 원형 C++ 코어에 재생해 평균·분위수·최대와 association/잔류 창을 대조한다.
차량 초기 방향과 재취득은 `audit_vehicle_reacquisition_core.py`·
`qualify_vehicle_innovation_gate.py`·`build_vehicle_direction_probe.py`·
`audit_vehicle_direction_probe.py`로 독립 코어와 후보를 비교한다. `audit_vehicle_map_prediction.py`는
같은 GT·시각에서 실제 Map-CV의 1/2/3초 오차를 대조한다. `build_vehicle_ros_candidates.py`와
`replay_vehicle_ros_candidates.py`는 별도 localhost master의 연구 wrapper·입력/처리 barrier를
사용하며 production binary를 설치하지 않는다. 원본 입력 age와 출력 시계를 분리하고
guard의 중심·ID 동등성 검사를 통과해야 효과를 계산한다.
현재 결정과 재현 범위는 [차량 검증](../docs/bevfusion/261014_VEHICLE_DIRECTION_REACQUISITION_REVIEW.md)을 따른다.
`audit_base5_offline_box_center_origin.py`는 native annotation의 물리적 중심과 tagged
출력 원점을 감사한다. 이 도구들 자체가 production 옵션이나 GT를 바꾸지는 않는다.

BEVFusion bridge의 stage JSON은 `BEVFUSION_PUBLISH_FRAME_TIMING=true`를
`scripts/runtime/run_bevfusion.sh` 앞에 명시해서 선택한다. 기본 OFF이며 canonical 및
Base5 trial의 `frame_timing`을 recorder가 모두 요청한다. 기존 latency 범위를 재정의하지
않고 raw/prepared queue·host 준비·server·publish를 나눈다. publish 시간은 downstream
전달 완료 시간이 아니다. 자세한 범위는 [진단 계약](../docs/bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#diagnostics)을 따른다.

## 원시 LiDAR 패킷·카메라 녹화

50/51패킷 scan 경계와 first-packet stamp를 재검토할 때는 가벼운 latency trace만으로는
부족하다. `scripts/rosbag/record/record_lidar_packet_probe.py`는 raw VelodyneScan과
PointCloud2·4카메라 JPEG·Ego/Object GT·IMU를 짧게 함께 기록한다. 현재 운영 기본값은
900 RPM·50패킷·첫 패킷 stamp다. 녹화기는 driver 설정을 바꾸지 않고 기본으로 50 계약을 검사한다.
과거 51 입력 재현은 driver에 명시적 51과 recorder에 `--expected-packets 51`을 함께 지정한다.
`--list`는 ROS 없이 목록만 표시하고, `--check`는 live 수신만
확인한다. `--run`만 bag을 기록하며 ROS 노드 기동·파라미터 변경·차량 제어는 하지 않는다.

네트워크 연결 이후 자동 로드·운전·녹화까지 맡기는 경우에는
`run_lidar_packet_probe_auto.py --run`을 쓴다. 이 실행기는 기존 validator를 GT 모드로
재사용하며 별도 실험 설정을 생성한다. MORAI는 `ego_network_validator_local`에 연결하고
사용자 `rc`만 유지한다. 다른 캠페인과 동시에 실행하지 않는다. standalone recorder와
달리 이 자동 실행기는 시험 노드 기동·시나리오 로드·차량 제어를 수행한다.

시나리오 이름·로컬 UDP 설정·세 터미널 명령과 수동 운전 구간은
[benchmark 준비 절차](../configs/scenario/perception_benchmark/README.md#2026-10-14-원시-lidar-패킷-실험-준비)를 따른다.
과거 51 원본과 완료된 50/51 비교는 이력으로 보존한다. 현재 운영·과거 재현의 구분은
[LiDAR 전환 안내](../docs/bevfusion/BASE5_BEVFUSION_PRODUCTION_HANDOFF.md#lidar-50-적용과-51-복귀)를 따른다.
새 비교를 할 때만 명시적 정책별 입력을 확보하고 MORAI OFF 상태에서 후속 분석한다.

오프라인 분석 도구는 실행 역할을 구분한다. `lidar_packet_reassembly.py`는 bag을 읽어
packet 경계/시각만 감사한다. `decode_lidar_packet_candidates.py`는 별도 ROS master와
transform-only nodelet을 띄우며 UDP driver·제어는 실행하지 않는다. 원본 51 cloud의
typed fields 동등성을 먼저 검사한다. `infer_lidar_packet_candidates.py`는 입력 선정과
GPU 추론을, `analyze_lidar_packet_candidates.py`는 동일 operating policy의 결과를
분리한다. `audit_tracking_continuity_packets.py`는 독립 C++ 코어를 재생한다.
각 `--help`의 새 output 경로와 source/calibration/weight 계약을 확인하고 실행한다.
현재 결과와 해석 한계는 [packet 검토](../docs/bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#packet)에 있다.

선택적 live LiDAR 비교 인자는 `lidar.launch`의 `npackets`/`cut_angle`과 상위
`morai_udp_interface.launch`의 `lidar_npackets`/`lidar_cut_angle`이다. 2026-10-14 운영 default는
50이며 51 재현에는 명시적 `lidar_npackets:=51`이 필요하다. cut 단위는 라디안이다.
원시 녹화기의 기본 검사도 50이며 과거 51 재현에는 `--expected-packets 51`을 사용한다.

새 정지·보행 후보 비교는 `run_perception_candidate_probe.py --run`으로 실행한다.
유한 시나리오 사본·Ego 제동·실제 로드 UID·충돌·드리프트·원본 저장을 검사하며 본수집이 아니다.
`experiment_live_perception_candidates.py`의 prepare/infer와
`analyze_live_perception_candidates.py`가 같은 입력의 반복·depth hint·C++ association을 대조한다.
실제 서버는 `check_live_candidate_server_parity.py`, tagged 원점→두 fresh tracker는
`run_live_box_origin_gate.py`와 `analyze_live_box_origin_gate.py`로 분리한다.
자동 GT 주행은 `run_lidar_live_policy_campaign.py`이며 첫 안전/주행 실패에서 중단한다.
현재 적용 결정·실패 원본·실행 범위는 [Live 후보 검토](../docs/bevfusion/261014_LIVE_PERCEPTION_CANDIDATE_REVIEW.md)를 따른다.
기본 LiDAR 정책은 50이며 재시작 시 driver private namespace의 이전 override만 지운다.

새 독립 차량 3장면은 `build_independent_vehicle_probe_scenes.py`로 준비하고
`run_independent_vehicle_lidar_campaign.py --scene-manifest <plan.json> --output <새 폴더> --run`으로
GT 제어·실제 51/50 녹화를 수행한다. 준비 동안 실제 P·정지를 확인한 뒤 D 제어기로 전환한다.
`--finalize-existing`는 모든 원본이 검사 통과한 경우에만 P/50 복귀를 재확인하며 주행은 반복하지 않는다.
다른 validator·제어기·UDP 수신기와 동시에 실행하지 않는다.

`prepare_independent_vehicle_inputs.py`는 움직이는 Ego와 ScenarioLoad 이전 구간을 구분한다.
`scene_epoch_valid` 이후 초기 Park부터 fresh tracker로 비교하며 이전 장면 입력을 섞지 않는다.
`audit_independent_vehicle_direction.py`와 `qualify_vehicle_covariance_recovery.py`는 별도 코어 원형이다.
`lidar_followup_packet_audit.py`의 실제 수신 age와 가상 재조립 span을 구분하고,
`prepare_source_footer_gt_sensitivity.py`는 GT-only 현재 중심 민감도에만 사용한다.
현재 근거·보류 원형·원본 경로는 [독립 검증](../docs/bevfusion/261014_INDEPENDENT_VEHICLE_AND_LIDAR_50_51_REVIEW.md)을 따른다.
