# Base4 rosbag 수집 — CURRENT STATE

- 상태: **CURRENT / CONTROL-SAFETY REVALIDATED**
- 마지막 검증: 2026-09-20
- 대상: raw 4-camera + LiDAR BEVFusion, 이후 Base4-Temporal

## 1. 현재 결론

MR에서 다음 hybrid 계약이 실제 2-scene 자동 수집과 BEVFusion dataset 변환까지
통과했다.

```text
Camera/LiDAR/GPS/IMU          <- UDP sensor-only gateway
Ego/Object/ScenarioLoad/Ctrl <- rosbridge minimal Ego Network
scene boundary              <- /dataset/scenario_marker
source of truth             <- continuous stream -> gated scene rosbag
```

원본 전송 계약은 연속 stream이지만 디스크 기록은 이제 **48 MiB snapshot ring buffer를
scene별로 flush하는 gated recorder**를 기본으로 한다. 센서 stream은 끊지 않으며,
`capture_end`가 정상 완료된 scene만 bag 하나로 저장한다. 실제 첫 유효 frame의
`sequence_start`가 snapshot window를 다시 열어 고속 warm-up 중 오래된
`capture_start`가 밀려나도 유효 구간을 보존한다. load/reconnect 대기 구간을
디스크에 계속 쓰는 기존 continuous recorder는 진단 fallback으로만 유지한다.

`SaveSensorData`, semantic/depth image, ReplayInfo는 사용하지 않는다. 현재 검증된 변환
결과는 **연속 23--35 LiDAR frame raw window와 16-frame BEVFusion smoke dataset**이다. causal sequence index와
velocity target을 만드는 Base4-Temporal converter는 다음 구현 단계이며, 이번 PASS를 곧
temporal dataset 완성으로 해석하면 안 된다.

## 2. launcher별 파일

| 용도 | MR | MRP |
|---|---|---|
| launcher | `~/MoraiLauncher_Lin` | `~/MoraiLauncher_Lin_per` |
| network profile | `configs/network/ego_network_mr_minimal_hybrid.json` | `configs/network/ego_network_mrp_minimal_hybrid.json` |
| 설치 이름 | `ss_ego_minimal_hybrid.json` | `ros_base4_minimal_hybrid.json` |
| ROS overlay | `base_ws/install_aw149` | 위 overlay + `morai_personal_ws/devel` |

두 tracked profile은 현재 byte-identical이지만 launcher별 provenance와 설치 경로를 섞지
않는다. MR full 23-entry ROS profile은 장시간 sensor-rate 저하가 재현되어 금지한다.

## 3. 녹화 topic 계약

필수 11개:

```text
/sensor/lidar1
/sensor/camera/front/compressed
/sensor/camera/front_left/compressed
/sensor/camera/front_right/compressed
/sensor/camera/back/compressed
/Ego_topic
/Object_topic
/dataset/scenario_marker
/ctrl_cmd
/sensor/gps
/sensor/imu
```

진단용 optional은 Collision 두 topic, simulator process state, `/tf`, `/tf_static`이다.
optional topic이 없다는 이유로 유효한 sensor/GT run을 버리지 않는다.

## 4. 실행 순서

ROS master, launcher별 rosbridge, sensor-only UDP, MORAI GUI 연결까지의 전체 순서는
[`morai_mr_mrp_hybrid_scenario_capture_runbook.md`](morai_mr_mrp_hybrid_scenario_capture_runbook.md)를
따른다.

### 4.1 Scenario와 minimal sidecar 설치

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/install_dynamic_capture_set.py \
  <source-manifest.json> \
  --network-template configs/network/ego_network_mr_minimal_hybrid.json \
  --install-dir ~/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025 \
  --online-bag-only \
  --runtime-manifest <runtime-manifest.json>
```

MRP에서는 network template과 install directory를 MRP용으로 바꾼다. installer report의
`required_topics`는 `/Ego_topic`, `/Object_topic`, `/ScenarioLoad` 세 개여야 하며
`online_bag_only=true`여야 한다.

본 수집 source manifest에는 반드시 현재 GUI와 같은 `sensor_config` 및 SHA-256을 넣는다.
Runner가 `<runtime-metadata-dir>/dynamic_metadata/calibration.json`을 만들었는지 recorder를
끄기 전에 확인한다. 이번 2-scene geometry pilot manifest에는 이 필드가 없어, 동일한
`Sensor_competition_local.json`으로 취득한 기존 Base3-Online calibration(hash
`84f171a6...`)을 smoke 변환에 재사용했다. 본 수집에서는 이 fallback을 허용하지 않는다.

### 4.2 Gated recorder — 기본값

Runner보다 먼저 시작한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash

RUN_TAG="$(date +%y%m%d_%H%M%S)_base4_pilot" \
dataset/morai_od/scripts/record_base4_temporal_gated_bag.sh \
  "$PWD/rosbags/datasets/base4_temporal/pilot"
```

- 기본 ring buffer: `48 MiB`.
- 기본 post-roll: `0.25 s`.
- 기본 안전 marker window: `3.40 s`.
- 최소 잔여 공간: `35 GiB`.
- 성공 scene마다 약 `50.5 MB` bag 하나와
  `*_snapshot_events.jsonl`의 scene-to-bag 절대 경로 대응을 남긴다.
- 실패/interrupted scene은 기본적으로 flush하지 않는다.

`48 MiB`는 시간 길이가 아니라 serialized byte 크기다. 카메라 bitrate가 바뀌면 보존
시간도 바뀐다. 모든 campaign은 첫 20--30 scene에서 marker 양끝이 bag에 남는지 audit한
뒤 계속한다. `sequence_start`가 잘리거나 window가 `3.40 s`를 넘으면 buffer 또는 capture
window를 조정한다.

연속 recorder A/B가 필요할 때만 다음 fallback을 쓴다.

```bash
RUN_TAG="$(date +%y%m%d_%H%M%S)_base4_continuous" \
dataset/morai_od/scripts/record_base4_temporal_bag.sh \
  "$PWD/rosbags/datasets/base4_temporal/continuous_probe"
```

### 4.3 Runner

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash

python3 -u dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest <runtime-manifest.json> \
  --capture-dir <runtime-metadata-dir> \
  --no-timestamp-prefix \
  --morai-scenario-dir ~/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025 \
  --motion-mode rolling-grouped \
  --online-bag-only \
  --scenario-load-attempts 3 \
  --capture-timeout-sec 30 \
  --continue-on-scene-error
```

마지막 `capture_end` 뒤 recorder를 `Ctrl-C`로 종료한다.

### 4.4 Passive progress monitor

모니터는 `summary.json`과 append-only JSONL만 읽으며 ROS topic을 구독하거나 recorder,
runner, sensor stream에 개입하지 않는다. 전체 manifest ETA와 별도로 중간 감사 지점을
표시하려면 `--milestone-scenes`를 사용한다.

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/monitor_ros_capture_progress_v2.py \
  --run-dir /home/luv/morai_cap/<runtime-dir> \
  --manifest dataset/morai_od/manifests/<runtime-manifest.json> \
  --milestone-scenes 500
```

화면에는 attempted/completed/failed scene, family별 완료량, p50/p95 scene duration,
sequence rate, disk, checkpoint age, 전체 ETA와 500-scene 감사 checkpoint ETA가 표시된다.
이 모니터의 경고는 runner를 자동 중단하지 않는다.

## 5. 감사와 임시 single-frame 변환

```bash
BAG=<capture.bag>

python3 dataset/morai_od/scripts/audit_base3_online_bag.py \
  "$BAG" \
  --json-output artifacts/datasets/base4_pilot_bag_audit.json

python3 perception/detection3d/bevfusion/tools/build_base3_online_bevfusion_infos.py \
  "$BAG" \
  --calibration <runtime-metadata-dir>/dynamic_metadata/calibration.json \
  --asset-offsets dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json \
  --output-root artifacts/datasets/morai_base4_pilot_smoke \
  --serialized-root /datasets/base4_pilot_smoke \
  --samples-per-scene 3 \
  --minimum-gt-points 5 \
  --state-alignment interpolate \
  --distance-filter box-range
```

검증되지 않은 visible asset을 GT에서만 제거하면 false-negative supervision이 된다.
그 asset의 scene 전체를 제외해야 할 때만 다음을 명시한다.

```bash
  --drop-scene-asset WoodBox
```

이 옵션은 dropped scene과 asset을 `conversion_audit.json`에 남긴다. offset catalog에 없는
asset을 옵션 없이 만나면 변환기는 계속 fail-fast한다.

## 6. 2026-09-20 MR pilot 근거

- raw bag: `rosbags/datasets/base4_temporal/mr_minimal_hybrid_pilot/260920_202943_mr_minimal_hybrid_2scene_0.bag`
- runtime: `/home/luv/morai_cap/260920_202957_base4_mr_minimal_2scene/`
- scene: CargoBox control + WoodBox probe, 각 3 trigger
- 결과: 2/2 completed, failure 0
- capture-window rates:
  - LiDAR `11.90 / 12.63 Hz`
  - 4-camera `29.93 / 29.60 Hz`
  - Ego `47.96 / 48.01 Hz`
  - Object `47.60 / 47.65 Hz`
- camera--LiDAR age: p50 `2.0--3.7 ms`, p95 약 `26 ms`
- raw audit: PASS, error 0
- smoke conversion: WoodBox scene 명시 제외 후 CargoBox 1 scene, 3 frame,
  Obstacle GT 3개, MMDetection3D info validator PASS

### 6.1 48 MiB gated recorder A/B

- 유효 raw bag:
  `rosbags/datasets/base4_temporal/gated48_2scene_pilot/`
- runtime:
  `/home/luv/morai_cap/260920_210739_base4_gated48_2scene/`
- 32 MiB A/B: 약 `2.59 s`만 보존해 `2.76--2.80 s` marker window의
  `capture_start`가 잘림. **기각**.
- 48 MiB A/B: CargoBox/WoodBox 모두 start/end marker 보존, scene당 약
  `50.5 MB`.
- window LiDAR: `11.45 / 13.06 Hz`; camera: `29.74--29.92 Hz`.
- camera--LiDAR max age: `< 88 ms`, 100 ms 초과 0.
- raw audit: error 0, PASS.
- CargoBox 3-frame single-frame 변환 및 info validator PASS.
- scene-to-bag 파일 대응 sidecar의 synthetic write-completion test PASS.

현재 실측 용량이면 1,200 scene은 약 `61 GB`, 1,500 scene은 약 `76 GB`다.
연속 녹화 실측 약 `38 GiB/h`와 달리 scene load 대기시간에는 디스크가 증가하지 않는다.

### 6.2 Base4 고속 고주로 live pilot

- generator: `generate_base4_temporal_highway.py`
- runtime: `/home/luv/morai_cap/260920_213125_base4_highway_live12_retry2`
- raw bags: `rosbags/datasets/base4_temporal/highway_live12/`
- 성공: 실제 merge conflict `2/2`, post-merge cruise/overtake `9/9`.
- 검증 Ego command: `50`, `70`, `85`, `100`, `115`, `120 km/h`.
- NPC 시작/목표속도는 sample spawn 계약의 `40/50/60` 세 값만 사용했다.
- 유효 window: `1.91--2.76 s`, LiDAR `23--35 frame`, 약 `11.59--12.75 Hz`.
- 11 scene을 16 frame씩 변환해 train `144`, val `32`, test `0`을 생성했다.
- ramp 초기 접근 1 scene은 actor가 제어권 전환 전에 interaction 구간을 벗어나 반복
  실패했다. planning 반응 우선순위도 낮으므로 production generator에서는 제거하고 실제
  merge conflict와 post-merge 추월에 quota를 집중한다.
- raw audit의 316 LiDAR sample 중 camera nearest age가 100 ms를 약간 넘은 frame이
  camera별 3개(`0.949%`, max `112.4 ms`) 있었다. raw는 보존하고 converter에서
  tolerance를 넘는 candidate를 선택하지 않는다.

### 6.3 중단·격리된 최초 고주로 production campaign

- 최종 campaign tag: `p12v3`, 총 `1,200 scene`.
- manifest: `dataset/morai_od/manifests/base4_temporal_highway_p12v3_runtime.json`.
- raw output: `rosbags/datasets/base4_temporal/highway_p12v3/`.
- runtime output: `/home/luv/morai_cap/260920_215947_b4h_p12v3/`.
- split: train `960 scene`, val `240 scene`, dataset test 없음.
- 구성: 실제 merge conflict `300 scene`, post-merge cruise/overtake `900 scene`.
- merge Ego는 `50/54/58 km/h`만 사용하며 모든 값이 59 km/h 미만이다.
- post-merge Ego는 `70/85/100/115/120 km/h`, NPC는 sample 계약의
  `40/50/60 km/h`이고 모든 NPC가 시작속도=목표속도다.
- post-merge 유효 Ego lane은 세 개를 `20/40/40%`로 구성했다. 차로와 interaction
  role의 주기 상관으로 한 차로가 사라지던 문제는 schedule offset으로 제거했다.
- merge의 `2030--2060` route index만 사용한다. 그보다 이른 지점은 live test에서
  actor가 실제 interaction gate에 들어오지 않는 경우가 반복되어 ramp 접근 표본으로
  오인될 수 있으므로 제외했다.
- 31-scene 초기 production audit는 `28 completed / 3 failed`(`9.7%`)였다. 실패는
  MORAI spawn이 gate에 나타나지 않은 경우이며 gated recorder가 bag을 저장하지 않는다.
  본 run 종료 후 실패 scene 수만큼 별도 seed로 보충한다.
- 이 campaign은 **중단됐으며 학습에 사용하지 않는다.** 저장된 초기속도로 Ego가
  external-control 준비 전에 ramp 곡선을 직진 통과한 것이 확인됐다.
- runtime과 raw는 원인 근거 보존을 위해 격리했으며 자동 재개하지 않는다.

### 6.4 제어 안전성 수정과 r9 smoke

- Ego load speed를 `0 km/h`로 고정했다.
- control handoff 전 drift가 `0.75 m`를 넘으면 같은 scene을 다시 load한다.
- rolling capture도 motion arm, 최소 Ego speed, stable-speed gate를 적용한다.
- Ego 가속 중 NPC가 먼저 전진하는 거리를 역산해 spawn 위치를 보정한다.
- post-merge 기준 속도는 `70/85/100 km/h`로 축소했다.
- `/home/luv/morai_cap/260920_b4_r9_smoke`와
  `/home/luv/morai_cap/260920_b4_r10_100k_smoke`에서 merge 및 70/100 km/h cruise
  `3/3 scene`, `48/48 frame`, failure 0으로 통과했다.
- 상세 수치와 deskew 결과:
  [`260920_base4_deskew_and_control_safety_evidence.md`](260920_base4_deskew_and_control_safety_evidence.md).

## 7. 다음 구현 gate

1. 새 seed의 20--30 scene limited pilot에서 70/85/100 km/h와 세 lane을 먼저 감사한다.
2. failure, collision, pre-control drift, steering saturation이 모두 기준 안이면 본
   campaign을 새 tag로 생성한다. 중단된 `p12v3`는 재사용하지 않는다.
3. scene 단위 train/val split만 사용하고 작은 pilot에는 dataset test split을 만들지 않는다.
4. LiDAR timestamp 기준 Ego/Object pose interpolation은 유지한다.
5. 연속 frame의 실제 `dt`, track identity, current-frame target과 causal sweep index를 만든다.
6. teleport/load 구간은 marker 밖이므로 sequence에 포함하지 않는다.
7. raw bag과 runtime metadata는 보존하고 거리/visibility 정책은 파생 info에서 선택한다.
8. temporal info validator와 Docker dataloader smoke를 통과한 뒤 본 수집으로 간다.
