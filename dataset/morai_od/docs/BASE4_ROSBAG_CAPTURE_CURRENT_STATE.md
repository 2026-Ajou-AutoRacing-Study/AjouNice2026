# Base4 rosbag 수집 — CURRENT STATE

- 상태: **CURRENT / INFRASTRUCTURE VALIDATED**
- 마지막 검증: 2026-09-20
- 대상: raw 4-camera + LiDAR BEVFusion, 이후 Base4-Temporal

## 1. 현재 결론

MR에서 다음 hybrid 계약이 실제 2-scene 자동 수집과 BEVFusion dataset 변환까지
통과했다.

```text
Camera/LiDAR/GPS/IMU          <- UDP sensor-only gateway
Ego/Object/ScenarioLoad/Ctrl <- rosbridge minimal Ego Network
scene boundary              <- /dataset/scenario_marker
source of truth             <- continuous rosbag
```

`SaveSensorData`, semantic/depth image, ReplayInfo는 사용하지 않는다. 현재 검증된 변환
결과는 **single-frame BEVFusion smoke dataset**이다. 12--24 frame causal sequence index와
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

### 4.2 Recorder

Runner보다 먼저 시작한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash

RUN_TAG="$(date +%y%m%d_%H%M%S)_base4_pilot" \
dataset/morai_od/scripts/record_base4_temporal_bag.sh \
  "$PWD/rosbags/datasets/base4_temporal/pilot"
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

## 7. 다음 구현 gate

1. Base4 고주로 motion family와 clip 길이를 manifest에 고정한다.
2. scene 단위 train/val split만 사용하고 작은 pilot에는 dataset test split을 만들지 않는다.
3. LiDAR timestamp 기준 Ego/Object pose interpolation은 유지한다.
4. 연속 frame의 실제 `dt`, track identity, current-frame target과 causal sweep index를 만든다.
5. teleport/load 구간은 marker 밖이므로 sequence에 포함하지 않는다.
6. raw bag과 runtime metadata는 보존하고 거리/visibility 정책은 파생 info에서 선택한다.
7. temporal info validator와 Docker dataloader smoke를 통과한 뒤 본 수집으로 간다.
