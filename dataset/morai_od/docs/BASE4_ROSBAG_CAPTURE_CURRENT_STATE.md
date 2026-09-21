# Base4 rosbag 수집 — CURRENT STATE

- 상태: **CURRENT / VELOCITY-DISTRIBUTION DIAGNOSTIC PREPARED**
- 마지막 검증: 2026-09-22
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
결과는 scene당 연속 LiDAR window, LiDAR 시각의 Ego/Object 보간, 9D box의
`vx, vy`, Ego 정합 causal 3-sweep과 point `dt`까지 포함한다. 24-scene 제한 파일럿에서
raw 485 frame과 deskew 467 frame을 만들고 V1/T1 dataloader 및 1-epoch smoke를
통과했다. 다만 이 수량은 **파이프라인 검증용**이며 성능이나 분포 완성을 주장하는
dataset이 아니다.

2026-09-21 MR 동기 annotation 교차검증에서 MR의 raw `/Ego_topic.heading`이 scenario
좌표계보다 약 `+1.35 deg` 회전되어 있음이 확인됐다. Competition localization과 같은
`-1.347 deg`를 적용하면 동일 asset의 두 관측 간 box-center 잔차가
`p50 0.192 -> 0.046 m`, `p95 0.228 -> 0.081 m`로 감소했다. 따라서 **MR에서 녹화한
bag의 GT, ego2global, causal sweep pose, ego deskew pose에는 반드시**
`--ego-heading-correction-deg -1.347`을 사용한다. MRP는 `0.0`이다. MRP 결론은 기존
동기 캡처 `31,644` observation 전체에서 0도와 잘못된 -1.347도를 비교해 각각
center residual p50 `0.126 m`와 `0.319 m`가 나온 것으로 확정했다.

이 발견 전에 변환한 `p24v1` 및 `hwy3h1` info와 그 학습 checkpoint는 파이프라인
EVIDENCE로만 보존한다. 원본 rosbag 재취득 없이 전체 hwy3h1을
`morai_base4_temporal_highway_hwy3h1_mrhc_v2`로 재변환했고, 13,749 frame의 구조 및
temporal audit가 통과했다. 다음 V1/T1 학습은 이 corrected V2만 사용한다.

Corrected V2의 V1/T1 E4까지 학습·고주로 rosbag 평가한 결과, detector velocity는
실주행에서 약 `3.8--4.0 m/s` AVE로 일반화에 실패했다. label 자체는 연속 BBox-center
미분과 p50 `0.075 m/s`, p95 `0.379 m/s` 수준으로 일치했지만, 완료된 학습 frame의
차량 속도는 주로 40/50 km/h에 몰렸다. V1/T1 모두 rosbag에서 12--13 m/s 부근으로
출력이 수렴한 현상을 temporal 구조 문제와 속도 분포 문제로 분리해야 한다.

이를 위해 기존 mission profile은 그대로 둔 채 `velocity-diagnostic` opt-in profile과
28-scene MR pilot을 준비했다. 합류 7개와 합류 후 순항 21개에서 NPC 요청 속도
10/20/30/40/50/60/70 km/h를 자동 순환한다. Ego는 runner가 정지 상태부터 waypoint를
따라 자동 운전하며 사용자가 직접 throttle/steering을 넣지 않는다. 실제 달성 속도는
요청값이 아니라 수집 후 `/Object_topic`과 BBox-center trajectory로 감사한다. 상세 경로,
scene별 속도와 실행법은
[`260922_base4_velocity_diagnostic_pilot_runbook.md`](260922_base4_velocity_diagnostic_pilot_runbook.md)에
고정한다.

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
  --ego-heading-correction-deg -1.347 \
  --distance-filter box-range
```

위 값은 **MR 전용**이다. MRP bag은 옵션을 생략해 기본 `0.0`을 유지한다. 변환 결과의
`conversion_audit.json`과 PKL `metainfo`에 실제 사용값이 기록된다.

3-sweep/velocity dataset을 만든 뒤에는 구조 validator와 별도로 temporal quality audit를
실행한다. 이 검사는 scene split leakage, 양수 sweep age, SE(3), Ego/Object/camera 동기
age, m/s 속도 범위와 BBox-center 궤적의 finite-difference velocity 일관성을 확인한다.

```bash
python3 perception/detection3d/bevfusion/tools/validate_base3_bevfusion_infos.py \
  --info-dir artifacts/datasets/<dataset> \
  --capture-root artifacts/datasets/<dataset> \
  --serialized-capture-root /datasets/<portable-name>

python3 perception/detection3d/bevfusion/tools/audit_base4_temporal_quality.py \
  --info-dir artifacts/datasets/<dataset>
```

`temporal_quality_audit.json`이 `valid: true`가 아니면 학습하지 않는다. 특히
`ObjectStatus.velocity`의 km/h 값을 m/s로 오인하는 3.6배 label 오류는 speed ceiling과
BBox-center trajectory 일관성 검사가 함께 차단한다.

검증된 scan-end Ego deskew derived dataset을 만들 때만 다음을 추가한다. 원본 bag은
보존되며, 보정은 dataloader가 아니라 이 변환 단계에서 한 번 수행된다.

```bash
  --lidar-deskew ego-pose-2d \
  --lidar-deskew-reference end \
  --deskew-max-point-time-span-sec 0.15 \
  --deskew-invalid-policy drop
```

E25 production과 기존 Base3 재현은 계속 `--lidar-deskew off`가 기준이다. 채택 근거와
무재학습 A/B는
[`260920_base4_deskew_and_control_safety_evidence.md`](260920_base4_deskew_and_control_safety_evidence.md)에 있다.

팀 전달 package도 raw와 `ego_deskew_scan_end`를 별도 이름으로 만든다. Deskew package는
이미 보정된 `lidar.bin`을 포함하므로 dataloader는 그대로 읽지만, 해당 checkpoint의
online bridge에는 동일한 `ego_pose_2d/end` 입력 계약이 필수다. 현재 전체 Base4 deskew
package는 아직 생성하지 않았으며, 기존 공유 데이터가 자동으로 deskew된 것도 아니다.

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
- NPC 시작/목표속도는 pipeline 통제용 anchor인 `40/50/60` 세 값만 사용했다.
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
- post-merge Ego는 `70/85/100/115/120 km/h`, NPC는 pilot anchor인
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

### 6.5 24-scene 제한 파일럿 `p24v1`

- seed: `260921101`, 목표 구성: merge `6`, post-merge `18`.
- runtime: `/home/luv/morai_cap/260921_b4_p24v1`.
- raw bag: `rosbags/datasets/base4_temporal/p24v1/`.
- 결과: `24 attempted / 23 completed / 1 failed`, bag `23`개. 실패한
  `b4p24v1ddmc006`은 frame 14에서 actor가 interaction gate를 이탈했다.
- 유효 capture window의 LiDAR는 약 `11.2--12.6 Hz`, camera는
  `28.2--30.3 Hz`, Ego/Object는 약 `44--49 Hz`였다.
- camera--LiDAR nearest age는 p50 `1.67--3.29 ms`, p95 약 `27--28 ms`다.
- 전체 `736` LiDAR candidate 중 bag edge의 `2` frame만 camera age가
  `100 ms`를 넘었다(`0.272%`, max 약 `113 ms`). Raw audit에는 엄격한 error로
  남지만 converter가 해당 frame을 제외하므로 dataset 오염은 없다.
- Ego header stamp가 `1 ms` 역행한 state 한 건은 converter의 timestamp sort 후
  interpolation하며 audit에 기록한다.

변환 결과는 다음과 같다.

| 파생 dataset | scene train/val | frame train/val | Vehicle GT | 용도 |
|---|---:|---:|---:|---|
| `morai_base4_temporal_highway_p24v1_raw` | `19/4` | `395/90` | `485` | V1/T1 기준 입력 |
| `morai_base4_temporal_highway_p24v1_ego_deskew` | `19/4` | `383/84` | `467` | deskew 독립 A/B |

두 dataset 모두 info validator를 통과했다. Sparse GT는 raw `39`, deskew `57`,
30 m box-range 밖 GT는 raw `177`, deskew `170`개가 제외됐다. positive Vehicle이
남지 않은 frame도 학습에서 제외하지만 raw bag은 그대로 보존한다. Split은 scene
단위이며 test split은 만들지 않았다.

Raw info에는 LiDAR 시각의 9D box `[x,y,z,l,w,h,yaw,vx,vy]`, Ego pose,
현재 frame과 직전 두 frame의 `sweep2lidar`, 양수 time lag가 들어간다. Object topic의
body-frame `km/h` velocity는 world frame을 거쳐 target Ego frame의 `m/s`로 변환한다.
Velocity target의 기준점은 `/Object_topic.position` asset pivot이 아니라 catalog로
보정한 geometric BBox center다. 같은 unique ID의 LiDAR 시각 전후 Object state에서
yaw rate를 구하고, catalog의 pivot-to-center offset에 `omega x r`을 더한다. 전후 state가
없거나 support span이 `0.10 s`를 넘거나 yaw rate가 `2 rad/s`를 넘으면 해당 instance만
pivot velocity로 안전하게 fallback하며 source와 보정량을 metadata/audit에 기록한다.
현재 고주로 Base4 scenario scale은 모두 `1`이고, 임의 scenario scale 보정은 MORAI의
공식 계약이 확정되기 전까지 이 경로에 넣지 않는다.
학습 loader는 과거 point를 Ego 정합한 뒤 다섯 번째 channel에 `dt`를 넣는다. GT Object
motion으로 과거 point를 현재 위치로 옮기지는 않는다. Positive GT가 없는 중간 frame도
causal sweep 후보로 보존해 history가 인위적으로 건너뛰지 않게 한다. 수정 후 raw의
sweep lag는 p50 `0.129 s`, p95 `0.181 s`, max `0.246 s`이며 `0.25 s` 초과가 없다.

자세한 학습 smoke와 기존 평가 bag 분류는
[`../../../docs/bevfusion/260921_BASE4_TEMPORAL_P24_PIPELINE_SMOKE.md`](../../../docs/bevfusion/260921_BASE4_TEMPORAL_P24_PIPELINE_SMOKE.md)에 있다.

### 6.6 고속 연속속도 48-scene 파일럿

`high-speed-pilot` profile을 추가해 post-merge 직선에서 Ego `80--120 km/h`, NPC
`25--80 km/h`를 연속 stratified schedule로 수집했다. 전방 추월, 측방 통과, 추월 후
후방 이탈 역할을 `2:1:1`로 배정했다.

- 최종 수집: `48/48 scene`, `768/768 trigger frame`.
- raw bag: r4 34개 + r5 retry 11개 + r5 retry2 3개, 총 약 `2.3 GiB`.
- 실제 Ego/NPC 속도 범위: `77.96--120.45 / 25.61--79.23 km/h`.
- raw dataset: train/val `471/106 frame`, Vehicle GT `577`.
- Ego deskew dataset: train/val `463/103 frame`, Vehicle GT `566`.
- 두 dataset 모두 info/temporal validator PASS, scene leakage와 unknown asset `0`.
- 원본 audit의 camera age 100 ms 초과는 bag edge 후보 camera별 2개뿐이며 converter가
  선택한 최종 frame의 max는 raw `34.01 ms`, deskew `36.80 ms`다.

구현·재시도 원인·경로·수치는
[`260922_base4_highspeed_jitter_pilot_results.md`](260922_base4_highspeed_jitter_pilot_results.md)에
고정한다.

## 7. 다음 구현 gate

1. `p24v1`은 파이프라인 smoke 전용으로 동결한다. 이 dataset의 val이나 1-epoch 결과로
   모델 성능을 주장하지 않는다.
2. Corrected V2 V1/T1 E4 비교는 완료됐다. 단순 causal 3-sweep T1은 V1보다 recall,
   downstream tracking/prediction과 latency가 나빠 현재 형태로 채택하지 않는다.
3. scene 단위 train/val split만 사용하고 작은 pilot에는 dataset test split을 만들지 않는다.
4. LiDAR timestamp 기준 Ego/Object pose interpolation은 유지한다.
   학습용 Ego trajectory와 scan 내부 deskew는 rosbag의 `/Ego_topic` GT pose를 source of
   truth로 사용한다. GNSS-quantized `/localization/kinematic_state`로 학습 geometry를
   만들지 않는다.
5. 기존 merge/highway1 bag은 반복 사용된 development 평가로만 쓰고, 현장점검 full-stack
   bag은 closed-loop 회귀 근거로만 쓴다. 100 km/h급 최종 비교에는 새 held-out 주행이
   필요하다.
6. 28-scene velocity diagnostic 수집과 합류 재보정은 완료했다. 최종 구성은 보정 합류
   7 scene과 post-merge 순항 21 scene이며, 10--70 km/h 합류 bin을 각각 확보했다.
   3-sweep velocity dataset의 구조/시간/속도 audit와 T1 smoke도 통과했다. 이 작은
   dataset은 mechanism 진단용으로만 유지하고 중단된 `p12v3`는 재개하지 않는다.
7. MORAI 공식 답변상 본선 고주로·로터리 NPC 속도는 샘플과 동일하지 않다. 이에 따라
   `40/50/60 km/h` 고정 prior 대신 연속 속도 jitter를 쓰는 48-scene 고속 파일럿을
   완료했다. 다음 gate는 raw/deskew V1/T1 mechanism A/B와 held-out 고속 bag 평가다.
8. 본선 고주로 RandomMode 사용 시 `MiddleSize`만 사용할 예정이고 사용 여부 자체는
   아직 미정이다. 다음 main quota는 MiddleSize 중심으로 좁히되 비공개 로터리 차종과
   결정 변경에 대비한 작은 multi-model coverage를 유지한다.
9. 학습·평가 GT는 MORAI 권고에 따라 sensor-capture geometric BBox 기준으로 통일한다.
   현재 Base4 converter의 asset별 center/size catalog 보정과 회전 center velocity를
   유지하며, raw `/Object_topic` box 기반 평가는 production 지표로 사용하지 않는다.
10. 다음 production 수집은 현재 속도별 spawn 위상 보정을 시작점으로 삼되, MORAI가
    본선 속도 구성을 비공개로 유지하므로 40/50/60 고정 prior 대신 연속 속도 jitter를
    사용한다. 모델 선택은 별도 seed의 80--120 km/h controller 주행 bag에서 한다.
11. V1/T1 직전 hard-case 보강판은 velocity diagnostic 28 scene, 병목 선별 14 scene,
    일반 고속 횡 offset 11 scene을 입력으로 만들었다. 최종 Vehicle-positive 결과는
    `51 scene / 651 frame`, temporal audit PASS다. 일반 합류와 순항에도 scene-constant
    symmetric 횡 offset을 적용하며 세부 근거와 실행법은
    [`260922_base4_velocity_hardcase_v2_results.md`](260922_base4_velocity_hardcase_v2_results.md)를
    따른다.
12. 고주로 1차 실험 dataset은 hard-case `51`, 고속 연속속도 `48`, 새 균형 보강 `60`
    scene을 합쳤다. 최종 raw는 `159 scene / 2,028 frame`, scan-end Ego deskew twin은
    `157 scene / 1,887 frame`이며 두 temporal audit 모두 PASS다. 세부 분포와 학습 recipe는
    [`260922_base4_highway_first_experiment_dataset.md`](260922_base4_highway_first_experiment_dataset.md)를
    따른다.
