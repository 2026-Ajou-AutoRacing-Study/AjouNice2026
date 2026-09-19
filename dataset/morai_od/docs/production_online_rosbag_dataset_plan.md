# Production-Online ROS Bag Dataset 계획

> **문서 상태: PLAN + DECISION HISTORY.** 설계 배경과 단계별 결정 기록이다. 완료된
> 결과와 현재 경로는
> [`BASE3_ONLINE_CURRENT_STATE.md`](BASE3_ONLINE_CURRENT_STATE.md)가 우선한다.

마지막 갱신: 2026-09-19.

## 목적

Base3 시나리오 생성기의 공간·객체 다양성은 유지하면서 센서 취득 경로를
`SaveSensorData`에서 실제 production과 같은 UDP/ROS online 경로로 바꾼다.
Base3 offline val은 높지만 mission rosbag에서 성능이 하락하는 원인을 단순 객체 수가
아닌 입력 domain 차이까지 포함해 줄이는 것이 목적이다.

가칭은 `Base3-Online`이다. 기존 문서의 3-sweep velocity용 `Base4-Temporal`과 같은
이름을 쓰지 않는다. Pilot 결과 후 최종 dataset 이름을 확정한다.

## 핵심 방식

1. `rosbag record`를 실행한 상태에서 기존 시나리오 생성기·주입기를 순차 실행한다.
2. MORAI가 실제로 발행하는 압축 카메라, LiDAR, Ego, Object 토픽을 저장한다.
3. `SaveSensorData` artifact는 학습 입력으로 사용하지 않는다.
4. 각 sample은 LiDAR header timestamp를 기준으로 production synchronizer와 동일한
   카메라 선택 정책을 적용해 만든다.
5. Ego/Object 상태는 LiDAR 시각으로 보간해 LiDAR 좌표계 3D GT를 만든다.

### 전송 경로 계약

- 모델 입력인 카메라와 LiDAR, 보조 센서인 GPS/IMU는 MORAI UDP 설정에서 전송하고
  `morai_udp_interface.launch`가 ROS 토픽으로 변환한다.
- 현재 MRP에서는 competition-local UDP profile의 `/Ego_topic`이 실제 패킷을 보내지
  않는 현상이 재현됐다. 따라서 Ego/Object GT와 ScenarioLoad/control은 ROSbridge를
  사용한다. Ego/Object는 모델 입력이 아니라 좌표 변환·정답 생성용 oracle이므로 이
  선택은 모델의 online 센서 입력 domain을 바꾸지 않는다.
- UDP gateway는 `enable_ego_network:=false enable_control:=false`로 실행한다.
- rosbridge Ego Network에는 기존 `ros.json`을 사용한다. 별도 hybrid JSON을 만들지
  않는다. Simulator Network와 `/ego_setting`은 본 수집에 사용하지 않는다.
- 같은 센서/Ego/Object 토픽을 UDP receiver와 rosbridge가 동시에 발행하면 해당 run은
  무효다. 녹화 전 publisher contract 검사를 통과해야 한다.
- 본 수집은 MORAI Real Time Mode와 `/use_sim_time=false`를 기준으로 한다. `/clock`은
  필수 입력이 아니며 원인 분석이 필요할 때만 선택적으로 기록한다.
- 매 scene은 기존 ROS-only EgoNetwork sidecar를 포함한 full ScenarioLoad로 로드한다.
  2026-09-19의 2-scene pilot에서 두 번째 scene의 Ego Network가 약 1.9초 안에 자동
  복구됐고 수동 재연결 없이 완료됐다.

이 방식으로 다음 online 특성을 학습 데이터에 포함한다.

- JPEG 압축 영상과 production 색상·resize 전처리.
- 카메라--LiDAR timestamp 차이.
- UDP 수신 jitter와 실제 ROS callback 순서.
- Ego와 동적 객체의 연속 motion.
- production과 동일한 sensor configuration과 calibration.

## 기록 토픽 계약

최소 기록 대상은 다음과 같다.

```text
/sensor/lidar1
/sensor/camera/front/compressed
/sensor/camera/front_left/compressed
/sensor/camera/front_right/compressed
/sensor/camera/back/compressed
/Ego_topic
/Object_topic
/tf
/tf_static
/dataset/scenario_marker
```

필요하면 `/clock`, `/ReplayInfo_topic_out`, simulator process state를 진단용으로
추가하되 학습 sample의 기준 시각은 LiDAR header다.

## Scene marker 계약

하나의 bag에 여러 scene을 기록하므로 시나리오 전환 프레임을 자동 제외할 수 있어야
한다. 자동 수집기는 `/dataset/scenario_marker`에 최소 다음 event를 발행한다.

```json
{"event":"scene_loading","scene_id":"...","family":"..."}
{"event":"capture_start","scene_id":"...","family":"..."}
{"event":"capture_end","scene_id":"...","family":"..."}
```

`capture_start`는 ScenarioLoad 완료만으로 발행하지 않는다. Ego pose, expected object,
LiDAR, 카메라 네 개가 모두 안정화되고 최소 settling window를 통과한 뒤 발행한다.
`capture_end` 이후 다음 `scene_loading`까지의 데이터는 학습에서 제외한다.

## 자동 수집기 online 모드

`--online-bag-only`는 외부 rosbag recorder가 실제 센서 stream을 기록한다는 계약이다.
거리별 trigger에서는 Ego/Object/timing metadata만 남기고 `/SaveSensorData`를 호출하거나
MORAI 파일 artifact를 검증하지 않는다. 현재는 `--motion-mode rolling-grouped`와 함께
사용한다.

online mode는 다음 상태 순서를 가진다.

```text
scene load
  -> expected Ego/Object 및 센서 안정화
  -> capture_start marker
  -> rolling Ego 또는 동적 객체 운행
  -> 유효 관찰 window
  -> capture_end marker
  -> next scene
```

ROS 연결 일시 불안정은 생성기 검증 로직을 임의로 강화해 숨기지 않는다. 기존 운영
원칙대로 MORAI--rosbridge 재연결을 먼저 요청하고 동일 scene을 재시도한다.

## Dataset builder 계약

1. marker의 유효 window 밖 프레임을 제거한다.
2. LiDAR timestamp를 기준으로 각 카메라 ring buffer에서 production과 동일한 정책으로
   가장 가까운 이미지를 선택한다.
3. 허용 camera age를 넘긴 sample은 제외하고 원인을 집계한다.
4. `/Ego_topic`과 `/Object_topic` history를 LiDAR 시각에 보간한다.
5. scenario manifest의 asset, class, 크기와 Object 상태를 결합해 3D GT를 만든다.
6. 기존 LiDAR visibility 정책을 유지한다.
   - 0-point GT 제거.
   - 1--4-point sparse GT는 positive supervision에서 제외.
   - 5-point 이상을 정상 positive로 사용.
7. frame별 원시 Object 위치 미분값을 학습 GT처럼 노출하지 않는다. 필요할 때만
   진단 namespace에 기록한다.

## 중복과 용량 제어

30 Hz 카메라를 scene 전체에서 모두 독립 single-frame sample로 쓰지 않는다.

- 정적 scene은 scene당 대표 LiDAR frame 3--8장만 선택한다.
- 동적 Vehicle/Pedestrian은 motion phase를 보존하되 시간적으로 지나치게 가까운
  frame을 제거한다.
- train/val/test는 frame random split이 아니라 scene/run 단위로 분리한다.
- bag은 batch 또는 약 4 GiB 단위로 LZ4 split한다.
- 긴 ScenarioLoad 대기 구간과 전환 구간은 bag에 남아도 dataset index에서 제외한다.

같은 rosbag을 반복 재생해 sample 수만 늘리는 것은 데이터 다양성 증가로 인정하지
않는다. 매 scene의 Ego/object/asset/geometry randomization이 실제로 달라야 한다.

## 실행 단계

### Phase -1. online pilot — 완료

2026-09-19에 full ScenarioLoad + `--online-bag-only`로 1-scene 및 2-scene 연속
pilot을 완료했다. 2-scene bag은 두 capture window 모두 audit PASS였으며 수동
network reconnect 없이 scene 전환까지 완료했다.

- clean 2-scene bag:
  `rosbags/datasets/base3_online/paired_pilot/260919_121227_base3_online_paired_0.bag`
- audit:
  `/home/luv/morai_cap/base3_online_bag_only_pilot2/online_bag_audit.json`
- Camera--LiDAR age: p50 1.57--5.05 ms, p95 3.19--8.57 ms, max 74.04 ms.
- capture-window rate: LiDAR 11.64/12.83 Hz, camera 24.06/26.18 Hz,
  Ego/Object 약 39.5--41.1 Hz.
- 100 ms 초과 0건, capture window 내부 timestamp 역행 0건.

### Phase 0. 현재 baseline 고정

- BEVFusion E1--E40 corrected-range30 validation과 best checkpoint 선정을 완료한다.
- 기존 8-bag production 기준을 고정한다.

### Phase 1. 30--50 scene pilot

- Vehicle, Pedestrian, Obstacle, empty/hard-negative를 포함한다.
- 정적 scene과 실제로 움직이는 Pedestrian/Vehicle scene을 모두 넣는다.
- rosbag topic rate, timestamp, marker, frame coverage, disk rate를 감사한다.
- 임의 sample을 RViz와 카메라 projection으로 확인한다.

### Phase 2. 변환기 검증

- production synchronizer가 선택한 카메라와 dataset builder 선택 결과가 같은지
  sample 단위로 비교한다.
- GT box를 raw LiDAR와 RViz에 중첩해 위치, yaw, 크기를 확인한다.
- 시나리오 전환·teleport 프레임이 index에 0개인지 확인한다.
- 동일 scene 연속 frame 과대표집 통계를 낸다.

#### 2026-09-19 2-scene 변환기 oracle 검증 결과

PointPainting·semantic mask 경로는 후속 범위에서 제외하고 BEVFusion의 raw 4-camera +
LiDAR 입력 계약만 검증했다.

`/Object_topic.position`은 3D BBox geometric center가 아니라 asset pivot이다. 따라서
단순 `morai_map -> ego -> lidar` 변환만 하면 GT가 체계적으로 틀어진다. 과거 Base3의
MORAI `LIDAR_5/*_Instance.txt`와 같은 asset의 ObjectStatus를 결합해 75개 최종 Base3
asset 전부에 대해 asset-local pivot-to-center offset을 복원했다.

- catalog:
  `dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json`
- calibration observations: `31,644`
- asset 수: `75` (`43 Vehicle + 22 Pedestrian + 10 Obstacle` 전부 포함)
- center residual: p50 `0.126 m`, p95 `0.259 m`
- builder:
  `dataset/morai_od/scripts/build_morai_asset_bbox_offset_catalog.py`

이 offset을 clean 2-scene online bag의 LiDAR 시각 ObjectStatus에 적용한 결과는 다음과
같다.

- sampled frames: `6`, GT boxes: `15`, 미지원 asset: `0`
- naive pivot box: point-in-box 합계 `1,823`, median `28`, zero-point box `1`
- corrected center box: 합계 `1,938`, median `47`, zero-point box `0`
- LiDAR BEV 및 4-camera projection을 수동 확인했으며 Vehicle 윤곽과 box가 정합했다.
- oracle output:
  `artifacts/datasets/base3_online_paired_pilot_alignment_v1/`
- audit:
  `dataset/morai_od/scripts/audit_base3_online_gt_alignment.py`

동일 6 frame을 MMDetection3D v2 info로 변환한 뒤 기존 BEVFusion validator와 실제
Docker 학습 dataloader를 통과했다.

- dataset output:
  `artifacts/datasets/morai_base3_online_paired_pilot_v1/`
- output: 4-camera tensor `4 x 3 x 256 x 704`, LiDAR `Nx4`, GT 15개
- converter:
  `perception/detection3d/bevfusion/tools/build_base3_online_bevfusion_infos.py`
- dataloader smoke:
  `perception/detection3d/bevfusion/tools/smoke_base3_online_dataloader.py`

따라서 static single-frame BEVFusion용 좌표·calibration·파일 계약은 통과했다. 동적
actor는 LiDAR rolling scan의 기준시각과 point-time convention을 별도로 확정한 뒤
추가한다.

#### 2026-09-19 30-scene mixed pilot 결과

- 30/30 scene, 90/90 capture trigger 완료, 실패 0.
- Vehicle 12 / Obstacle 9 / standing Pedestrian 5 / sparse background 4 scene.
- bag 2.73 GB, capture window 30개, marker 및 필수 topic 계약 PASS.
- capture-window rate p50: LiDAR 11.81 Hz, camera 23.90 Hz,
  Ego 42.01 Hz, Object 41.89 Hz.
- 선택한 90 frame의 camera age 최대 14.83 ms, Ego/Object age 최대
  12.29/7.23 ms.
- asset 미지원 0, 5-point visibility 적용 후 GT 210개
  (Vehicle 76 / Pedestrian 34 / Obstacle 100).
- scene split train 60 / val 15 / test 15 frame.
- 기존 validator와 팀 공용 BEVFusion Docker dataloader가 세 split 모두 PASS.

재현 가능한 전체 실행·audit·변환 명령은
`dataset/morai_od/docs/base3_online_rosbag_collection_runbook.md`를 따른다.

### Phase 3. 작은 controlled fine-tuning

- 약 1,000--2,000 independent frame으로 시작한다.
- Base3 corrected best를 초기값으로 짧게 fine-tuning한다.
- Base3-only와 Base3+Online을 동일 offline val 및 동일 8개 mission bag에서 비교한다.
- GT sampling, 해상도 변경, temporal modeling을 첫 실험에 동시에 섞지 않는다.

### Phase 4. 확대 여부 결정

다음 조건을 만족할 때만 본 수집으로 확대한다.

- online bag에서 class별 AP/recall 또는 false positive가 반복적으로 개선됨.
- Base3 frozen val과 box geometry가 허용 범위 이상 퇴화하지 않음.
- runtime latency와 input contract가 production과 동일함.
- 개선이 특정 bag 하나의 반복 frame에만 의존하지 않음.

## 성공 판단

offline val 상승만으로 성공 처리하지 않는다. 같은 threshold와 evaluator를 사용한
mission rosbag에서 다음을 본다.

- Vehicle/Pedestrian/Obstacle AP, precision, recall, F1.
- mATE, mASE, mAOE와 거리·방향 slice.
- 정적 배경 hard-negative false positive.
- tracker F1/MOTA/AVE와 Map-CV prediction FDE.
- inference p50/p95 latency, output Hz, drop/skip, sensor-stamp age.

## Online batch 실행 순서

MORAI Personal은 Real Time Mode로 실행한다.

- Sensor Network: `Sensor_competition_local.json`을 UDP로 Load/Connect.
- Ego Network: 기존 `ros.json`을 ROS로 Load/Connect.
- Simulator Network: 연결하지 않음.

```bash
# Terminal 1
roscore

# Terminal 2
source /opt/ros/noetic/setup.bash
source ~/AjouNice2026/base_ws/install_aw149/setup.bash
source ~/morai_personal_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch

# Terminal 3: sensor-only UDP -> ROS gateway
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
export AJOUNICE_ROOT="$PWD"
rosparam set /use_sim_time false
roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$PWD/configs/sensors/Sensor_competition_local.json" \
  enable_ego_network:=false \
  enable_control:=false

# Terminal 4: bag recorder. 먼저 실행해야 첫 marker가 보존된다.
cd ~/AjouNice2026
dataset/morai_od/scripts/record_base3_online_paired_bag.sh

# Terminal 5: full ScenarioLoad + online marker/metadata
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
source ~/morai_personal_ws/devel/setup.bash
python3 -u dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest dataset/morai_od/manifests/static_classcentric_base3_balanced_pilot30_runtime.json \
  --scenario b3stopocorr30pilot_dsv001 \
  --capture-dir /home/luv/morai_cap/base3_online_paired_smoke \
  --no-timestamp-prefix \
  --motion-mode rolling-grouped \
  --online-bag-only \
  --scenario-load-attempts 3 \
  --capture-timeout-sec 30
```

Terminal 5가 끝난 뒤 Terminal 4를 `Ctrl-C`로 종료한다. 생성된 split bag 전부를 다음처럼
검사한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
source ~/morai_personal_ws/devel/setup.bash
python3 dataset/morai_od/scripts/audit_base3_online_bag.py \
  rosbags/datasets/base3_online/paired_pilot/<이번 실행 bag>.bag \
  --json-output /tmp/base3_online_paired_audit.json
```

## 보류 사항

- 실제 대회 bag에는 `/Object_topic` GT가 없으므로 자동 정답 dataset으로 바로 쓰지
  않는다. 수동 라벨, hard-negative audit, 제한적 pseudo-label 용도로 분리한다.
- ReplayInfo-only bag을 MORAI에서 복원하는 방식은 `/Ego_topic` 복원 및 loop boundary
  문제가 있으므로 본 수집의 기본 경로로 확정하지 않는다.
- LiDAR scan 내부 Ego/Object motion distortion는 timestamp 보간만으로 완전히 해결되지
  않는다. 필요하면 별도 deskew/rolling-scan 연구로 분리한다.
