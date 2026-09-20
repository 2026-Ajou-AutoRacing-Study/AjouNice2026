# Base3-Online ROS bag 수집·변환 런북

> **문서 상태: CURRENT.** 최신 상태와 canonical 경로는
> [`BASE3_ONLINE_CURRENT_STATE.md`](BASE3_ONLINE_CURRENT_STATE.md)를 먼저 확인한다.
> MR/MRP launcher별 ROS/UDP 설정과 터미널 순서는
> [`morai_mr_mrp_hybrid_scenario_capture_runbook.md`](morai_mr_mrp_hybrid_scenario_capture_runbook.md)를
> 단일 기준으로 사용한다. 이 문서는 Base3-Online campaign의 녹화·audit·변환 명령에
> 집중한다.

마지막 검증: 2026-09-19. 대상 모델은 raw 4-camera + LiDAR BEVFusion이다.
PointPainting용 semantic image는 이 경로에서 만들지 않는다.

## 1. 고정 계약

- MORAI Personal: Real Time Mode.
- Sensor Network: `Sensor_competition_local.json`을 UDP로 Load/Connect.
- Ego Network: `ros.json`을 ROS로 Load/Connect.
- Simulator Network: 연결하지 않는다.
- 센서 publisher: UDP receiver 단 하나.
- `/Ego_topic`, `/Object_topic`: rosbridge 단 하나.
- `/use_sim_time=false`; `/clock`은 기본 기록 대상이 아니다.
- recorder를 scenario runner보다 먼저 시작하고, runner 종료 후 `Ctrl-C`로 recorder를
  종료한다.
- `capture_start`--`capture_end` 밖의 ScenarioLoad/teleport 구간은 dataset index에서
  제외한다.

## 2. 실행 순서

### Terminal 1 — ROS master

```bash
roscore
```

### Terminal 2 — MORAI ROSbridge

```bash
source /opt/ros/noetic/setup.bash
source ~/AjouNice2026/base_ws/install_aw149/setup.bash
source ~/morai_personal_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

이후 MORAI에서 위의 Sensor/Ego Network profile을 각각 Connect한다.

### Terminal 3 — sensor-only UDP gateway

```bash
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
export AJOUNICE_ROOT="$PWD"
rosparam set /use_sim_time false

roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$PWD/configs/sensors/Sensor_competition_local.json" \
  enable_ego_network:=false \
  enable_control:=false
```

### Terminal 4 — publisher와 live message 사전검사

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/check_base3_online_runtime.py \
  --gt-transport ros
```

`PASS`가 아니면 녹화를 시작하지 않는다. 특히 같은 토픽에 UDP와 rosbridge publisher가
동시에 붙은 상태는 무효다.

### Terminal 5 — 연속 rosbag 녹화

```bash
cd ~/AjouNice2026

RUN_TAG="$(date +%y%m%d_%H%M%S)_base3_online_mixed30" \
dataset/morai_od/scripts/record_base3_online_paired_bag.sh \
  "$PWD/rosbags/datasets/base3_online/mixed30"
```

recorder는 LZ4와 4 GiB split을 사용한다. 생성된 `*_0.bag`, `*_1.bag` 등은 한 run의
분할 조각이므로 audit과 변환에 항상 전부, 파일명 순서대로 넘긴다.

### Terminal 6 — 30-scene 파일럿

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
source ~/morai_personal_ws/devel/setup.bash

RUN_DIR="/home/luv/morai_cap/$(date +%y%m%d_%H%M%S)_base3_online_mixed30"

python3 -u dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest dataset/morai_od/manifests/static_classcentric_base3_balanced_pilot30_runtime.json \
  --capture-dir "$RUN_DIR" \
  --no-timestamp-prefix \
  --motion-mode rolling-grouped \
  --online-bag-only \
  --scenario-load-attempts 3 \
  --capture-timeout-sec 30 \
  --continue-on-scene-error
```

runner가 `auto-capture complete`를 출력한 후 Terminal 5를 `Ctrl-C`로 종료한다. 강제 종료
순서를 반대로 하면 마지막 bag index 또는 마지막 `capture_end`가 손상될 수 있다.

## 3. rosbag audit

아래의 `BAGS`에는 같은 run의 split bag을 모두 넣는다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
source ~/morai_personal_ws/devel/setup.bash

BAGS=(rosbags/datasets/base3_online/mixed30/<run>_*.bag)

python3 dataset/morai_od/scripts/audit_base3_online_bag.py \
  "${BAGS[@]}" \
  --json-output artifacts/datasets/base3_online_mixed30_bag_audit.json
```

판정 원칙:

- 필수 토픽 0개, marker 불일치, window 중첩, 큰 timestamp 역행은 실패다.
- 5 ms 이하 rosbridge 도착 순서 jitter는 warning이며 변환 시 timestamp 정렬한다.
- raw 전체 프레임에서 camera age 100 ms 초과 비율이 0.1% 이하면 warning으로 남긴다.
- 실제 선택되는 학습 프레임은 변환기가 100 ms를 엄격히 적용하므로 한 건이라도 넘으면
  변환이 실패한다.

## 4. GT 정합 oracle

`ObjectStatus.position`은 box center가 아니라 asset pivot이다. 반드시 Base3 75개 asset의
`morai_asset_bbox_offsets_base3_v1.json`을 적용한다.

```bash
python3 dataset/morai_od/scripts/audit_base3_online_gt_alignment.py \
  "${BAGS[@]}" \
  --calibration "$RUN_DIR/dynamic_metadata/calibration.json" \
  --asset-offsets dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json \
  --output-dir artifacts/datasets/base3_online_mixed30_alignment_motion_v2 \
  --samples-per-scene 3 \
  --state-alignment interpolate \
  --max-extrapolation-sec 0.05
```

`unknown_assets`는 반드시 빈 배열이어야 한다. 생성된 `*_bev.png`와 4-camera JPG를
Vehicle/Pedestrian/Obstacle에서 각각 눈으로 확인한다.

## 5. BEVFusion dataset 변환·검증

```bash
python3 perception/detection3d/bevfusion/tools/build_base3_online_bevfusion_infos.py \
  "${BAGS[@]}" \
  --calibration "$RUN_DIR/dynamic_metadata/calibration.json" \
  --asset-offsets dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json \
  --output-root artifacts/datasets/morai_base3_online_mixed30_motion_v2 \
  --serialized-root /datasets/base3_online_mixed30_motion_v2 \
  --samples-per-scene 3 \
  --minimum-gt-points 5 \
  --state-alignment interpolate \
  --max-extrapolation-sec 0.05 \
  --distance-filter box-range

python3 perception/detection3d/bevfusion/tools/validate_base3_bevfusion_infos.py \
  --info-dir artifacts/datasets/morai_base3_online_mixed30_motion_v2 \
  --capture-root artifacts/datasets/morai_base3_online_mixed30_motion_v2 \
  --serialized-capture-root /datasets/base3_online_mixed30_motion_v2
```

0-point와 1--4-point GT는 원본 audit에는 남지만 positive supervision에서는 제외된다.
train/val/test는 frame random split이 아니라 manifest의 scene split을 유지한다.

State alignment는 LiDAR timestamp 앞뒤 Ego/Object pose를 보간한다. capture window
경계에서 한쪽 state만 있을 때만 body-frame velocity로 최대 50 ms 외삽하며, 이 제한을
넘으면 변환을 실패시킨다. 이는 GT timestamp 정렬이며 rolling LiDAR point deskew는 아니다.

거리 선택은 raw bag을 수정하지 않고 info 변환 단계에서 적용한다.

| `--distance-filter` | GT 계약 |
|---|---|
| `box-range` | 기본값. 기존 모델과 같은 `[-30,30] x [-30,30] x [-3,2]` center 범위 |
| `radial-0-30` | Ego/LiDAR 원점에서 XY 반경 `[0, 30)` m |
| `radial-30-plus` | XY 반경 `[30, inf)` m |
| `all` | 거리로 GT를 제거하지 않음 |

반경 경계는 `--distance-threshold-m`으로 바꿀 수 있다. 현 production BEVFusion의
BEV grid는 `box-range`이므로 학습용 기본값을 변경하지 않는다. 30 m 밖 전용 info를
만들 수는 있지만, 실제로 학습하려면 그 범위를 포함하는 별도 BEV grid config가 필요하다.

Docker의 실제 dataloader까지 확인한다.

```bash
docker run --rm --gpus all --shm-size=4g \
  -v "$PWD:/workspace/AjouNice2026" \
  -w /workspace/AjouNice2026/perception/detection3d/bevfusion \
  -e PYTHONPATH=/workspace/AjouNice2026:/opt/mmdetection3d \
  ajounice/bevfusion:dev-cu128 bash -lc '
    set -e
    for split in train val test; do
      python tools/smoke_base3_online_dataloader.py \
        --config configs/bevfusion_base3_b0_4cam_lidar.py \
        --ann-file /workspace/AjouNice2026/artifacts/datasets/morai_base3_online_mixed30_v1/morai_base3_infos_${split}.pkl \
        --samples 3
    done
  '
```

## 6. 2026-09-19 mixed30 검증 결과

- 수집: 30/30 scene, 90/90 trigger, 실패 0.
- 구성: stopped Vehicle 12, static Obstacle 9, standing Pedestrian 5,
  sparse/empty background 4.
- bag: 2.73 GB, 258.8 s, 단일 split part.
- capture-window p50: LiDAR 11.81 Hz, camera 23.90 Hz, Ego 42.01 Hz,
  Object 41.89 Hz.
- camera--LiDAR age: p50 2.24--4.81 ms, p95 7.84--10.52 ms.
- raw 1,363 LiDAR frame 중 같은 한 시점만 camera별 100 ms를 근소하게 초과했으며,
  선택된 90 frame의 최대 camera age는 14.83 ms였다.
- 선택 frame 상태 age 최대: Ego 12.29 ms, Object 7.23 ms.
- asset 미지원 0.
- 5-point visibility 적용 전 318 GT, 적용 후 210 GT:
  Vehicle 76, Pedestrian 34, Obstacle 100.
- visibility 유지율은 Vehicle 97.4%, Pedestrian 51.5%, Obstacle 57.5%였다. 본 수집은
  생성 scene 수가 아니라 필터 후 positive 수를 목표로 잡고 Pedestrian/Obstacle을 더
  배정해야 한다.
- output: train 60 / val 15 / test 15 frame.
- info validator PASS, 팀 공용 Docker train/val/test dataloader PASS.

결론: 이 파일럿은 static single-frame BEVFusion용 수집·GT·변환 계약의 기준 run으로
사용할 수 있다. 다음 단계는 바로 대규모 수집이 아니라, 이 방식으로 1,000--2,000개의
독립 frame을 만들고 E25에서 controlled fine-tuning하여 동일 mission rosbag에서
Base3-only와 비교하는 것이다.

## 7. 아직 보류하는 범위

- 동적 actor와 velocity GT.
- LiDAR rolling scan 내부 deskew 및 point-time convention.
- PointPainting semantic mask 생성.
- 실제 대회 bag의 자동 GT 생성(`/Object_topic`이 제공되지 않음).

이 네 항목은 현재 static online fine-tuning baseline과 섞지 않는다.

## 8. 고주로 전용 Online 1K와 portable 경로 계약

첫 확장 실험은 전체 대회 분포를 한 번에 바꾸지 않고 고주로 sensor-domain만 분리한다.

- 334 Vehicle-positive independent scene x 3 m 간격 3 frame = 1,002 frame.
- 이 1차 확장은 고주로 Vehicle domain만 분리해 보는 controlled experiment다.
  Pedestrian, Obstacle, empty/hard-negative는 넣지 않는다.
- 위치: ramp approach, merge lane, exit split, four-to-three taper,
  post-taper two-usable의 다섯 topology를 모두 interleave한다.
- Ego target speed: 30--55 km/h.
- 버스전용/제한 link에는 Ego와 NPC를 배치하지 않는다.
- 첫 비교는 online-only fresh와 production E25-init fine-tuning을 둘 다 수행한다.
  fresh도 완전 random-init이 아니라 팀 표준 nuScenes 호환 초기 가중치를 쓴다.

생성기는 다음과 같다.

```bash
python3 dataset/morai_od/scripts/generate_base3_online_highway_1k.py \
  --clips 334 \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --network-template \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/EgoNetwork/ros_path_probe_MN.json"
```

MGeo 합류·테이퍼의 끊어진 분기 때문에 route-relative `-20 m` rear actor가 실제로
수백 m 떨어진 링크에 배치될 수 있다. 생성 후 실제 Ego 거리 80 m를 넘는 actor를
scenario/metadata/manifest에서 함께 제거한다. 이 조건은 detector 범위 밖의 무의미한
GT와 MORAI spawn 누락을 동시에 막는다.

물리 파일 구조를 Base3 offline과 강제로 섞지 않는다. Docker에서 보이는 canonical
root만 다음처럼 통일한다.

```text
/datasets/
├── base3/                         # 기존 offline portable dataset
└── base3_online_highway_motion_v2/ # 이번 권장 online highway dataset
```

Online 변환 시 `--serialized-root /datasets/base3_online_highway_motion_v2`를 사용한다.
Offline portable PKL은 이미 `/datasets/base3/...`을 가리킨다. 이후 replay 혼합은 센서
파일을 복사하는 방식이 아니라 두 PKL의 `data_list`를 namespace가 충돌하지 않게
결합한다. 팀 Docker는 호스트 dataset root 하나만 `/datasets`에 mount한다.

## 9. Highway Vehicle 1K 최종 결과

Vehicle-positive campaign은 334/334 scene, raw 1,002 frame, runtime failure 0으로
완료했다. LiDAR timestamp pose interpolation, production box-range, 5-point visibility
gate를 적용한 권장 V2는 train/val/test `692/147/144`, 총 983 frame과 Vehicle GT
1,814개다. 세 split 모두 실제 팀 Docker dataloader를 통과했다. nearest-state V1의
1,000 frame/2,049 GT는 정합 A/B 재현용으로만 보존한다.

상세 결과와 경로는
[`260919_base3_online_highway_vehicle_1k_results.md`](260919_base3_online_highway_vehicle_1k_results.md)를
기준으로 한다.

## 10. Highway Static V2 대용량 추가 수집

Base4 temporal을 시작하기 전에 동일한 검증 방식으로 정적 Vehicle scene을 1,000개 더
수집하는 캠페인은
[`260920_base3_online_highway_static_v2_collection_plan.md`](260920_base3_online_highway_static_v2_collection_plan.md)를
따른다. 1,000개를 MRP에 동시에 설치하지 않고 250-scene shard 4개를 순차 설치·보관한다.
이 캠페인은 `train/val/test=800/200/0`이며 기존 334-scene 결과를 덮어쓰지 않는다.
