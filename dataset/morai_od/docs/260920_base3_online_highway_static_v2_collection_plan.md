# Base3-Online Highway Static V2 수집 계획

- 상태: **PREPARED — raw rosbag 미취득**
- 생성일: 2026-09-20
- 목적: Base4 temporal 이전에 검증된 Base3-Online 방식으로 고주로 정적 Vehicle
  sensor-domain 데이터를 충분히 확보한다.

## 실험 계약

- 동적 NPC와 velocity head 실험이 아니다.
- 정지 Vehicle scene만 사용한다.
- Ego는 고주로를 `30--55 km/h` 목표속도로 주행한다.
- scene당 3 m 간격의 독립 single-frame target 3개를 얻는다.
- 입력은 production과 같은 UDP 4-camera + LiDAR ROS topic이다.
- Ego/Object는 GT 좌표 계산용 rosbridge oracle이다.
- 내부 dataset test split은 만들지 않는다. scene 단위 `train 800 / val 200`이다.
- 최종 일반화 판정은 별도 고주로 rosbag으로 한다.

Base4의 12--24 연속 LiDAR frame, velocity GT, causal sweep은 이 캠페인에 섞지 않는다.

## 생성 결과

| 항목 | 값 |
|---|---:|
| scene | 1,000 |
| raw target frame | 3,000 |
| train / val / test scene | 800 / 200 / 0 |
| Vehicle asset | 43종 |
| runtime actor / scene | 1--4, median 2 |
| Ego target speed | 30.03--55.00 km/h |
| non-local actor prune | 32 |

고주로 topology는 각 split에서 정확히 80/20으로 유지한다.

| topology | 전체 | train | val |
|---|---:|---:|---:|
| ramp approach | 180 | 144 | 36 |
| merge lane | 220 | 176 | 44 |
| exit split | 240 | 192 | 48 |
| four-to-three taper | 160 | 128 | 32 |
| post-taper two usable | 200 | 160 | 40 |

전체 manifest와 250-scene shard 4개 모두 정적 validator `errors=0`,
`warnings=0`을 통과했다. Base3 placement audit도 `PASS`다.

## 파일

```text
dataset/morai_od/manifests/base3_online_highway_static_v2.json
dataset/morai_od/manifests/base3_online_highway_static_v2_batch001.json
...
dataset/morai_od/manifests/base3_online_highway_static_v2_batch004.json
```

생성 scenario와 metadata는 dataset `.gitignore` 대상이며 로컬에서 재생성한다.

```bash
cd ~/AjouNice2026
python3 dataset/morai_od/scripts/generate_base3_online_highway_1k.py \
  --clips 1000 \
  --campaign-tag highwaystaticv2 \
  --seed 260920300 \
  --group base3_online_highway_static_v2 \
  --dataset-id morai_static_classcentric_base3_10k_online_highway_static_v2 \
  --scenario-prefix b3ohs2 \
  --portable-dataset-root /datasets/base3_online_highway_static_v2 \
  --train-ratio 0.8 --val-ratio 0.2 --test-ratio 0 \
  --stratify-splits-by-spatial-stratum \
  --shard-size 250
```

## 수집 순서

ROS master, rosbridge, competition-local sensor UDP gateway가 연결된 뒤 runtime check를
먼저 통과시킨다. 상세 연결은
[`base3_online_rosbag_collection_runbook.md`](base3_online_rosbag_collection_runbook.md)를
따른다.

### Terminal A — 연속 split rosbag

```bash
cd ~/AjouNice2026
RUN_TAG="$(date +%y%m%d_%H%M%S)_base3_online_highway_static_v2" \
dataset/morai_od/scripts/record_base3_online_paired_bag.sh \
  "$PWD/rosbags/datasets/base3_online/highway_static_v2"
```

### Terminal B — 4개 shard 자동 수집

```bash
cd ~/AjouNice2026
SESSION_ROOT="$HOME/morai_cap/$(date +%y%m%d_%H%M%S)_base3_online_highway_static_v2"

dataset/morai_od/scripts/run_base3_online_highway_static_v2_capture.sh \
  --session-root "$SESSION_ROOT"
```

runner는 한 번에 250개만 MRP에 설치하고 shard 종료 후
`~/morai_scenario_archive/base3_online_highway_static_v2/`로 이동한다. MRP scenario
폴더에는 이전 생성물이 1,000개 이상 남아 있으므로, live smoke 전에 기존 생성물 archive
정리가 필요하다. 사람 손으로 만든 sample scenario는 삭제하지 않는다.

runner 종료 후 Terminal A의 recorder를 `Ctrl-C`로 닫는다. 334-scene 실측치를 선형
환산하면 약 `2.5--3시간`, raw bag `75--85 GiB`가 예상된다. 4 GiB split bag은 run 종료
전까지 삭제하지 않는다.

## 완료 gate

1. 1,000/1,000 scene 완료, runtime failure 0 또는 원인 확인 후 refill.
2. target marker 3,000개와 scene window 일치.
3. camera/LiDAR 필수 topic 0 구간 없음.
4. motion-aligned 변환에서 unknown asset 0.
5. `num_lidar_pts >= 5`, production box-range 적용 후 최종 frame/GT 통계 기록.
6. scene 단위 train/val 80/20 유지.

## 완료 결과

2026-09-20 수집·변환·결합 검증을 완료했다.

| 항목 | 결과 |
|---|---:|
| scene | 1,000 / 1,000 completed |
| capture | 3,000 / 3,000 |
| runtime failure | 0 |
| raw split bag | 18개, 약 75 GiB |
| motion-aligned train / val / test | 2,327 / 580 / 0 |
| retained frame | 2,907 |
| retained Vehicle GT | 5,435 |
| unknown asset | 0 |

93개 frame은 production box-range와 5-point gate를 통과한 Vehicle이 없어
Vehicle-positive deck에서 제외됐다. 전체 bag audit에서는 308,125개 Object state 중
1개가 7 ms 역행해 기존 5 ms warning 문턱을 2 ms 초과했다. 모든 scene window가
완료됐고 변환기가 timestamp 정렬과 선택 frame 정합을 재검증했으며, 최종 2,907 frame
info validator가 통과했으므로 rosbridge delivery jitter로 판정했다.

```text
# raw
rosbags/datasets/base3_online/highway_static_v2/

# parsed sensor deck
artifacts/datasets/morai_base3_online_highway_static_v2_motion_v2/
```

기존 Highway Motion V2와 센서 파일 복사 없이 결합한 학습 deck은 다음과 같다.

| split | 기존 V2 | Static V2 | 결합 |
|---|---:|---:|---:|
| train | 692 | 2,327 | 3,019 |
| val | 147 | 580 | 727 |
| test | 144 | 0 | 144 |
| total | 983 | 2,907 | 3,890 |

결합 Vehicle GT는 7,249개이며 duplicate sample ID는 없다. 기존 held-out test 144장은
train/val에 섞지 않았다.

```text
artifacts/datasets/morai_base3_online_highway_combined_v3/
perception/detection3d/bevfusion/configs/team/recipes/online_highway_combined_fresh_e40.yaml
```

결합 info validator와 팀 Docker 16-sample train/val smoke가 모두 통과했다. 이 recipe의
`fresh`는 완전 random init이 아니라 nuScenes-compatible 초기 weight에서 새 optimizer로
E40 학습한다는 뜻이다.
