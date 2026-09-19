# Base3-Online rosbag dataset — CURRENT STATE

- 상태: **CURRENT**
- 마지막 검증: 2026-09-20
- 기준 branch: `perception/base3-online-rosbag-dataset`
- 최소 구현 commit: `b1a341c9`

## 한 줄 결론

MORAI scenario 생성기의 다양성은 유지하되 production과 같은 UDP 카메라/LiDAR ROS
stream을 rosbag으로 기록하고, LiDAR timestamp에 Ego/Object pose를 보간해 만든 고주로
Vehicle 전용 983-frame BEVFusion dataset이 현재 최종본이다.

학습에는 `morai_base3_online_highway_motion_v2`만 사용한다.
`morai_base3_online_highway_v1`은 nearest-state A/B 근거이며 production 학습 전달본이
아니다.

## 무엇을 왜 바꿨나

Base3-Static은 `/SaveSensorData`가 만든 깨끗한 offline sensor artifact였다. Offline
val은 매우 높았지만 실제 rosbag에서는 특히 Obstacle과 일부 도로 geometry에서 성능이
하락했다. 객체 수만 늘리는 대신 다음 domain 차이를 dataset에 넣기 위해 취득 경로를
바꿨다.

```text
이전: ScenarioLoad -> SaveSensorData files -> offline parser
현재: ScenarioLoad -> 실제 UDP camera/LiDAR -> ROS topics -> rosbag
                    -> marker window -> synchronized parser
```

- Camera/LiDAR: competition-local Sensor JSON과 UDP receiver.
- Ego/Object oracle: MRP의 UDP Ego가 발행되지 않는 문제 때문에 rosbridge.
- 같은 topic의 중복 publisher는 금지.
- `/use_sim_time=false`, sample 기준은 LiDAR header timestamp.
- 한 bag에 여러 scene을 기록하되 `/dataset/scenario_marker`의
  `capture_start`--`capture_end` 구간만 dataset으로 사용.

Ego/Object는 모델 입력이 아니라 GT 좌표 계산용 oracle이므로 rosbridge 사용이 sensor
input domain을 offline으로 되돌리는 것은 아니다.

## 수집 진행과 정리 결과

1. 1/2-scene paired pilot로 topic, marker, 자동 ScenarioLoad 전환을 검증했다.
2. mixed30으로 Vehicle/Pedestrian/Obstacle/empty 변환 계약을 검증했다.
3. 첫 controlled experiment는 고주로 Vehicle만 분리해 334 scene을 수집했다.
4. 잘못 구성한 mixed campaign과 prune 전 중단본은 삭제했다.
5. 완성본 raw bag 7개와 runtime metadata만 최종 source로 유지했다.

최종 raw 결과:

| 항목 | 결과 |
|---|---:|
| scene | 334 / 334 completed |
| runtime failure | 0 |
| raw candidate frame | 1,002 |
| split bag | 7개, 약 24.06 GiB |
| topology | ramp, merge, exit split, 4→3 taper, post-taper |
| Vehicle asset | 43종 |

## 경로 계약

### 원본과 로컬 산출물

```text
# Canonical raw bag
/home/luv/AjouNice2026/rosbags/datasets/base3_online/highway_1k/

# Scenario/runtime metadata
/home/luv/morai_cap/260919_base3_online_highway_vehicle_1k/

# Historical nearest-state A/B dataset — 학습 기본값으로 사용 금지
/home/luv/AjouNice2026/artifacts/datasets/morai_base3_online_highway_v1/

# Canonical motion-aligned dataset
/home/luv/AjouNice2026/artifacts/datasets/morai_base3_online_highway_motion_v2/
```

### 팀원 portable 경로

```text
$HOME/ajounice_bevfusion_artifacts/datasets/base3_online_highway_motion_v2/
```

PKL 내부 absolute prefix는 호스트 경로가 아니라 다음 container root다.

```text
/datasets/base3_online_highway_motion_v2
```

팀 학습 runner는 YAML의 `dataset.capture_root`, `dataset.info_root`,
`dataset.container_root`를 읽어 이 경로로 mount한다. 기존처럼 모든 dataset을
`/datasets/base3`에 강제로 mount하지 않는다.

## 최종 변환 계약

1. 유효 marker window 밖의 load/teleport frame 제거.
2. LiDAR timestamp별 가장 가까운 4-camera frame 선택, camera age gate 적용.
3. Ego/Object pose는 LiDAR 시각 앞뒤 상태로 보간.
4. bracket가 없을 때만 velocity 기반 외삽, 최대 `0.05 s`.
5. `/Object_topic.position`은 geometric center가 아닌 asset pivot이므로 Base3 75개
   asset의 pivot-to-center catalog 적용.
6. GT는 LiDAR frame으로 변환.
7. `num_lidar_pts < 5` GT는 positive supervision에서 제외.
8. production과 같은 `[-30,-30,-3,30,30,2] m` center box-range 적용.
9. train/val/test는 scene 단위 split.

최종 V2:

| 항목 | 값 |
|---|---:|
| train / val / test | 692 / 147 / 144 |
| total | 983 frame |
| Vehicle GT | 1,814 |
| Pedestrian / Obstacle | 0 / 0 |
| state alignment | pose interpolation |
| distance policy | production box-range |

V1의 1,000 frame/Vehicle GT 2,049개는 distance filter 전 nearest-state 비교 결과다.
숫자가 더 많다는 이유로 V1을 최신본으로 선택하지 않는다.

## 정합 검증

- 334 scene 모두 Ego/Object 양쪽 pose bracket 보간 성공.
- nearest 대비 Ego correction 이동량 p50 `2.54 cm`, p95 `4.38 cm`, max `7.01 cm`.
- corrected point-in-box total `117,974 -> 118,704` (`+0.62%`).
- zero-point box `0` 유지.
- MMDetection3D info validator PASS.
- 팀 Docker train/val/test dataloader PASS.
- alignment/audit tests `10/10` PASS.

이 보정은 box 기준시각 정합이다. LiDAR 한 scan 내부 point deskew는 아니다.

## 전달 압축

로컬 업로드 준비 폴더:

```text
/home/luv/AjouNice2026/artifacts/handoff/base3_online_highway_260919/
```

| 파일 | 용도 | 크기 |
|---|---|---:|
| `base3_online_highway_motion_v2_dataset_260919.tar.zst` | 팀 학습 필수 | 409,013,730 B |
| `base3_online_highway_raw_rosbag_260919.tar.zst` | 재파싱·감사 선택 | 24,775,718,506 B |
| `README_FIRST.md` | 압축 해제 위치 | - |
| `SHA256SUMS` | 무결성 검증 | - |

두 archive 모두 생성 후 `sha256sum -c SHA256SUMS`를 통과했다. Base3-Static 10K,
nuScenes 호환 초기 weight, E25 weight, 팀 Docker image는 기존 공유본을 재사용하며 위
묶음에 중복 포함하지 않는다.

```text
b50580ca9d139f1726d7db4df18fdeaf23cb607d684ca70ba11a17540ac5531d  base3_online_highway_motion_v2_dataset_260919.tar.zst
5024dbfb700c4941a0f7a947b882a79b47b64fb24624f5b32f3a328183f0fa92  base3_online_highway_raw_rosbag_260919.tar.zst
```

## 현재 학습 비교

tracked recipe 두 개를 사용한다.

- `online_highway_fresh_e40.yaml`
  - Base3/E25 없이 nuScenes 호환 초기값, 새 optimizer, E40.
- `online_highway_e25_finetune_e12.yaml`
  - E25 model state, 새 optimizer/scheduler, E12.

둘 다 rosbag-derived highway dataset만 train/val/test에 쓴다. 완전 random-init 실험은
아니다. 2080 Ti는 `batch 2 x accumulation 2`, 3090은 `4 x 1`로 effective batch 4를
유지한다.

### E25 fine-tuning 완료 결과

2026-09-19 `online_highway_e25_finetune_e12`를 완료했다. 학습 범위와 같은 고주로
합류·고주로1만 primary로 평가하면 Online val best E7은 Vehicle AP를
`0.6708 -> 0.7187`, center p50을 `0.5993 -> 0.4771 m`, 올바른 2D tracking
AVE를 `0.9685 -> 0.8906 m/s`로 개선했다. 전체 운동 구간 3초 Top-1 FDE는
양쪽 모두 `4.6948 m`이고 CV-stable 3초 minFDE-K mean은
`1.7659 -> 1.7882 m`로 2.2 cm 악화됐다. 과거 motion-state 악화 결론은
body-y twist를 버린 evaluator와 sensor playback timer miss를 수정한 뒤 철회했다.
고주로 specialization 실험은 성공이지만 Obstacle 회귀 때문에 단일 전역
production은 E25를 유지한다.
상세 근거는
[`../../../docs/bevfusion/260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md`](../../../docs/bevfusion/260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md)를 따른다.

### 다음 세대: Base4-Temporal-Highway-Pilot

현재 983-frame V2는 Base3-Online single-frame sensor-domain 실험의 최종본이다. 이를
그대로 전체 Base4라고 이름 바꾸거나, 독립 frame을 가짜 sequence로 묶지 않는다.

다음 수집은 `B4T-HWY-P0`라는 별도 pilot으로 진행한다.

- 실제 LiDAR cadence의 연속 sequence를 수집한다.
- LiDAR 시각 pose interpolation과 단일 `velocity_target_ego_mps=[vx,vy]` 계약을 쓴다.
- 과거 sweep은 Ego motion만 현재 frame에 정합하고 GT Object motion compensation은
  input에 적용하지 않는다.
- train/val은 sequence 단위로 나누며 작은 pilot에는 dataset test split을 만들지 않는다.
- 기존 고주로 합류·고주로1 bag은 반복 사용된 dev 평가이고, 최종 test는 새 held-out
  고주로 bag 최소 2개로 수행한다.
- E25 7D, single-frame 9D velocity, causal 3-sweep 9D를 비교하고 tracker velocity
  off/on까지 분리한다.

전체 계약과 채택 gate는
[`../../../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`](../../../docs/bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md)를 따른다.

Vehicle-only corpus라서 3-class `val/center/mAP`는 Pedestrian/Obstacle 0을 포함한다.
`vehicle_mean_AP`, `vehicle_mATE`로 epoch 후보를 좁히되 production 교체는 기존 7-bag의
Vehicle/Pedestrian/Obstacle과 tracker/Map-CV까지 확인한 뒤 결정한다.

실행은 다음 CURRENT handoff만 따른다.

[`../../../docs/bevfusion/260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md`](../../../docs/bevfusion/260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md)

## 관련 코드

- runtime 검사: `dataset/morai_od/scripts/check_base3_online_runtime.py`
- recorder: `dataset/morai_od/scripts/record_base3_online_paired_bag.sh`
- raw audit: `dataset/morai_od/scripts/audit_base3_online_bag.py`
- GT alignment: `dataset/morai_od/scripts/audit_base3_online_gt_alignment.py`
- converter: `perception/detection3d/bevfusion/tools/build_base3_online_bevfusion_infos.py`
- team runner: `perception/detection3d/bevfusion/tools/run_team_experiment.py`
- recipes: `perception/detection3d/bevfusion/configs/team/recipes/`

세부 명령은 [`base3_online_rosbag_collection_runbook.md`](base3_online_rosbag_collection_runbook.md),
정량 근거는
[`260919_base3_online_highway_vehicle_1k_results.md`](260919_base3_online_highway_vehicle_1k_results.md)를
본다.
