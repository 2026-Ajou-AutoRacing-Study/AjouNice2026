# Base3-Online Highway Vehicle 1K 수집 결과

## 결론

2026-09-19에 고주로 Vehicle-positive sensor-domain specialization dataset을
완성했다.

- Raw capture: `334 scene x 3 frame = 1,002 frame`.
- Runtime scene failure: `0/334`.
- 학습 positive gate: Vehicle GT가 5 LiDAR point 이상 남는 frame만 유지.
- 최초 nearest-state BEVFusion dataset: `1,000 frame`, Vehicle GT `2,049`개.
- 권장 motion-aligned + production box-range dataset: `983 frame`, Vehicle GT
  `1,814`개.
- Pedestrian/Obstacle GT: `0`; empty/hard-negative campaign scene도 `0`.
- V1 train/val/test: `702 / 150 / 148 frame`.
- 권장 V2 train/val/test: `692 / 147 / 144 frame`.
- 팀 공용 BEVFusion Docker의 train/val/test dataloader 모두 PASS.

이 dataset은 Base3 전체 분포를 대체하지 않는다. Production checkpoint E25에
고주로 Vehicle online domain을 짧게 controlled fine-tuning하는 실험용 add-on이다.

## 산출물

### Raw rosbag

```text
/home/luv/AjouNice2026/rosbags/datasets/base3_online/highway_1k/
└── 260919_base3_online_highway_vehicle_1k_base3_online_paired_{0..6}.bag
```

- 총 크기: 약 `24.06 GiB` (`du` 표시는 약 `25G`).
- 4 GiB split part 6개와 종료 직후 57 MiB part 1개다.
- scene marker로 334개 capture window가 모두 복원된다.

### Runtime metadata

```text
/home/luv/morai_cap/260919_base3_online_highway_vehicle_1k/
```

- `summary.json`: requested/attempted/completed `334/334/334`, failed `0`,
  completed captures `1,002`, `finished=true`, `interrupted=false`.
- `dynamic_metadata/`: scene별 frame/timing/calibration 계약.

### BEVFusion portable dataset

```text
/home/luv/AjouNice2026/artifacts/datasets/morai_base3_online_highway_v1/
├── morai_base3_infos_train.pkl
├── morai_base3_infos_val.pkl
├── morai_base3_infos_test.pkl
├── conversion_audit.json
├── validation_audit.json
└── samples/
```

PKL 내부 경로는 호스트 절대경로가 아니라 다음 portable root를 사용한다.

```text
/datasets/base3_online_highway_v1
```

Docker에서는 위 artifact directory를 같은 경로로 mount한다.

위 경로는 비교 재현용 nearest-state v1이다. 현재 권장본은 다음과 같다.

```text
/home/luv/AjouNice2026/artifacts/datasets/morai_base3_online_highway_motion_v2/
```

PKL 내부 portable root는 `/datasets/base3_online_highway_motion_v2`다.

## 시나리오 분포

전체 334 scene이 `stopped_vehicle` family다.

| 고주로 topology | scene |
|---|---:|
| highway_exit_split | 80 |
| highway_merge_lane | 74 |
| highway_post_taper_two_usable | 67 |
| highway_ramp_approach | 60 |
| highway_four_to_three_taper | 53 |

- Vehicle asset: 43종.
- runtime Vehicle actor 수/scene: 1대 73 scene, 2대 171 scene,
  3대 86 scene, 4대 4 scene.
- capture view 합계: front 1,364, rear 533, left 92, right 78.
- train/val/test scene split: `234 / 50 / 50`.

MGeo merge/taper의 끊어진 topology 때문에 route-relative rear actor가 실제로
수백 m 떨어지는 사례 14개를 생성 후 제거했다. scenario, metadata, manifest의 actor
count를 함께 수정하며, 실제 Ego 거리 80 m 이내 actor만 runtime에 남긴다.

## rosbag 및 동기화 감사

Raw audit는 PASS했다. 전체 후보 frame 중 100 ms camera age 초과는 각 카메라
13--16개, 최대 0.103%뿐이어서 raw audit에서는 0.11%까지 warning으로 허용했다.
실제 학습 sample 선택은 별도의 strict 100 ms gate를 모두 통과했다.

선택된 1,002 frame의 nearest-state/camera age:

| 입력 | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|
| Ego | 6.87 ms | 13.62 ms | 19.12 ms | 34.79 ms |
| Object | 6.56 ms | 13.97 ms | 19.12 ms | 33.79 ms |
| Front camera | 7.59 ms | 20.61 ms | 27.48 ms | 37.50 ms |
| Front-left camera | 8.99 ms | 19.38 ms | 28.42 ms | 35.78 ms |
| Front-right camera | 8.83 ms | 19.83 ms | 28.21 ms | 34.66 ms |
| Back camera | 7.74 ms | 20.27 ms | 26.01 ms | 36.72 ms |

rosbridge header stamp의 1--2 ms 역전은 Object 9건, Ego 4건이었다. 변환기는
window 내부 상태를 header stamp로 정렬한 뒤 nearest match하므로 학습 sample에는
영향이 없다.

## GT 정합과 visibility

334 scene에서 한 frame씩 oracle alignment를 수행했다.

- 확인 box: 689개.
- unknown asset: 0.
- MORAI asset pivot을 그대로 box center로 쓸 때 zero-point box: 41개.
- asset별 pivot-to-center 보정 후 zero-point box: 0개.
- point-in-box median: `25 -> 37`.
- point-in-box total: `77,030 -> 117,974`.

이후 LiDAR timestamp 앞뒤 Ego/Object pose interpolation을 적용했다. 334개 audit
frame의 모든 Ego state와 Object state가 양쪽 표본으로 보간됐으며 velocity 외삽이나
spawn/despawn fallback은 한 건도 없었다.

- nearest + pivot 보정 point-in-box total: `117,974`.
- pose interpolation + pivot 보정 point-in-box total: `118,704`.
- 증가: `+730 point`, 약 `+0.62%`.
- median: 양쪽 모두 `37`; zero-point box: 양쪽 모두 `0`.
- nearest pose 대비 Ego 보정 이동량: p50 `2.54 cm`, p95 `4.38 cm`, 최대 `7.01 cm`.

따라서 interpolation은 zero-point 회귀 없이 실제 LiDAR 정합을 개선했다. 이 보정은
GT box를 LiDAR message timestamp에 맞추는 것이며 rolling scan point deskew는 아니다.

학습 변환에서는 minimum GT points를 5로 뒀다.

- 유지 Vehicle GT: 2,049개.
- 1--4 point라 supervision에서 제외된 Vehicle GT: 18개.
- 그 결과 Vehicle positive가 하나도 남지 않은 frame: test 2개.
- raw 1,002 frame은 보존하고, 최종 Vehicle-only PKL에서는 위 2개를 제외해
  정확히 1,000 positive frame을 사용한다.

V1 PKL에는 empty frame이 없다. Vehicle distance 분포는 전체적으로 약
3.35--54.61 m이며 median은 약 22 m다.

V2 변환은 기존 production 모델과 동일한 `[-30,30] x [-30,30] x [-3,2]`
center box-range를 기본으로 명시 적용했다.

- distance로 제외된 Vehicle GT: `252`개.
- sparse GT 제외: `1`개.
- 거리/visibility 적용 후 Vehicle-positive가 사라진 frame: `19`개.
- 최종: train/val/test `692 / 147 / 144`, 합계 `983 frame`.
- 최종 Vehicle GT: `1,814`개.

반경 30 m 필터와 기존 box-range는 같지 않다. V2 변환기는 `radial-0-30`,
`radial-30-plus`, `all`도 지원하지만 현 모델 학습에는 `box-range`를 유지한다.

## 검증 위치

```text
artifacts/datasets/base3_online_highway_vehicle_1k_scenario_audit/audit.json
artifacts/datasets/base3_online_highway_vehicle_1k_bag_audit.json
artifacts/datasets/base3_online_highway_vehicle_1k_alignment_v1/gt_alignment_audit.json
artifacts/datasets/base3_online_highway_vehicle_1k_alignment_motion_v2/gt_alignment_audit.json
artifacts/datasets/morai_base3_online_highway_v1/conversion_audit.json
artifacts/datasets/morai_base3_online_highway_v1/validation_audit.json
artifacts/datasets/morai_base3_online_highway_motion_v2/conversion_audit.json
artifacts/datasets/morai_base3_online_highway_motion_v2/validation_audit.json
```

Alignment directory에는 334개 BEV와 4-camera projection, 총 1,670개 이미지가 있다.

## 재현 계약

- 생성기:
  `dataset/morai_od/scripts/generate_base3_online_highway_1k.py`
- Runtime 검사:
  `dataset/morai_od/scripts/check_base3_online_runtime.py`
- 녹화기:
  `dataset/morai_od/scripts/record_base3_online_paired_bag.sh`
- raw audit:
  `dataset/morai_od/scripts/audit_base3_online_bag.py`
- GT oracle audit:
  `dataset/morai_od/scripts/audit_base3_online_gt_alignment.py`
- BEVFusion 변환:
  `perception/detection3d/bevfusion/tools/build_base3_online_bevfusion_infos.py`
- info validator:
  `perception/detection3d/bevfusion/tools/validate_base3_bevfusion_infos.py`
- Docker smoke:
  `perception/detection3d/bevfusion/tools/smoke_base3_online_dataloader.py`

최종 변환에는 반드시 다음을 사용한다.

```bash
--samples-per-scene 3 \
--minimum-gt-points 5 \
--require-positive-class vehicle \
--state-alignment interpolate \
--max-extrapolation-sec 0.05 \
--distance-filter box-range \
--serialized-root /datasets/base3_online_highway_motion_v2
```

## 삭제한 중단본

처음 잘못 해석해 수집한 Vehicle/Obstacle/empty 혼합본과 prune 전 짧은 본은
최종 dataset 및 학습 PKL에 포함되지 않는 것을 확인한 뒤 2026-09-19에 삭제했다.

- `highway_1k_aborted_mixed_campaign`: raw rosbag 약 18 GiB와 runtime metadata.
- `highway_1k_aborted_preprune`: raw rosbag 약 1.7 GiB와 runtime metadata.

완성된 `highway_1k` Vehicle-positive 캠페인과 비교용 `mixed30` pilot은 보존했다.
