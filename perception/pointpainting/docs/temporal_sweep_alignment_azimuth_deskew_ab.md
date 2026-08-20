# PointPainting temporal sweep alignment and azimuth deskew A/B

## 목적과 추천 순서

1. 기존 동적 1,500장으로 sweep 간 Ego 정합을 먼저 적용한다.
   - 재수집하지 않는다.
   - 과거 sweep을 현재 LiDAR 좌표계로 옮긴다.
   - point마다 현재 시각 대비 `time_lag_sec`를 제공한다.
   - 단일 scan의 7D box보다 객체 이동 단서를 직접 제공하므로 velocity 회귀와
     temporal 입력에 가장 직접적인 실험이다.
2. 동일 데이터에서 scan 내부 방위각으로 point time을 근사한다.
   - raw rolling scan과 azimuth Ego-deskew scan을 같은 split과 seed로 비교한다.
   - geometry, detection, velocity를 모두 개선할 때만 유지한다.
3. 효과가 확인된 다음 수집부터 exact timing sidecar를 추가한다.
   - `/sensor/lidar1` point time.
   - scan 시작·종료·reference timestamp.
   - scan 구간의 Ego pose history.
4. 이후에만 더 복잡한 temporal backbone이나 object-motion 보정을 검토한다.

이번 단계는 1~2번의 데이터·모델 계약과 학습 smoke까지만 수행했다. 본 에포크
학습과 production 변경은 수행하지 않았다.

## 기존 데이터 감사 결과

- SaveSensorData `Intensity.bin`은 모든 표본이 `19,200 point x 4 float32`이다.
- 저장 순서는 `1,200 azimuth x 16 channel`이다.
- 첫 azimuth는 0도이고 이후 약 0.299도씩 clockwise로 진행한다.
- 정확한 point time은 저장되지 않았지만 XYZ 방위각에서 LiDAR 주기 내 상대
  시각을 근사할 수 있다.
- Sensor 6 LiDAR 주기는 `0.06666667초`, extrinsic translation은
  `(0.855, 0, 1.388)m`, yaw는 0도이다.

### MORAI 저속 실행 시간축 문제

개인용 MORAI가 real time보다 느리게 실행되면 ROS header와 callback 시각은 약
0.5~0.6초 진행해도 simulator 안의 운동은 약 0.06~0.2초만 진행한다. 기존
`derived_velocity_mps`와 처음 만든 temporal lag가 이 wall-time을 사용해 실제
속도를 축소하는 문제가 있었다.

다음 방식으로 재수집 없이 simulator time을 복원했다.

- 움직이는 Ego는 snapshot 위치 차이 / Ego raw m/s를 사용한다.
- Ego가 정지하면 stable ID actor의 위치 차이 / ObjectStatus 물리 속도를 사용한다.
- 한 interval에 여러 추정치가 있으면 중앙값을 사용한다.
- 관측 불가능한 interval은 같은 scene의 wall-to-simulator 비율을 사용한다.
- sweep 좌표 정합은 저장된 두 Ego pose를 직접 사용한다.
- scan 내부 Ego deskew만 fixed LiDAR period와 Ego physical velocity를 사용한다.

실제 temporal 대상 80 scene은 모든 interval에서 motion time을 직접 복원했으며
fallback 사용은 0건이다. scene별 simulator/wall time 비율 중앙값은 약 0.203이다.

속도 GT도 같은 simulator time의 actor position finite difference로 다시 만들었다.

| Class | p50 | p90 | p99 | max |
| --- | ---: | ---: | ---: | ---: |
| Vehicle | 1.378 m/s | 11.388 m/s | 16.270 m/s | 16.814 m/s |
| Pedestrian | 0.000 m/s | 1.604 m/s | 2.221 m/s | 2.739 m/s |
| Obstacle | 0.000 m/s | 0.000 m/s | 0.000 m/s | 0.000 m/s |

## temporal dataset 계약

이 실험은 recurrent/attention temporal network가 아니다. 기존 PointPillars에
현재+과거 2개 sweep을 합치고 time-lag feature를 주는 3-sweep early temporal
baseline이다.

- sweep 순서: current, past-1, past-2.
- point feature: 기존 9개 + `time_lag_sec` = 10개.
- box: `x,y,z,dx,dy,dz,yaw,vx,vy` = 9D.
- 과거 sweep XYZ: 저장된 Ego pose로 current LiDAR frame에 변환한다.
- GT sampling: 사용하지 않는다. 단일 sampled box에는 일관된 3-sweep history가
  없기 때문이다.
- scene 단위 기존 split을 유지한다. train/val/test scene 중복은 0이다.

원본 동적 1,500장 중 temporal clip이 실제 있는 mission/hard-dynamic scene만
사용할 수 있다. scene마다 첫 두 frame은 과거 2개 sweep이 없어 target에서 빠진다.
이전 정적 540장은 Ego/Object temporal metadata가 없어 이번 pilot에 넣지 않는다.

| 항목 | 값 |
| --- | ---: |
| 전체 temporal target | 800 |
| Train / Val / Test | 540 / 130 / 130 |
| Mission / Hard dynamic | 620 / 180 |
| Past-1 lag p50 | 0.147초 |
| Past-2 lag p50 | 0.293초 |
| 전체 lag 범위 | -0.481 ~ +0.067초 |
| 합친 point 수 p50 / p95 | 28,002 / 31,557 |

데이터 경로는 다음과 같다.

- Raw temporal:
  `artifacts/datasets/pointpainting_dynamic1500_temporal3_raw_velocity9d_v1`.
- Azimuth Ego deskew:
  `artifacts/datasets/pointpainting_dynamic1500_temporal3_azimuth_ego_deskew_velocity9d_v1`.
- 공통 초기 checkpoint:
  `artifacts/detection3d/pointpainting/releases/260820_dynamic1500_temporal3_velocity9d_init/pointpainting_B0_epoch65_temporal3_velocity9d_init.pth`.

## 체크포인트 이식

- Epoch65 7D geometry head는 anchor별 기존 7개 채널을 그대로 보존한다.
- `vx,vy` 두 채널만 0으로 초기화한다.
- 기존 VFE `[64,15]`에서 9개 raw feature weight를 그대로 보존한다.
- 새 time feature weight를 0으로 삽입한 뒤 기존 cluster/center offset weight를 한 칸
  이동해 VFE `[64,16]`으로 만든다.
- raw와 deskew 모두 `127/127` parameter가 로드됐다.

## sweep 정합과 방위각 deskew 기하 A/B

도로 평면이 metric을 지배하지 않도록 `z >= -1.2m`인 semantic background
structure point의 current/past sweep symmetric nearest-neighbour distance를 800
frame 전체에서 비교했다.

| 지표 | 정합 안 함 | Sweep 정합 raw | Azimuth Ego deskew |
| --- | ---: | ---: | ---: |
| mean | 0.274516 m | **0.150253 m** | 0.150605 m |
| p50 | 0.190239 m | 0.060243 m | **0.060131 m** |
| p90 | 0.486373 m | **0.247464 m** | 0.247564 m |
| p95 | 0.629218 m | **0.475502 m** | 0.478124 m |

Sweep 간 Ego-pose 정합은 p50을 68.3%, p90을 49.1% 줄였다. 따라서 첫 단계는
명확히 유효하다.

방위각 deskew는 raw 정합 대비 frame p50 개선 비율 53.0%였지만 전체 p50 개선은
0.11mm뿐이다. p90은 0.10mm, p95는 2.62mm 악화됐다. 고속 Ego 194 frame도
p50은 약 0.38mm 좋아졌지만 p90/p95는 각각 약 3.38/2.94mm 나빠졌다.

- 방위각/time 방향과 Ego transform은 물리적으로 타당하다.
- sweep 간 Ego 정합은 temporal 입력의 기본값으로 유지할 근거가 충분하다.
- scan 내부 근사 deskew 효과는 너무 작고 tail이 일관되게 좋아지지 않는다.
- raw와 deskew를 실제로 같은 조건에서 짧게 학습한 뒤 detection/velocity A/B를
  해야 채택 여부를 결정할 수 있다.

## smoke 결과

Raw와 deskew 각각에서 단일 GPU batch forward/backward를 통과했다.

- dataset 540 train frame load.
- 10-feature point input load.
- GT tensor `[...,10]`: 9D box + class ID.
- non-zero moving velocity target 존재.
- velocity output channel gradient L1가 0보다 큼.
- prediction과 serializer가 `N x 9` box를 유지함.
- 전체 에포크 학습은 실행하지 않음.

## 사용자 본 학습 A/B

두 실험은 순차 실행한다. seed, 초기 checkpoint, optimizer, epoch를 바꾸지 않는다.
이 pilot은 동적 800 target만 사용하므로 obstacle production 성능을 결정하는 최종
학습이 아니다.

### A. Raw 3-sweep

```bash
cd ~/OpenPCDet/tools
source ~/miniconda3/etc/profile.d/conda.sh
conda activate openpcdet_rtx50
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

python train.py \
  --cfg_file cfgs/custom_models/pointpillar_morai_pointpainting_temporal3_velocity9d.yaml \
  --pretrained_model \
  ~/AjouNice2026/artifacts/detection3d/pointpainting/releases/260820_dynamic1500_temporal3_velocity9d_init/pointpainting_B0_epoch65_temporal3_velocity9d_init.pth \
  --epochs 30 \
  --workers 4 \
  --fix_random_seed \
  --extra_tag temporal3_raw_velocity9d_e30_v1 \
  --set \
  DATA_CONFIG.DATA_PATH \
  /home/luv/AjouNice2026/artifacts/datasets/pointpainting_dynamic1500_temporal3_raw_velocity9d_v1 \
  OPTIMIZATION.LR 0.001
```

### B. Azimuth Ego-deskew 3-sweep

```bash
cd ~/OpenPCDet/tools
source ~/miniconda3/etc/profile.d/conda.sh
conda activate openpcdet_rtx50
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

python train.py \
  --cfg_file cfgs/custom_models/pointpillar_morai_pointpainting_temporal3_velocity9d.yaml \
  --pretrained_model \
  ~/AjouNice2026/artifacts/detection3d/pointpainting/releases/260820_dynamic1500_temporal3_velocity9d_init/pointpainting_B0_epoch65_temporal3_velocity9d_init.pth \
  --epochs 30 \
  --workers 4 \
  --fix_random_seed \
  --extra_tag temporal3_azimuth_ego_deskew_velocity9d_e30_v1 \
  --set \
  DATA_CONFIG.DATA_PATH \
  /home/luv/AjouNice2026/artifacts/datasets/pointpainting_dynamic1500_temporal3_azimuth_ego_deskew_velocity9d_v1 \
  OPTIMIZATION.LR 0.001
```

## 채택 기준

- 주 지표: vehicle/pedestrian center AP와 velocity mAVE/MAE.
- 보호 지표: mATE, mASE, mAOE, IoU AP, recall.
- deskew는 raw 대비 velocity만 좋아지고 geometry AP가 악화하면 채택하지 않는다.
- raw 3-sweep이 single-frame보다 좋아져야 다음 temporal 수집·모델 개발로 간다.
- deskew가 raw보다 여러 seed에서 일관되게 좋아질 때만 다음 수집의 exact timing
  sidecar와 online deskew 계약으로 승격한다.
- 최종 production 학습에는 temporal metadata를 갖춘 obstacle/static clip을 새로
  포함해야 한다.
