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

python ~/AjouNice2026/perception/pointpainting/tools/train_openpcdet_wandb.py \
  --wandb-entity AjouNice2026 \
  --wandb-project pointpainting-morai \
  --wandb-group temporal3-deskew-ab-e30-v1 \
  --wandb-run-name temporal3-raw-velocity9d-e30-v1 \
  --wandb-tags temporal3,velocity9d,raw \
  --cfg_file cfgs/custom_models/pointpillar_morai_pointpainting_temporal3_velocity9d.yaml \
  --pretrained_model \
  ~/AjouNice2026/artifacts/detection3d/pointpainting/releases/260820_dynamic1500_temporal3_velocity9d_init/pointpainting_B0_epoch65_temporal3_velocity9d_init.pth \
  --epochs 30 \
  --num_epochs_to_eval 30 \
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

python ~/AjouNice2026/perception/pointpainting/tools/train_openpcdet_wandb.py \
  --wandb-entity AjouNice2026 \
  --wandb-project pointpainting-morai \
  --wandb-group temporal3-deskew-ab-e30-v1 \
  --wandb-run-name temporal3-azimuth-deskew-velocity9d-e30-v1 \
  --wandb-tags temporal3,velocity9d,azimuth-deskew \
  --cfg_file cfgs/custom_models/pointpillar_morai_pointpainting_temporal3_velocity9d.yaml \
  --pretrained_model \
  ~/AjouNice2026/artifacts/detection3d/pointpainting/releases/260820_dynamic1500_temporal3_velocity9d_init/pointpainting_B0_epoch65_temporal3_velocity9d_init.pth \
  --epochs 30 \
  --num_epochs_to_eval 30 \
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

## 2026-08-21 학습 결과

- A raw는 epoch 26, B azimuth Ego-deskew는 epoch 14가 각 run의 best였다.
- 동일한 동적 validation 130 frame에서 A raw가 B보다 geometry와 velocity를
  종합적으로 더 안정적으로 유지했다.
- A raw epoch 26의 핵심 결과는 다음과 같다.
  - center mean AP: Pedestrian `0.9807`, Vehicle `0.9745`.
  - BEV IoU AP@0.3/AP@0.5: Pedestrian `0.9851/0.8008`, Vehicle
    `0.9867/0.9867`.
  - mAVE: Pedestrian `0.899 m/s`, Vehicle `1.999 m/s`.
- Geometry 우선 선택에서는 A raw를 먼저 online 이식 검증 대상으로 골랐다.
  그러나 Vehicle velocity가 실제 목표라면 B도 반드시 별도로 평가해야 한다.
  B14는 A26보다 Vehicle mAVE와 speed MAE가 각각 약 `0.256 m/s`, `0.223 m/s`
  낮다. A만 평가한 결과로 B를 기각할 수 없다.
- 다만 validation에는 Pedestrian 40개, Vehicle 81개, Obstacle 0개만 존재한다.
  이 결과만으로 기존 3-class epoch 65 production을 대체할 수 없다.

## Online ROS 이식 및 end-to-end 검증

Online node에 우선 A raw용 다음 pilot 경로를 구현했다.

- 현재 painted sweep과 과거 2개 painted sweep을 보관한다.
- `/Ego_topic` pose를 보간해 과거 sweep 전체를 현재 LiDAR frame으로 정합한다.
- 각 point에 `time_lag_sec`를 추가해 학습과 같은 10-feature 입력을 만든다.
- scan 내부 deskew는 A 조건에 맞게 적용하지 않는다.
- 9D Vehicle box의 `vx,vy`는 표준 `DetectedObjects.has_twist`로 선택적으로
  발행한다.
- EKF velocity fusion은 mode 0과 mode 2를 같은 Detection bag으로 비교한다.

로터리 1 raw bag에서 node startup, 3-sweep history, GPU inference와 469개
Detection 메시지 발행은 모두 정상 동작했다. 그러나 domain-generalization은
실패했다.

| 모델/모드 | Tracking recall | Tracking precision | Center p50 | Speed p50/p90 | 1s minFDE-K p50 | 2s | 3s | 1s coverage |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| E65 single-frame production | 0.931 | 0.993 | **0.733 m** | **0.453/1.313 m/s** | **0.726 m** | **1.136 m** | **1.527 m** | **0.930** |
| A26 temporal, velocity fusion off | 0.477 | 0.727 | 0.797 m | 0.998/2.646 m/s | 1.468 m | 2.499 m | 3.423 m | 0.465 |
| A26 temporal, velocity fusion mode 2, noise 2.0 m/s | 0.248 | 0.445 | 0.732 m | 1.198/3.682 m/s | 1.319 m | 2.291 m | 3.478 m | 0.242 |

- A26은 E65보다 detection 수가 `736 -> 533`, non-empty frame이
  `468 -> 320`, score p50이 `0.686 -> 0.484`로 감소했다.
- velocity fusion은 A26의 noisy velocity 때문에 track propagation과 association을
  더 악화했다. covariance를 조정하는 문제 이전에 detector recall과 velocity
  generalization이 부족하다.
- 동적 validation의 높은 AP는 현재 sample-scene 로터리 bag에 일반화되지 않았다.
  인접 frame이 많은 작은 validation과 제한된 mission family 결과를 production
  성능으로 해석하면 안 된다.
- 위 표는 A26만 평가한 결과이며 B14의 결과가 아니다.
- 현재 raw ROS bag의 PointCloud2에는 point별 `time` field가 없다. B를 공정하게
  평가하려면 학습 때와 동일하게 azimuth에서 point offset을 추정하고 scan 내부
  Ego deskew를 적용하는 online 경로가 먼저 필요하다.
- A26 checkpoint와 online temporal 경로는 연구 pilot으로만 유지한다.
- B14는 아직 production 후보도 기각 후보도 아니며, online azimuth deskew 계약을
  일치시킨 뒤 별도 end-to-end gate를 통과시켜야 한다.
- production은 E65 single-frame, detector velocity 비활성, EKF velocity fusion
  mode 0을 유지한다.
- 다음 velocity 학습은 scene/route/actor profile을 분리한 validation과 실제
  로터리·고주로 replay를 필수 gate로 사용하고, obstacle/static temporal clip까지
  포함해야 한다.

### A26 online 저하 원인 감사

단순히 같은 `roundabout` family를 학습했다는 사실만으로 official sample bag을
본 것으로 간주할 수 없다. 실제 센서 관측 조건을 분해한 결과는 다음과 같다.

- Temporal train 540 target 중 stationary Ego frame은 0개다. Ego raw speed는
  최소 `0.965 m/s`다.
- Train roundabout vehicle의 상대 방위각 중앙값은 약 `86 deg`로 주로 좌측방이다.
- Official roundabout1 bag은 Ego가 전 구간 완전 정지하며, vehicle 상대 방위각
  중앙값은 약 `3 deg`로 정면 통과다.
- Vehicle 속도 중앙값은 train roundabout `5.11 m/s`, bag `5.51 m/s`다.
- Vehicle 거리 중앙값도 train roundabout 약 `20.9 m`, bag 약 `19.4 m`로
  비슷하다. asset도 train에 반복 포함된 K5 계열이다.

Vehicle asset, 속도, 거리보다 stationary-Ego/front-crossing 관측 조건이 더 크게
달랐고, multi-sweep voxel 분포 차이도 관측됐다. 다만 Ego pose 정합이 정확하면
Ego translation 자체는 좌표상 대부분 상쇄된다. 따라서 아래 결과는
stationary Ego가 단독 원인이라는 증명이 아니라 viewpoint, ray sampling, occlusion,
rolling-scan timing과 online temporal construction을 함께 점검해야 한다는 근거다.

| 입력 | 30m 내 3-sweep point p50 | occupied pillar p50 |
| --- | ---: | ---: |
| Temporal train roundabout | 23,619 | 7,966 |
| Official roundabout1 online | 35,743 | 3,803 |

Stationary Ego에서는 같은 beam의 배경 point가 세 sweep에 반복되어 point 수는
51% 많지만 occupied pillar는 약 52% 적다. Moving-Ego train에서는 시점 변화로
complementary surface가 누적된다. 이 density/coverage 차이는 실제지만, 현재
결과만으로 모델 실패의 단독 원인이라고 확정하지 않는다.

실제 거리별 online 검출도 이 해석을 지지한다.

- E65는 nearest vehicle 15m 미만 frame의 100%에서 하나 이상 검출했다.
- A26은 같은 구간에서 40.5%만 검출했다.
- 20--25m에서는 A26도 100% 검출했다.
- ROS score threshold를 `0.10 -> 0.01`로 낮춰도 object 수는 `533 -> 534`,
  non-empty frame은 `320 -> 321`에 그쳤다. score threshold 문제가 아니다.

E65는 1,050 train frame, 80 epoch와 GT sampling을 통해 세 class 및 다양한 상대
배치를 계속 봤다. A26은 E65에서 시작했지만 540 dynamic target만 30 epoch
fine-tune했고 coherent temporal history가 없는 GT sampling은 비활성화했다.
Obstacle GT도 0개다. 제한된 moving-Ego mission 분포에 모든 backbone/head를
계속 update한 결과, 동일 generator validation은 향상됐지만 E65의 광범위한
single-frame robustness를 일부 잃은 specialization으로 해석하는 것이 가장
근거가 강하다.

B deskew는 Vehicle velocity offline 지표가 A보다 낫지만 official roundabout1의
Ego가 정지해 있어 intra-scan Ego deskew 변위 자체는 거의 0이다. B checkpoint의
학습 차이로 결과가 달라질 수는 있으나, 위 stationary/front-view domain gap을
deskew만으로 근본 해결할 수는 없다.

원인을 확정하려면 저장된 동일 3-frame clip에 대해 offline temporal builder 출력과
online temporal assembler 출력을 point/time-lag 단위로 대조해야 한다. 두 입력이
일치하면 generator-to-official-bag domain gap과 specialization이 원인이고, 다르면
online temporal contract 구현 오류가 원인이다.
