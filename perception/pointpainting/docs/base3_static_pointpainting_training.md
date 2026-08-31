# Base3-Static PointPainting 학습

## 목적

- Base3-Static `10,902 frame`과 Base3 LRASPP를 사용해 single-frame PointPainting 기준선을 학습함.
- 첫 실험 이름은 `B3S-PP0`임.
- 정적 검출 품질을 먼저 확정하고 velocity와 temporal 구조는 섞지 않음.

## 입력 계약

- split은 clip 단위로 고정하고 학습 표본은 frame 단위로 전개함.
  - Train `7,632 frame`.
  - Val `1,635 frame`.
  - Test `1,635 frame`.
- painting checkpoint는 Base3 LRASPP의 `best_foreground_miou.pth`임.
- 전처리는 online과 같은 OpenCV, short-side `384`임.
- point feature는 다음 9개임.
  - `x, y, z, intensity`.
  - `background, pedestrian, vehicle, obstacle, unknown`.
- 3D box는 위치·크기·yaw만 갖는 7D임.
- 속도 회귀와 sweep stacking은 사용하지 않음.

## GT 품질 정책

- target sweep의 LiDAR point가 5개 미만인 box는 학습·평가 GT에서 제외함.
- MORAI mesh 경계 오차를 감안해 point count에만 `0.03 m` tolerance를 사용함.
- regression box 좌표·크기·yaw는 수정하지 않음.
- class별 유지·제외 개수는 `base3_openpcdet_info_audit.json`에 기록함.

## 학습 설정

- from scratch.
- batch size `4`.
  - 실제 RTX 5060 Ti 16GB end-to-end benchmark에서 BS4가 throughput·메모리·gradient noise의 균형점이었음.
- worker `4`, AMP 사용.
- Adam OneCycle, peak LR `0.003`, weight decay `0.01`.
- `40 epoch`, seed `0`.
- augmentation은 S1g 계열에서 효과가 확인된 기하 증강을 유지함.
  - x/y flip.
  - yaw rotation `±45°`.
  - scale `0.95~1.05`.
- GT database sampling은 첫 기준선에서 사용하지 않음.
- loss는 기존 PointPillars 기준을 유지함.
  - classification `1.0`.
  - localization `2.0`.
  - direction `0.2`.
- 매 epoch 직후 Val을 수행하고 `center_mAP` 최고 checkpoint를 별도로 보존함.
- Test split은 모델 선택에 사용하지 않고 최종 후보 확정 뒤 한 번 평가함.

## 판단 항목

- 주 지표.
  - class별 center AP.
  - mATE, mASE, mAOE.
  - IoU AP와 1 m operating-point precision/recall.
- 보호 지표.
  - obstacle이 높은 개수 때문에 전체 학습을 지배하는지 class별 AP와 positive 분포로 확인함.
  - 로터리·고주로·Pedestrian·Obstacle rosbag에서 offline replay 성능을 확인함.
- `B3S-PP0` 결과가 나온 뒤에만 GT sampling, class weighting, loss 변경을 별도 ablation으로 검토함.

## 학습 전 검증 결과

- 전체 painting 완료.
  - `10,902 frame`, `3.7 GB`.
  - 평균 `69.0 ms/frame`, p95 `79.3 ms/frame`.
  - camera-visible point `88.82%`, unknown `11.18%`.
- sparse-GT audit 완료.
  - Train 유지: Vehicle `5,732`, Pedestrian `4,595`, Obstacle `10,702`.
  - Train 제외: Vehicle `79`, Pedestrian `157`, Obstacle `803`.
- W&B disabled 1-epoch train→val smoke 완료.
  - 총 `3분 40초`.
  - initial loss `9.043`, epoch 평균 loss 약 `2.1` 수준으로 정상 감소함.
  - Val `center_mAP=0.617348`.
  - checkpoint와 `best_center_mAP` checkpoint가 모두 생성됨.
  - 40 epoch 예상 시간은 약 `2시간 27분`, 환경 변동을 포함하면 `2시간 25분~2시간 45분`임.

## B3S-PP0 E39 결과

- 선택 checkpoint는 `best_center_mAP_epoch_39.pth`임.
- frozen Base3 Test center mAP는 현재 canonical evaluator로 재계산한 `0.9744`임.
- 같은 checkpoint를 Base2 Val에 교차 평가한 center mAP는 `0.7052`임.
  - Vehicle `0.9840`, Pedestrian `0.7136`, Obstacle `0.4181`임.
  - Base3 내부 Test 성능만으로 online production 성능을 판단할 수 없음을 다시 확인함.
- rosbag 평가는 `morai-online-center-distance-v1`, 30 m, score `0.10`,
  `bbox-center-calibrated-v1`로 고정함.

### Class별 online OD

| 장면 | class | center mAP | F1@1m | mATE | mASE | mAOE |
|---|---|---:|---:|---:|---:|---:|
| 신로터리 | Vehicle | 0.715 | 0.871 | 0.530 m | 0.110 | 0.611 rad |
| 고주로 합류 | Vehicle | 0.652 | 0.800 | 0.523 m | 0.139 | 0.323 rad |
| 고주로1 | Vehicle | 0.765 | 0.938 | 0.457 m | 0.117 | 0.366 rad |
| Vehicle 30 m | Vehicle | 0.717 | 0.861 | 0.360 m | 0.146 | 0.159 rad |
| 횡단 Pedestrian | Pedestrian | 0.322 | 0.571 | 0.278 m | 0.326 | 1.186 rad |
| 거리별 Pedestrian | Pedestrian | 0.778 | 0.893 | 0.255 m | 0.316 | 1.779 rad |
| CargoBox | Obstacle | 0.407 | 0.622 | 0.543 m | 0.939 | 1.702 rad |

- 고주로1은 S1g보다 center mAP와 mATE가 소폭 낮지만 score `0.10`의 F1은
  `0.882 -> 0.938`, mAOE는 `0.546 -> 0.366 rad`로 개선됨.
- CargoBox는 기존 S1/S1g의 center mAP `0.000`에서 `0.407`로 개선됐으나,
  mASE `0.939`와 IoU AP@0.5 `0.000`이므로 정확한 회피 box로 확정하지 않음.
- 거리별 Pedestrian은 S1g의 `mAP/F1=0.642/0.821`에서
  `0.778/0.893`으로 개선됨.
- 횡단 Pedestrian은 `0.442/0.671 -> 0.322/0.571`로 하락함.
  score `0.10`에서 precision `1.000`, recall `0.399`이므로 false positive보다
  움직임·가림·online painting 조건에서의 누락이 병목임.

## B3S-PP1-GTDB 실험 계약

- 목적은 S1→S1g를 그대로 재현하는 것이 아니라 Base3의 실제 class 분포와 online
  약점을 반영해 single-frame 성능을 최대화하는 것임.
- B3S-PP0와 동일한 Base3 split, Base3 LRASPP painting, 9-feature 7D 구조,
  기하 증강, optimizer와 40 epoch를 유지함.
- GT database sampling 목표는 Vehicle/Pedestrian/Obstacle `20/10/4`임.
  - Vehicle은 로터리·고주로 상대 기하를 가장 강하게 보강함.
  - Pedestrian GT DB 후보 풀은 Batch 008의 걷기·달리기 single-frame pose를
    `60%`, standing pose를 `40%`로 층화함.
  - Obstacle은 자연 train GT가 이미 가장 많으므로 추가량을 4개로 제한함.
- GT DB 최소 LiDAR point는 Vehicle/Pedestrian/Obstacle `5/5/8`임.
- `LIMIT_WHOLE_SCENE=true`라 원본 장면에 해당 class가 이미 많으면 목표 개수까지만
  보충함.
- 자연 Base3 train label 분포는 바꾸지 않으며 GT sampling 후보 풀만 재구성함.
- 2026-08-30 실제 GT DB audit 결과는 다음과 같음.
  - quality filter 이후 Vehicle/Pedestrian/Obstacle 후보는
    `5,732 / 4,586 / 10,029`개임.
  - Pedestrian sampling pool은 moving `603`, standing `402`, 정확히 `60/40`임.
  - moving pool은 walking `265`, running `279`, activation transition `59`임.
  - 384-frame dataloader audit에서 Vehicle 목표 `12/16/20/24`의 실제 평균은
    `4.71/5.30/5.64/5.96`개였음. 큰 box의 collision rejection을 고려해 증가 이득이
    포화되기 직전인 `20`을 선택함.
  - 최종 256-frame audit의 Pedestrian/Obstacle 실제 평균은 `9.21/3.85`개이며,
    9-feature loader와 DB file completeness는 정상임.
- config는
  `pointpillar_morai_pointpainting_base3_static10k_geom_aug_gtdb_v20p10o4.yaml`,
  실험 이름은 `B3S-PP1-GTDB-V20P10O4`로 고정함.

## B3S-PP1-GTDB E34 결과

- 40 epoch 학습은 정상 완료했으며 Val center mAP가 가장 높은 E34를 선택함.
- PP0와 PP1은 Base3 split, Base3 LRASPP, 9-feature 7D head, optimizer와
  기하 증강이 같고 GT database sampling만 다름.

### Frozen Val/Test

| split | 모델 | center mAP | Vehicle AP | Pedestrian AP | Obstacle AP | IoU@0.5 mAP |
|---|---|---:|---:|---:|---:|---:|
| Val | PP0 E39 | 0.9758 | 0.9764 | **0.9850** | 0.9660 | 0.9516 |
| Val | PP1 E34 | 0.9786 | 0.9910 | 0.9773 | 0.9675 | 0.9591 |
| Test | PP0 E39 | 0.9744 | 0.9699 | 0.9810 | 0.9722 | 0.9551 |
| Test | **PP1 E34** | **0.9809** | **0.9843** | 0.9784 | **0.9801** | **0.9640** |

- Test center mAP는 PP0 대비 `+0.0066`임.
- Vehicle AP와 Obstacle AP가 각각 `+0.0144`, `+0.0079` 개선됐고
  Pedestrian AP는 `-0.0027`로 사실상 동률임.
- Val `0.9786`과 Test `0.9809`가 일치하므로 Val 과적합 징후는 없음.

### 동일 7-rosbag online OD 비교

고정 조건은 30 m, inference score floor `0.01`, production F1 score `0.10`,
OpenCV painting, 동일 Base3 LRASPP와 `bbox-center-calibrated-v1`임.

| 장면 | class | PP0 mAP | PP1 mAP | 변화 | PP0/PP1 F1@1m | PP0/PP1 mATE | PP0/PP1 mASE | PP0/PP1 mAOE |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| 신로터리 | Vehicle | 0.715 | **0.749** | +0.034 | 0.871 / **0.892** | 0.530 / **0.480 m** | 0.110 / **0.079** | 0.611 / **0.228 rad** |
| 고주로 합류 | Vehicle | **0.652** | 0.647 | -0.005 | **0.800** / 0.750 | **0.523** / 0.539 m | 0.139 / **0.108** | **0.323** / 0.462 rad |
| 고주로1 | Vehicle | 0.765 | **0.805** | +0.040 | **0.938** / 0.927 | **0.457** / 0.461 m | 0.117 / **0.089** | **0.366** / 0.434 rad |
| Vehicle 30 m | Vehicle | 0.717 | **0.754** | +0.037 | **0.861** / 0.823 | **0.360** / 0.370 m | 0.146 / **0.114** | 0.159 / **0.144 rad** |
| 횡단 | Pedestrian | 0.322 | **0.634** | **+0.311** | 0.571 / **0.760** | **0.278** / 0.308 m | **0.326** / 0.488 | **1.186** / 1.852 rad |
| 거리별 | Pedestrian | 0.778 | **1.000** | **+0.222** | 0.893 / **1.000** | 0.255 / **0.250 m** | **0.316** / 0.412 | **1.779** / 1.944 rad |
| CargoBox | Obstacle | **0.407** | 0.363 | -0.044 | **0.622** / 0.619 | **0.543** / 0.609 m | **0.939** / 0.939 | **1.702** / 1.874 rad |

- GT sampling은 7개 중 5개 장면의 class mAP를 개선함.
- moving-pose Pedestrian pool은 횡단 recall과 거리별 검출을 크게 개선했지만
  box scale/yaw는 개선하지 못함. PointPainting용 semantic class cue에는 유효하지만
  Pedestrian 방향 회귀 개선으로 해석하지 않음.
- Vehicle confidence가 전반적으로 높아져 AP ranking은 좋아졌지만 score `0.10`의
  후보 수도 증가함. 고주로 합류, 고주로1, 30 m의 fixed-threshold F1 하락은
  class별 score calibration이 별도로 필요함을 뜻함.
- CargoBox는 GT sampling으로 개선되지 않았으며 IoU AP@0.5도 계속 0임.
  허용 obstacle 형상에 맞춘 anchor/box supervision 문제로 남김.

### Tracking과 Map-CV

| 장면 | 모델 | Tracking F1 | MOTA | Center p50 | AVE mean | Speed MAE mean/p50 |
|---|---|---:|---:|---:|---:|---:|
| 신로터리 | PP0 | 0.935 | 0.871 | 0.541 m | 1.086 m/s | 0.596 / 0.435 m/s |
| 신로터리 | **PP1** | **0.954** | **0.906** | **0.463 m** | **0.809 m/s** | **0.454 / 0.325 m/s** |
| 고주로1 | PP0 | 0.951 | 0.900 | **0.433 m** | 0.544 m/s | **0.424 / 0.326 m/s** |
| 고주로1 | PP1 | **0.952** | **0.901** | 0.462 m | **0.540 m/s** | 0.429 / 0.333 m/s |

CV-stable minFDE-K의 각 셀은 `p50 / p90`임.

| 장면 | 모델 | 1초 | 2초 | 3초 |
|---|---|---:|---:|---:|
| 신로터리 | PP0 | 0.614 / 1.897 m | 1.172 / 3.283 m | 1.660 / 4.564 m |
| 신로터리 | **PP1** | **0.443 / 1.640 m** | **0.831 / 2.488 m** | **1.243 / 3.402 m** |
| 고주로1 | **PP0** | **0.432 / 1.007 m** | **0.614 / 1.669 m** | **0.819 / 2.221 m** |
| 고주로1 | PP1 | 0.454 / 1.089 m | 0.620 / 1.846 m | 0.885 / 2.546 m |

### 판정

- PP1은 PP0를 대체하는 **Base3 계열 보존 checkpoint**로 채택함.
  - frozen Test와 7-bag 평균 방향, 로터리 Tracking/Prediction, Pedestrian recall이
    명확히 좋아졌기 때문임.
- 전체 production은 아직 S1g score `0.40`을 유지함.
  - PP1 score `0.10`은 고주로 합류 F1과 고주로 Map-CV가 PP0보다 나쁘고,
    CargoBox도 개선하지 못함.
  - PP1의 다음 실험은 재학습보다 class별 score calibration을 우선함.
- 산출물은 다음임.
  - `artifacts/evaluation/pointpainting/base3_b3s_pp1_gtdb_e34_rosbag_260830`.
  - `artifacts/prediction_evaluation/base3_b3s_pp1_gtdb_e34`.

### S1g E37 직접 7-bag 재평가

- 과거 문서값을 섞지 않고 S1g E37을 PP1과 같은 7개 bag, 같은 현재 evaluator,
  score floor `0.01`과 production operating point `0.10`으로 다시 추론함.
- target-class 7-bag macro center mAP는 S1g `0.5574`, PP1 `0.7073`임.
- PP1은 7개 장면 모두에서 S1g 이상의 center mAP를 기록함.
  - 신로터리 `0.720 -> 0.749`.
  - 고주로 합류 `0.572 -> 0.647`.
  - 고주로1 `0.803 -> 0.805`.
  - Vehicle 30 m `0.722 -> 0.754`.
  - 횡단 Pedestrian `0.442 -> 0.634`.
  - 거리별 Pedestrian `0.642 -> 1.000`.
  - CargoBox `0.000 -> 0.363`.
- downstream은 각 모델의 현재 production 후보 threshold를 사용해 S1g score `0.40`,
  PP1 score `0.10`을 비교함.
  - PP1은 로터리 Tracking center p50이 S1g와 동률이고, AVE mean은
    `0.953 -> 0.809 m/s`로 개선함.
  - S1g는 로터리 Tracking F1/MOTA와 1--3초 prediction p90에서 우세함.
  - 고주로 PP1은 prediction p50이 우세하지만 S1g는 center p50과 prediction p90이
    우세함.
- 따라서 detector representation은 PP1이 S1g보다 강하지만, 전체 production
  pipeline은 PP1 class별 calibration 전까지 S1g가 더 안전함.
- S1g 직접 재평가 산출물은
  `artifacts/evaluation/pointpainting/base2_s1g_e37_rosbag7_260830`임.

### Vehicle downstream

| 장면 | Tracking F1 | MOTA | Center p50 | AVE mean | Speed p50 |
|---|---:|---:|---:|---:|---:|
| 신로터리 | 0.935 | 0.871 | 0.541 m | 1.086 m/s | 0.435 m/s |
| 고주로1 | 0.951 | 0.900 | 0.433 m | 0.544 m/s | 0.326 m/s |

CV-stable 구간의 minFDE-K는 다음과 같음. 각 셀은 `p50 / p90`임.

| 장면 | 1초 | 2초 | 3초 |
|---|---:|---:|---:|
| 신로터리 | 0.614 / 1.897 m | 1.172 / 3.283 m | 1.660 / 4.564 m |
| 고주로1 | 0.432 / 1.007 m | 0.614 / 1.669 m | 0.819 / 2.221 m |

### 원인 판정

1. 고주로는 실패가 아니라 개선 지표가 달라진 결과임.
   - S1g의 GT sampling은 confidence ranking과 반복적인 Vehicle box 회귀에 유리했음.
   - Base3는 실제 도로 기하와 hard negative를 유지해 fixed-threshold F1, yaw,
     Tracking 안정성이 좋아졌지만 center mAP·mATE는 S1g를 넘지 못함.
   - 정적 SaveSensorData가 움직이는 고주로 차량의 sparse point, 센서 비동기,
     rolling-scan object motion 분포를 직접 재현하지는 못함.
2. 로터리 OD는 급락하지 않았고 downstream velocity가 주 병목임.
   - S1g 대비 center mAP는 `0.720 -> 0.715`, Tracking center p50은
     `0.531 -> 0.541 m`로 소폭 하락함.
   - 하지만 Tracking speed p50은 `0.318 -> 0.435 m/s`, vector AVE는
     `1.086 m/s`임.
   - 곡선에서는 작은 위치·yaw 흔들림이 velocity 방향 오차와 Lanelet branch 오차로
     바뀌어 2~3초 FDE가 크게 누적됨.
3. Pedestrian 외형 학습 전체가 실패한 것은 아님.
   - 거리별 bag은 개선됐고 횡단 bag만 하락했음.
   - 움직이는 팔·다리, sparse point, 가림 뒤 등장, camera--LiDAR 시각 차이가
     겹치는 횡단 online domain이 Base3 정적 중심 분포보다 어려움.

### 다음 원인 분리 실험

1. Base3 로터리 detection에 GT velocity를 주입해 현재 velocity-free tracker와
   Prediction을 비교함.
2. 횡단 Pedestrian에서 frozen LRASPP painting과 oracle painting을 frame slice로
   비교해 segmentation·동기화 병목과 3D detector 병목을 분리함.
3. S1g와 Base3를 거리, 방위각, LiDAR point 수, 객체 속도로 나눠 recall과 box
   geometry 차이를 확인함.
4. 위 결과 전에는 B3S-PP0를 전체 production으로 승격하지 않고 S1g 기반 production을
   유지함. Base3 checkpoint는 class-centric spatial pretraining 후보로 보존함.

평가 산출물:

- `artifacts/evaluation/pointpainting/base3_b3s_pp0_e39_rosbag_260830`.
- `artifacts/prediction_evaluation/base3_b3s_pp0_e39`.

## 실행 명령

```bash
systemd-inhibit \
  --what=sleep:idle \
  --who=Base3-PointPainting \
  --why="Base3-Static PointPainting 40 epoch training" \
  --mode=block \
  bash -lc '
set -eo pipefail

cd /home/luv/AjouNice2026
source /home/luv/miniconda3/etc/profile.d/conda.sh
conda activate openpcdet_rtx50

export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
export WANDB_PROJECT=morai-3d-object-detection
export WANDB_MODE=online

python perception/pointpainting/tools/train_openpcdet_wandb.py \
  --wandb-run-name pointpainting-base3-static-b3s-pp0-s1geom-e40 \
  --wandb-group Base3-Static-PointPainting \
  --wandb-tags pointpainting,base3-static,single-frame,9feat,s1-geom,from-scratch \
  --wandb-mode online \
  --openpcdet-root /home/luv/OpenPCDet \
  --cfg_file cfgs/custom_models/pointpillar_morai_pointpainting_base3_static10k_geom_aug.yaml \
  --batch_size 4 \
  --epochs 40 \
  --workers 4 \
  --extra_tag base3_static_b3s_pp0_s1geom_e40 \
  --fix_random_seed \
  --validation-mode interleaved \
  --validation-interval 1 \
  --validation-start-epoch 1 \
  --selection-metric center_mAP \
  --ckpt_save_interval 1 \
  --max_ckpt_save_num 10 \
  --num_epochs_to_eval 0 \
  --wo_gpu_stat \
  --use_amp \
  --set \
    DATA_CONFIG.DATA_PATH /home/luv/AjouNice2026/artifacts/datasets/morai_pointpainting_base3_static10k_9feat_opencv_v1 \
    OPTIMIZATION.LR 0.003
'
```
