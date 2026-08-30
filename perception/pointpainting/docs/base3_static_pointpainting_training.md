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
- frozen Base3 Test center mAP는 `0.9799`임.
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
