# Base2 Temporal3 Velocity D4 Readiness

## 결론.

- Base2 10K의 D3 audit 결과를 실제 OpenPCDet 3-sweep 학습 데이터로 변환함.
- PointPainting 입력은 10 feature임.
  - `[x, y, z, intensity, background, pedestrian, vehicle, obstacle, unknown, time_lag_sec]`.
- Box head는 기존 7D geometry에 `vx, vy`를 추가한 9D를 예측함.
- GT는 9D box 뒤에 `velocity_loss_weight`를 추가한 10D numeric contract임.
- Geometry loss는 모든 유효 box에 유지함.
- Velocity loss만 label quality에 따라 가중함.
- 16 GB GPU에서 batch size 4 forward/backward가 통과함.
- 전체 학습은 아직 시작하지 않았음.

## 입력 데이터와 split.

- 원본 Base2 dataset.
  - `/home/luv/morai_cap/morai_temporal_sequence_v2_base2_10504_final`.
- D3 temporal index.
  - `artifacts/datasets/morai_temporal_base2_10k_d3`.
- D4 OpenPCDet dataset.
  - `artifacts/datasets/morai_pointpainting_base2_temporal3_velocity_weighted`.

| Split | D4 sample | 정책 |
|---|---:|---|
| Train | 5,509 | 학습에 사용함. |
| Val | 1,181 | epoch 평가와 모델 선택에 사용함. |
| Test | 1,182 | 최종 모델 선택 전까지 materialize하지 않고 봉인함. |

- D3에서 timing-invalid인 6 sample은 D4에서 제외됨.
- Train/Val sample overlap은 0개임.
- 고유 painted sweep은 8,924개이며, 같은 source frame은 한 번만 segmentation한 뒤 재사용함.
- D4 결과 용량은 약 11 GB임.

## Temporal 입력 계약.

- 하나의 target sample은 현재 sweep과 과거 2 sweep으로 구성함.
- 과거 sweep 좌표는 D3의 `lidar_to_target_lidar`로 현재 Ego/LiDAR 좌표계에 정합함.
- 각 sweep의 모든 point에는 실제 sweep timestamp 차이인 상수 `time_lag_sec`를 추가함.
  - 예: 약 `[-0.176, -0.080, 0.000]`초.
- scan 내부 point time은 MORAI SaveSensorData에 없으므로 방위각 기반 근사 deskew를 적용하지 않음.
- 첫 baseline은 sweep 간 Ego-motion alignment만 사용함.

## Velocity label 계약.

| Source | Box 수 | 처리 |
|---|---:|---|
| `static_zero` | 13,127 | `vx=0`, `vy=0`, weight 1.0을 사용함. |
| `morai_raw_local_rotated` | 4,483 | MORAI raw world velocity를 target LiDAR 좌표계로 회전하고 D3 quality weight를 사용함. |
| invalid | 4 | Geometry는 학습하고 velocity weight만 0으로 둠. |

- 전체 velocity weight 평균은 `0.980377`임.
- Class별 box 수는 Obstacle 12,062, Vehicle 4,118, Pedestrian 1,434개임.
- label quality가 낮아도 center/size/yaw supervision을 버리지 않음.

## Weighted velocity head.

- Head 이름은 `AnchorHeadSingleVelocityWeighted`임.
- 일반 OpenPCDet anchor assignment에는 9D box와 class만 전달함.
- positive anchor와 원본 GT를 center/class 기준으로 다시 연결해 velocity quality를 부착함.
- regression channel `0:7`은 기존 geometry loss를 그대로 사용함.
- regression channel `7:9`만 quality weight를 곱함.
- W&B/TensorBoard에 다음 항목을 별도 기록함.
  - `rpn_loss_geometry`.
  - `rpn_loss_velocity`.
  - `velocity_positive_weight_mean`.

## Augmentation 결정.

- 첫 T0 baseline은 geometry augmentation을 사용하지 않음.
- multi-sweep GT sampling은 서로 다른 시간의 정합된 point/box 관계를 깨므로 비활성화함.
- flip은 velocity와 좌표계 handedness를 함께 정확히 바꿔야 하므로 첫 baseline에서는 비활성화함.
- 첫 실험 목적은 augmentation 효과가 아니라 temporal velocity head 자체의 효과를 분리하는 것임.
- T0가 정상 학습된 뒤 shared rotation/scale만 별도 ablation하는 것이 다음 순서임.

## Online ROS 계약.

- `morai_pointpainting_ros`에 `temporal_time_lag_mode`를 추가함.
- 기본값은 `sweep_constant`임.
  - 현재 sweep은 0초임.
  - 과거 sweep은 실제 source-target LiDAR timestamp 차이를 사용함.
  - Ego pose history로 과거 sweep 좌표를 현재 좌표계에 정합함.
- `azimuth_approx`는 연구용 비교 모드로만 남김.
- `temporal_num_sweeps=1`인 기존 production은 동작이 변하지 않음.

## Audit 및 smoke 결과.

- D4 readiness audit은 `pass`임.
  - 256 point file 표본의 shape와 finite 값을 확인함.
  - 모든 표본에서 현재 0초를 포함한 3개의 distinct sweep lag를 확인함.
  - point count p50은 28,375개임.
- E65 checkpoint migration smoke.
  - 기존 7D box head를 9D로 확장함.
  - VFE에 열 번째 time-lag feature weight를 0으로 삽입함.
  - 127/127 parameter가 로드됨.
  - 이는 구조 호환성 검증용이며 주 학습 초기값으로 채택한 것은 아님.
- Positive velocity-weight GPU smoke.
  - velocity gradient L1은 `2.6037`로 non-zero임.
- Zero velocity-weight inverse smoke.
  - velocity gradient는 정확히 `0.0`임.
  - geometry gradient는 `32.862`로 유지됨.
- 전체 Train dataset single-batch smoke.
  - loss `6.3501`.
  - velocity gradient `2.0471`.
  - geometry gradient `31.2595`.
- Batch size 4 FP32 forward/backward smoke.
  - 16 GB GPU에서 OOM 없이 통과함.
  - GT tensor shape은 `(4, 6, 11)`임.
  - velocity gradient `0.6348`, geometry gradient `30.8336`임.
- W&B offline 1-epoch end-to-end smoke가 통과함.
  - run: `artifacts/wandb/wandb/offline-run-20260824_034014-j3z09w31`.
  - train/eval/MORAI metric logging을 확인함.
  - 4-sample 중복 pilot의 AP는 성능 지표로 해석하지 않음.
- 첫 전체 Train/Val 1-epoch smoke 이후 표준 MORAI validation에도 velocity metric을 추가함.
  - 2m center 안에서 score-ranked 1:1 matching함.
  - GT quality로 가중한 `mAVE`, speed MAE, 종방향 MAE, 횡방향 MAE를 class별 기록함.
  - 이후 epoch validation과 W&B에서 geometry와 velocity 성능을 함께 확인할 수 있음.

## Model-neutral W&B canonical schema.

- Fast-BEV와 PointPainting이 같은 W&B panel을 재사용하도록 validation key를 통일함.
- PointPainting의 기존 `val/morai/*` key는 원본 evaluator 진단용으로 유지함.
- 공통 panel은 다음 canonical namespace만 사용함.
  - `val/center_mAP`, `val/center/*`: center-distance AP.
  - `val/tp/*`: mATE, mASE, mAOE.
  - `val/operating/*`: 1 m best F1, precision, recall, score.
  - `val/range/*`: 거리 구간별 center AP.
  - `val/iou/*`: BEV IoU AP와 recall.
  - `val/velocity/*`: mAVE, speed/longitudinal/lateral MAE.
- TensorBoard 자동 동기화가 만드는 `eval/eval_with_train/*` 중복 namespace는
  W&B 자동 panel 생성에서 숨김 처리함. 기록 자체는 삭제하지 않음.
- 모든 `val/*` scalar의 x축은 명시적인 `epoch`으로 정의함.
- AP/F1/recall은 W&B summary `max`, error 계열은 summary `min`으로 정의함.

## Fast-BEV-compatible epoch train/val loop.

- PointPainting도 Fast-BEV와 같은 epoch 단위 실행 계약을 사용함.
  - `Train epoch N`.
  - `checkpoint_epoch_N.pth` 저장.
  - 같은 model state로 `Val epoch N` 즉시 실행.
  - canonical `val/*`을 W&B의 `epoch=N`에 즉시 기록.
  - `center_mAP` 기준 best checkpoint 갱신.
  - 이후 `Train epoch N+1`로 진행.
- 기본 validation mode는 `interleaved`, interval은 1 epoch임.
- OpenPCDet의 기존 학습 종료 후 checkpoint 재평가는 interleaved mode에서
  비활성화해 같은 epoch을 두 번 평가하지 않음.
- epoch별 일반 checkpoint는 최근 10개만 유지해도 됨.
  - best checkpoint는 `best_center_mAP_epoch_N.pth`로 별도 보존함.
- 2-epoch 순서 smoke가 통과함.
  - 실제 로그 순서는 `Train 1 -> Val 1 -> Train 2 -> Val 2`임.
  - `checkpoint_epoch_1.pth`, `checkpoint_epoch_2.pth`, best checkpoint가 생성됨.
  - 4-sample pilot의 AP=0은 실행 순서 검증용이며 성능으로 해석하지 않음.

### Canonical schema 1-epoch smoke.

- run name: `pointpainting-base2-t0-canonical-schema-smoke-e1`.
- output:
  `/home/luv/OpenPCDet/output/custom_models/pointpillar_morai_pointpainting_base2_temporal3_velocity_weighted/base2_t0_canonical_schema_smoke_e1`.
- W&B offline run:
  `artifacts/wandb/wandb/offline-run-20260824_045324-00gvq234`.
- Train 5,509 frame와 Val 1,181 frame 전체를 사용해 성공함.
- 학습 시간은 약 3분 41초, validation 추론은 약 24초임.
- canonical scalar 299개가 TensorBoard에 기록됨.
- 주요 1-epoch 결과는 다음과 같음.
  - `val/center_mAP=0.480851`.
  - `val/iou/mAP_0.25=0.604030`.
  - `val/iou/mAP_0.50=0.420965`.
  - `val/velocity/vehicle_mAVE=4.140074 m/s`.
  - `val/velocity/pedestrian_mAVE=1.217069 m/s`.
- 이는 로깅·학습 파이프라인 smoke 결과이며 최종 성능으로 해석하지 않음.
- 1 epoch OneCycle schedule이 끝난 checkpoint이므로 40 epoch 본 학습에 resume하지 않음.

## 주 학습 초기화 결정.

- 주 T0 실험은 from scratch로 시작함.
- E65는 single-sweep 9-feature 모델이라 temporal 10-feature velocity model의 정답 초기값이 아님.
- E65 migration checkpoint는 구조 smoke와 후속 initialization ablation에만 사용함.
- LRASPP 4-class segmentation checkpoint는 기존 학습된 production weight를 사용함.
  - segmentation은 D4 materialization 시 이미 적용되어 있으므로 PointPillars 학습 중에는 다시 실행하지 않음.

## 승인 후 실행할 전체 학습 명령.

```bash
cd ~/AjouNice2026
source ~/miniconda3/etc/profile.d/conda.sh
conda activate openpcdet_rtx50

export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
export WANDB_PROJECT=morai-3d-object-detection
export WANDB_MODE=online

python perception/pointpainting/tools/train_openpcdet_wandb.py \
  --wandb-run-name pointpainting-base2-t0-temporal3-velocity-weighted-s0 \
  --wandb-group Base2-D4 \
  --wandb-tags pointpainting,base2,temporal3,velocity,from-scratch \
  --wandb-mode online \
  --openpcdet-root /home/luv/OpenPCDet \
  --cfg_file cfgs/custom_models/pointpillar_morai_pointpainting_base2_temporal3_velocity_weighted.yaml \
  --batch_size 4 \
  --epochs 40 \
  --workers 4 \
  --extra_tag base2_t0_temporal3_velocity_weighted_s0 \
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
  --set DATA_CONFIG.DATA_PATH /home/luv/AjouNice2026/artifacts/datasets/morai_pointpainting_base2_temporal3_velocity_weighted
```

- `--pretrained_model`을 의도적으로 넣지 않음.
- W&B project는 `morai-3d-object-detection`임.
- 매 epoch validation이 끝나는 즉시 기존 공통 W&B panel이 갱신됨.
- online W&B는 실제 사용자 터미널에서 실행함.
- 본 문서 작성 시점에는 전체 학습을 실행하지 않았음.

## T0 이후 판단 순서.

1. Train/Val velocity loss가 감소하고 velocity error가 실제로 개선되는지 확인함.
2. Center AP, BEV IoU AP, mATE가 E65 대비 크게 손상되지 않는지 확인함.
3. Vehicle velocity를 거리·속도 bin별로 평가함.
4. Rosbag에서 tracking velocity error와 Map-CV 1/2/3초 FDE를 평가함.
5. 성공하면 shared rotation/scale augmentation ablation을 수행함.
6. 필요할 때만 E65 initialization ablation을 수행함.
7. 최종 후보를 정한 뒤 봉인된 Test split을 한 번 평가함.

## T0 40-epoch 완료 결과와 online 판정.

- Base2 validation에서는 temporal velocity 학습 자체가 성립함.
  - Center mAP best는 epoch 25의 `0.928269`임.
  - Vehicle mAVE best는 epoch 33의 `0.771284 m/s`임.
  - Detection과 velocity의 실용 Pareto 후보는 epoch 35임.
    - Center mAP `0.916078`.
    - BEV IoU AP@0.50 `0.876759`.
    - Vehicle mATE `0.164562 m`.
    - Vehicle mAVE `0.775446 m/s`.
- 그러나 epoch 35를 실제 동적 rosbag에 적용하면 E65보다 detection geometry와
  recall이 크게 저하됨.
  - 새 로터리 Vehicle center mAP: E65 `0.681`, T0 `0.428`.
  - 고주로 Vehicle center mAP: E65 `0.653`, T0 `0.431`.
  - 새 로터리 detector mAVE 진단값: `1.794 m/s`.
  - 고주로 detector mAVE 진단값: `2.921 m/s`.
- Detector velocity를 EKF measurement로 직접 융합하면 두 장면 모두 tracker의
  위치 차분 기반 속도보다 Map-CV FDE가 악화됨.
- 판정은 다음과 같음.
  - T0를 production으로 채택하지 않음.
  - E65와 tracker 위치 차분 속도를 production으로 유지함.
  - 봉인 Test split은 실패한 T0에 소비하지 않음.
  - velocity loss 가중치 ablation보다 Base2 SaveSensorData validation과 실제
    asynchronous rosbag 사이의 geometry/domain gap 진단을 우선함.

### T0 checkpoint online 1-sweep 진단.

- Epoch 35의 구조와 가중치를 그대로 두고 online 입력만 3-sweep에서 현재
  1-sweep으로 바꿔 새 로터리와 고주로를 다시 평가함.
- 10-feature 계약은 유지하고 현재 sweep의 `time_lag_sec=0`을 명시함.
- 새 로터리는 혼합 결과임.
  - Vehicle center mAP는 `0.428 -> 0.312`로 악화됨.
  - CV minFDE-K p50은 1/2/3초 `1.009/1.589/2.283 m`에서
    `0.624/1.417/1.964 m`로 개선됨.
  - p90과 coverage는 E65보다 여전히 크게 나쁨.
- 고주로는 1-sweep이 일관되게 개선됨.
  - Vehicle center mAP `0.431 -> 0.496`.
  - Tracking F1 `0.642 -> 0.796`.
  - CV minFDE-K p50 1/2/3초 `0.987/1.508/2.037 m`에서
    `0.641/0.949/1.583 m`로 개선됨.
- 결론은 다음과 같음.
  - online 3-sweep stacking이 특히 고주로 성능 악화의 주요 원인임.
  - 로터리 center AP 저하는 temporal stacking 하나로 설명되지 않으며 scene
    domain·semantic·confidence 문제도 함께 존재함.
  - 본 모델은 3-sweep으로 학습됐으므로 1-sweep 결과는 production 후보가 아니라
    원인 분리용 진단임.
  - 다음 통제 실험은 동일 Base2 데이터로 학습한 진짜 single-frame S0여야 함.

## Base2 S0 single-frame 통제 실험.

- D4가 보존한 target-frame painted cache에서 9-feature S0 dataset을 파생함.
  - 경로: `artifacts/datasets/morai_pointpainting_base2_singleframe9`.
  - Train `5,509`, Val `1,181`, 총 `6,690` sample임.
  - T0와 target frame과 split이 정확히 동일함.
  - Point 파일은 D4 cache와 hard-link하므로 센서 처리 재실행과 3.1 GB 중복
    저장을 피함.
- S0 학습 계약은 다음과 같음.
  - 현재 1-sweep만 사용하는 9-feature PointPainting임.
  - 7D box head이며 velocity regression과 `time_lag_sec`를 제거함.
  - T0와 동일한 batch size 4, 40 epoch, one-cycle LR, 무증강 정책을 사용함.
  - E65가 사용한 GT sampling·flip·rotation·scale은 첫 통제 실험에 넣지 않음.
    데이터와 temporal/velocity 계약 차이를 먼저 분리한 뒤 별도 ablation함.
- Dataloader smoke 결과는 다음과 같음.
  - Train sample `5,509`, batch `1,378`.
  - Item point shape `(N, 9)`.
  - Item GT shape `(M, 8)`이며 마지막 열은 OpenPCDet가 추가한 class index임.

## 활성 최우선 계획.

- 이 절은 다음 작업에서 반드시 다시 언급할 최우선 항목만 유지함.
- 항목을 완료했거나 검토 결과 더 이상 최우선이 아니면 이 절에서 제거함.
- 새 작업 중 최우선 후속 조치가 생기면 대화에서 명시하고 이 절에 추가함.

### P0. Base2 validation과 online rosbag 사이의 domain gap 진단.

- T0는 Base2 validation에서는 E65에 가까운 detection 성능과 `0.77 m/s` 수준의
  Vehicle mAVE를 보였지만 실제 rosbag에서는 detection과 velocity가 모두 악화됨.
- 다음 실험은 velocity gradient를 키우기 전에 다음 차이를 하나씩 분리함.
  - SaveSensorData와 online rosbag의 camera/LiDAR 시각 및 JPEG 입력 차이.
  - 학습 3-sweep 구성과 online sweep history 구성·time-lag 분포 차이.
  - 저장된 semantic score와 online LRASPP score 분포 차이.
  - Base2 scenario 분포와 실제 로터리·고주로 객체 거리·방향 분포 차이.
- 동일 checkpoint에 대해 single-sweep/temporal, offline/online 입력을 교차 평가해
  geometry 붕괴가 sweep 정합, semantic 입력, 또는 scene domain 중 어디서 시작하는지
  특정함.
- rosbag Vehicle center mAP와 F1이 E65에 근접하기 전에는 velocity head를
  production tracker에 연결하지 않음.

### P1. Vehicle velocity supervision 재설계 ablation.

- 현재 velocity regression channel 가중치는 `vx=0.2`, `vy=0.2`임.
  - 첫 T0에서 geometry 성능을 보호하기 위한 보수적인 기준값임.
  - Vehicle AVE가 약 `1 m/s` 부근에서 정체한다면 supervision이 약한 것이
    병목인지 확인할 근거가 충분함.
- 전체 box 수에서 정지 Obstacle이 다수를 차지하므로 전체 velocity loss 감소만
  보고 판단하지 않음.
  - Vehicle moving label을 class-balanced 또는 moving-balanced 방식으로 별도
    정규화하는 후보를 검토함.
  - 정적 객체의 `0 m/s` supervision은 유지하되 쉬운 zero target이 gradient를
    지배하지 않도록 함.
- 다음 ablation 후보는 한 번에 하나의 요인만 바꿔 비교함.
  - V0: T0와 동일한 `0.2` baseline.
  - V1: Vehicle velocity channel 가중치 `0.5`.
  - V2: `0.2`를 유지하고 class-balanced/moving-balanced velocity loss 적용.
  - V3: V1과 V2가 각각 유효할 때만 두 방법을 결합함.
- 판단 지표는 train 전체 velocity loss가 아니라 다음을 우선함.
  - Vehicle AVE, speed MAE, 종방향/횡방향 MAE.
  - Vehicle Center AP, mATE, BEV IoU AP의 손상 여부.
  - Rosbag tracking velocity error와 Map-CV 1/2/3초 FDE.
- P0 online geometry/domain gap을 해결한 뒤 짧은 ablation으로 gradient scale과
  detection 손상을 먼저 확인하고, 유효한 후보만 본 학습함.
- velocity supervision 정책을 확정하면 이 항목을 제거함.

### P2. Epoch 단위 learning-rate 실시간 W&B 기록.

- 현재 LR은 TensorBoard 원본 `meta_data/learning_rate`로 iteration마다 기록됨.
- `epoch/learning_rate_last` 요약은 학습 종료 후 업로드되므로 진행 중인 run의
  epoch panel에서 즉시 보기에는 부적합함.
- 다음 trainer 수정에서 각 epoch 종료 시 다음을 W&B에 즉시 기록함.
  - `epoch`을 공통 x축으로 사용함.
  - 해당 epoch의 시작 LR, 마지막 LR, 필요 시 평균 LR을 기록함.
  - Train section에서 모델과 무관하게 재사용할 canonical key를 사용함.
- 기존 iteration 단위 TensorBoard LR은 상세 진단용으로 유지함.
- epoch LR panel이 실제 online smoke에서 갱신되는 것을 확인하면 이 항목을 제거함.

### P3. Batch size 8/12와 learning-rate scaling ablation.

- 현재 진행 중인 T0는 검증된 기준선인 batch size 4로 끝까지 학습함.
- T0 완료 후 다음 본 학습 전에 batch size 8을 최우선으로 비교함.
  - 동일 LR과 LR 1.5~2배 후보를 짧은 1~3 epoch smoke로 비교함.
  - samples/sec, epoch 시간, peak VRAM, loss 안정성, epoch validation을 확인함.
- Batch size 8이 안정적이면 batch size 12도 같은 방식으로 확인함.
- 단순히 VRAM을 많이 쓰는 설정이 아니라 throughput과 validation을 함께 비교해
  이후 production 학습 batch size와 LR을 확정함.
- 본 비교가 끝나 설정을 확정하면 이 항목을 완료 기록으로 옮기거나 제거함.
