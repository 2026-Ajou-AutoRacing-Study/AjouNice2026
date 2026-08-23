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

## 활성 최우선 계획.

- 이 절은 다음 작업에서 반드시 다시 언급할 최우선 항목만 유지함.
- 항목을 완료했거나 검토 결과 더 이상 최우선이 아니면 이 절에서 제거함.
- 새 작업 중 최우선 후속 조치가 생기면 대화에서 명시하고 이 절에 추가함.

### P0. Batch size 8/12와 learning-rate scaling ablation.

- 현재 진행 중인 T0는 검증된 기준선인 batch size 4로 끝까지 학습함.
- T0 완료 후 다음 본 학습 전에 batch size 8을 최우선으로 비교함.
  - 동일 LR과 LR 1.5~2배 후보를 짧은 1~3 epoch smoke로 비교함.
  - samples/sec, epoch 시간, peak VRAM, loss 안정성, epoch validation을 확인함.
- Batch size 8이 안정적이면 batch size 12도 같은 방식으로 확인함.
- 단순히 VRAM을 많이 쓰는 설정이 아니라 throughput과 validation을 함께 비교해
  이후 production 학습 batch size와 LR을 확정함.
- 본 비교가 끝나 설정을 확정하면 이 항목을 완료 기록으로 옮기거나 제거함.
