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

## S0 online semantic ablation과 Vehicle 난이도 audit.

- S0 E28의 foreground semantic probability를 `1.0/0.5/0.25/0.0`으로 줄여
  새 로터리와 고주로 rosbag을 재평가함.
  - 줄어든 foreground probability는 background로 이동해 feature 확률 합을
    항상 1로 유지함.
  - Detector velocity는 발행하지 않고 tracker 위치 차분 속도만 사용함.
- Semantic scale `1.0`이 두 장면 모두 가장 높았음.
  - 로터리 Vehicle center mAP: `0.553/0.536/0.535/0.536`.
  - 고주로 Vehicle center mAP: `0.710/0.696/0.680/0.674`.
  - online camera/LiDAR 비동기가 존재하더라도 semantic painting을 약화하는
    것만으로 성능이 회복되지 않으므로 주 병목 가설에서 제외함.
- Base2 Val E28 prediction과 online rosbag을 30 m 안의 Vehicle positive 기준으로
  동일하게 slice audit함.
  - Base2 Val은 전방 `72.3%`, 좌측 `7.7%`, 우측 `2.2%`, 후방 `17.8%`임.
  - 새 로터리는 전방 `50.7%`, 좌측 `21.5%`, 후방 `27.8%`임.
  - Base2 Val recall@1m은 전방/좌측/우측/후방
    `100/100/100/93.1%`임.
  - 새 로터리는 `80.6/29.5/-/86.1%`임.
- 가장 큰 실패 조합은 `좌측 + Ego yaw-rate >= 5 deg/s`임.
  - Base2 Val은 이 조합이 6개뿐이며 모두 point가 매우 조밀함.
  - 새 로터리는 50개이며 recall@1m이 `22.0%`, score mean이 `0.073`임.
  - 같은 고회전 후방 Vehicle 36개는 recall@1m `100%`이므로 회전 하나만의
    문제가 아니라 회전 중 측방 관측 조합의 domain gap임.
- Train split도 별도로 확인함.
  - 30 m 안의 Vehicle 1,697개 중 좌측+고회전은 44개(`2.6%`)뿐임.
  - 실제 로터리는 284개 중 50개(`17.6%`)라 약 6.8배 높은 비중임.
  - Train의 로터리 두 family에 한정하면 좌측+고회전 Vehicle은 19개뿐임.
- Base2의 `roundabout_circulation` Val Vehicle 50개는 전방 17개·후방 33개이고
  좌우 측방이 0개임.
- 전체 Train frame 5,509개 중 30 m 안에 Vehicle이 있는 frame은 1,643개
  (`29.8%`), 모든 객체가 없는 frame은 1,057개(`19.2%`)임.
  - no-Vehicle frame 비중은 높지만 로터리 miss가 좌측+고회전에 집중되므로
    hard negative 과다만으로 현재 실패를 설명하기는 어려움.
  - 생성된 raw Vehicle box 중 약 절반은 30 m model range 밖이라 dataloader에서
    Vehicle supervision으로 사용되지 않는 점은 다음 dataset quota에서 수정함.
- 로터리 point-count p50은 53개, Base2 Val은 63개라 차이는 있지만 주 병목으로
  보기 어려움.
  - 로터리 좌측 Vehicle은 point-count p50 51개인데도 recall이 29.5%임.
  - online point count는 adjusted detection stamp와 가장 가까운 raw LiDAR scan을
    연결한 근삿값이며 방향·score·recall 통계에는 이 근사가 들어가지 않음.
- 재현 가능한 도구와 결과는 다음과 같음.
  - `perception/pointpainting/tools/audit_base2_vehicle_difficulty.py`.
  - `artifacts/detection3d/pointpainting/analysis/base2_s0_vehicle_difficulty`.

## S1 single-frame geometric augmentation ablation 준비.

- 목적은 S0의 실제 rosbag 측방·고회전 Vehicle domain gap이 기하 증강 부재에서
  비롯됐는지 통제 실험으로 확인하는 것임.
- S0와 동일하게 유지하는 조건은 다음과 같음.
  - Base2 single-frame9 Train `5,509`, Val `1,181`과 동일 split.
  - 9-feature painted point, 7D box head, 3-class anchor, optimizer, LR, batch size 4,
    40 epoch를 그대로 사용함.
  - velocity head와 GT sampling은 사용하지 않음.
- Train에만 다음 기하 증강을 적용함.
  - `random_world_flip`: x/y 축.
  - `random_world_rotation`: `-45~+45 deg`.
  - `random_world_scaling`: `0.95~1.05`.
- OpenPCDet config loader는 중첩 `_BASE_CONFIG_`를 재귀적으로 병합하지 않으므로
  model과 dataset S1 YAML에 전체 계약을 명시함.
  - S0/S1 config를 로드해 비교한 결과 `DATA_AUGMENTOR` 이외의 model, optimizer,
    class, dataset 항목이 동일함을 확인함.
- 증강 smoke 결과는 다음과 같음.
  - Train augmentor 3개, Val augmentor 0개임.
  - 다른 seed의 동일 Train sample은 GT와 point가 함께 변함.
  - Val sample은 seed와 무관하게 동일함.
  - synthetic box center와 해당 point의 변환 후 거리 오차는 `0`임.
  - intensity와 semantic painting feature는 기하 증강 전후 동일함.
- 1 epoch 전체 offline smoke를 완료함.
  - Train `1,378` iteration과 Val `1,181` frame이 정상 종료됨.
  - AMP, checkpoint, best checkpoint, epoch validation, W&B offline logging이
    모두 정상임.
  - 소요 시간은 약 `2분 40초`임.
  - 1 epoch OneCycle을 한 번에 끝낸 `center_mAP=0.4328`은 구조 검증값일 뿐
    성능 비교에 사용하지 않음.
- 본 S1 평가 순서는 다음과 같음.
  - 먼저 Base2 Val의 center AP, mATE, mASE, mAOE, BEV IoU를 S0와 비교함.
  - 같은 새 로터리·고주로 rosbag을 재평가함.
  - 특히 로터리 `좌측 + Ego yaw-rate >= 5 deg/s` recall과 score를 다시 audit함.
  - S1의 목표는 offline Val의 작은 상승보다 online 측방 domain gap 회복임.
  - S1도 해당 slice를 회복하지 못하면 기하 증강만으로 해결할 수 없다고 보고,
    targeted side/high-yaw data 또는 rotational rolling-scan 입력 문제로 넘어감.

## S1 single-frame geometric augmentation 최종 결과.

- 최종 비교 checkpoint는 S0 E28과 S1 E38임.
  - S0: `best_center_mAP_epoch_28.pth`.
  - S1: `best_center_mAP_epoch_38.pth`.
- Base2 Val에서는 S1이 전반적으로 소폭 개선됨.
  - 전체 center mAP: `0.9381 -> 0.9417`.
  - Vehicle center AP: `0.9738 -> 0.9767`.
  - Vehicle mATE: `0.1043 -> 0.0962 m`.
  - Vehicle BEV IoU AP@0.50: `0.9755 -> 0.9800`.
  - Pedestrian center AP: `0.9856 -> 0.9918`.
  - Obstacle center AP: `0.8549 -> 0.8565`이며 BEV IoU AP@0.50은
    `0.7364 -> 0.7709`로 개선됨.
- 동일 online rosbag에서도 Vehicle 검출은 명확히 개선됨.
  - S0와 S1을 같은 runner, score threshold, frame coverage로 재실행해 비교함.
  - 새 로터리 center mAP: `0.5275 -> 0.6878`.
  - 새 로터리 1 m operating-point F1: `0.6232 -> 0.8836`.
  - 고주로 center mAP: `0.7123 -> 0.7717`.
  - 고주로 1 m operating-point F1: `0.5739 -> 0.9403`.
  - 고주로 AP@0.5 m는 `0.3195 -> 0.2711`로 낮아져, coarse recall 개선이
    sub-meter 위치 정밀도 개선을 의미하지는 않음.
- 기존 production E65도 같은 runner와 frame coverage로 추가 재평가함.
  - 새 로터리 Vehicle center mAP는 E65 `0.6566`, S1 `0.6878`임.
  - 새 로터리 F1@1m는 E65 `0.8744`, S1 `0.8836`임.
  - 고주로 center mAP는 E65 `0.6571`, S1 `0.7717`임.
  - 고주로 F1@1m는 E65 `0.8010`, S1 `0.9403`임.
  - E65 대비 online Vehicle detection은 S1이 두 장면 모두 우세함.
- 목표였던 측방·고회전 domain gap도 회복됨.
  - 새 로터리 좌측 Vehicle recall@1m: `29.5% -> 54.1%`.
  - 새 로터리 Ego yaw-rate 10 deg/s 초과 recall@1m: `56.0% -> 74.5%`.
  - 고주로 좌측 Vehicle recall@1m: `56.3% -> 87.5%`.
- 객체가 전혀 없는 `competition_route_full_highspeed.bag`의 false detection도
  크게 감소함.
  - Vehicle false prediction: `940 -> 55`, 검출 frame 비율 `36.0 -> 2.5%`.
  - Pedestrian: `158 -> 19`, 검출 frame 비율 `7.2 -> 1.0%`.
  - Obstacle: `6,148 -> 1,193`, 검출 frame 비율 `84.7 -> 41.2%`.
  - Obstacle false positive는 여전히 높으므로 full-route hard-negative mining이
    후속 과제로 남음.
- S1의 주 부작용은 online Vehicle의 180도 방향 반전임.
  - 새 로터리 raw mAOE: `0.311 -> 0.785 rad`지만 pi-symmetric mean은
    `0.147 -> 0.055 rad`로 개선됨.
  - 고주로 raw mAOE: `0.063 -> 0.472 rad`지만 pi-symmetric mean은
    `0.061 -> 0.041 rad`로 개선됨.
  - 90도 초과 방향 반전 비율은 로터리 `7.8 -> 24.4%`, 고주로
    `0.3 -> 14.1%`임. 거의 모든 표본이 135도 이상이라 일반 yaw noise가 아니라
    정확한 앞뒤 방향 ambiguity임.
  - S1이 사용한 x/y 양축 flip 중 front-back flip이 방향 supervision을 불필요하게
    흔든 가능성이 가장 큼.
  - 다만 E65 raw mAOE도 로터리 `0.885 rad`, 고주로 `0.583 rad`로 S1의
    `0.785/0.472 rad`보다 더 나쁨. S1의 yaw 문제는 S0 대비 regression이지만
    현재 E65 production 대비로는 개선임.
- 결론은 S1 가설 자체는 채택함.
  - 기하 증강은 실제 측방 recall과 hard-negative 일반화를 크게 개선함.
  - 다만 현재 S1 E38을 그대로 production으로 확정하지 않고, left-right flip만
    유지하고 front-back flip을 제거한 S1b를 다음 통제 실험으로 수행함.
  - S1b에서도 raw yaw 반전이 남으면 detector 출력 yaw를 이동 방향·Lanelet로
    정규화하는 후처리와 direction-head 학습을 별도로 비교함.

## 팀원 B0 E80 object-compensated 3-sweep checkpoint audit.

- 전달받은 원본 artifact는 다음 release 디렉터리에 보존함.
  - `artifacts/detection3d/pointpainting/releases/260824_base2_team_b0_temporal3_object_compensated_80e`.
  - checkpoint는 80 epoch, OpenPCDet `0.6.0+233f849`, 127개 state tensor임.
- 제공 model YAML의 주요 차이는 다음과 같음.
  - 3-sweep 9-feature이며 `time_lag_sec`와 velocity head가 없음.
  - 7D box, direction head, Pedestrian/Vehicle 1-size anchor를 사용함.
  - Obstacle은 6-size anchor를 사용함.
  - batch size 2, 80 epoch, OneCycle LR `0.003`, `PCT_START=0.4`임.
- 제공되지 않은 artifact가 있음.
  - YAML이 참조하는 dataset config가 빠져 있어 9-feature/range30 계약을 복원함.
  - object-motion compensation을 수행하는 train/validation builder와 online
    inference 구현은 제공되지 않음.
  - 따라서 팀원 Val 점수는 같은 raw Base2에서 나왔더라도 전처리 입력 분포가
    우리 S0/S1과 다르며, 현재 파일 두 개만으로 동일 Val을 재현할 수 없음.
- online node에 9-feature temporal history를 지원하는 일반 경로를 추가함.
  - 기존 10-feature temporal model의 time-lag 동작은 유지함.
  - 9-feature model은 과거 sweep을 Ego pose로 현재 LiDAR frame에 옮기되
    `time_lag_sec`를 추가하지 않음.
  - 이는 production에서 가능한 Ego-only 조건이며 object compensation은 아님.
- 동일 rosbag의 Vehicle 결과는 다음과 같음.

| 장면 | 입력 | center mAP | F1@1m | mATE | mAOE |
|---|---:|---:|---:|---:|---:|
| 로터리 | Team E80 single-sweep | 0.446 | 0.731 | 0.552 m | 1.049 rad |
| 로터리 | Team E80 Ego-only 3-sweep | 0.200 | 0.360 | 0.873 m | 1.131 rad |
| 고주로 | Team E80 single-sweep | 0.646 | 0.889 | 0.492 m | 0.641 rad |
| 고주로 | Team E80 Ego-only 3-sweep | 0.268 | 0.409 | 0.847 m | 0.515 rad |

- S1 E38은 같은 조건에서 로터리/고주로 center mAP `0.688/0.772`, F1@1m
  `0.884/0.940`임.
- 객체가 없는 전체 경로에서 Team E80 Ego-only 3-sweep은 오검출이 적음.
  - Vehicle 9개, Pedestrian 4개, Obstacle 896개임.
  - 검출 frame 비율은 각각 `0.47/0.21/29.28%`임.
  - S1의 `2.84/0.98/42.25%`보다 낮아 background 억제는 강함.
- runtime은 12.54 Hz 입력을 drop 없이 처리함.
  - Team E80 Ego-only 3-sweep compute p50/p95 `54.26/61.51 ms`.
  - S1 single-sweep `43.14/46.98 ms`보다 p50 약 11.1 ms 느림.
- 결론은 checkpoint 자체의 고득점 가능성을 부정하는 것이 아님.
  - object-compensated Val에서는 동적 객체 point가 한 box에 정렬되어 훨씬 쉬운
    분포가 되므로 높은 점수가 나올 수 있음.
  - 하지만 online에서 같은 정렬을 하려면 이전 detection/tracking association과
    객체 motion 추정이 먼저 필요함. GT `/Object_topic`을 사용하면 oracle 평가일
    뿐 production 검증이 아님.
  - 팀원 inference 전처리 코드나 명확한 causal object-compensation 계약을 받기
    전에는 이 checkpoint를 production 후보로 비교하지 않음.

## 활성 계획 문서 이동.

- 프로젝트 전체의 현재 최우선 작업은
  [`docs/ACTIVE_PRIORITIES.md`](../../../docs/ACTIVE_PRIORITIES.md)에서 관리한다.
- 이 문서는 Base2 temporal/velocity D4의 실험 근거와 완료된 판단 기록으로 보존한다.
