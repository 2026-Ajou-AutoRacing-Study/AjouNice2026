# Base4 T2 sweep-separated temporal BEV P0

마지막 갱신: 2026-09-22.

## 결론

Base4 T1의 early point concatenation 다음 단계로, 세 LiDAR sweep을 공유 encoder로
각각 처리한 뒤 BEV 공간에서 합치는 두 가지 최소 구조를 구현·검증했다.

- `T2 gate`: 위치별 softmax convex mixing. 성능 gate에서 기각.
- `T2R`: current BEV에 ordered temporal delta residual을 더하는 구조. 두 고주로
  rosbag에서 T1 대비 velocity 평균 오차와 frame jitter를 모두 줄여 후속 후보로 남겼다.

Production은 계속 Base3 E25 7D이며 tracker detector-velocity fusion도 `off`다.
T2R은 아직 두 개의 반복 사용 dev bag에서만 검증했고 LiDAR encoder를 3회 실행하므로,
이 결과만으로 production을 교체하지 않는다.

## 구현 계약

- 입력은 T1과 동일한 `[x, y, z, intensity, age_sec]` causal 3-sweep이다.
- dataset loader가 과거 point XYZ를 현재 LiDAR frame으로 Ego 정합한다.
- 모델은 `age_sec=0`, 최근 과거, 먼 과거로 point를 분리한다.
- 세 sweep에 동일한 voxel/sparse encoder를 적용한다.
- encoder 출력 세 개를 위치별 softmax gate로 합친 뒤 기존 camera BEV와 fusion한다.
- camera는 현재 frame만 사용한다.
- sweep 누락 또는 age group 수가 맞지 않으면 묵시적으로 단일 frame으로 퇴행하지 않고
  즉시 실패한다.

구현:

- `plugin/morai_temporal_bevfusion.py`
- `configs/bevfusion_base4_t2_4cam_lidar_temporal_bev_velocity.py`
- `configs/team/recipes/base4_highway_first_exp_t2_temporal_bev_e4.yaml`
- `tools/test_temporal_bev_gate.py`

## Warm start와 gate 초기화

T1 checkpoint에는 temporal gate가 없으므로 해당 파라미터만 새로 초기화한다. 초기
softmax 평균은 current/t-1/t-2가 약 `0.9951/0.00247/0.00247`이다. 따라서 학습 시작
직후 기존 current-frame geometry를 거의 보존하면서도 두 과거 sweep에 gradient가
흐른다.

Smoke checkpoint에서 확인한 값:

- gate weight absolute max: `2.93e-5`
- gate bias: `[4.000022, -2.000022, -2.000024]`

즉 새 gate는 optimizer에 포함되고 실제로 업데이트되며 checkpoint에도 보존된다.

## 2026-09-22 smoke 결과

- dataset: Base4 highway first experiment raw
- train/val: 각각 16 sample
- epoch/update: 1 epoch, 4 optimizer updates
- physical batch: 4
- GPU memory: 약 `10.6 GB` on RTX 5060 Ti
- train, val, best velocity checkpoint, best mAP checkpoint, last checkpoint: 모두 성공
- W&B:
  `https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-highway-first-exp-t2-temporal-bev-e4-rtx5060ti-260922-192946-smoke`

4 update뿐인 smoke의 mAP, mAVE, speed MAE는 성능 근거로 사용하지 않는다. Smoke는
integration 및 gradient 검증만 담당한다.

## T2 gate 정식 E4: 기각

Base4 highway first experiment raw의 1,688 train / 340 val에서 4 epoch를 학습했다.

| epoch | center mAP | mATE | mAVE | speed MAE |
|---:|---:|---:|---:|---:|
| 1 | 0.2984 | 0.2230 | 3.6722 | 3.6424 |
| 2 | 0.2987 | 0.2281 | **3.6027** | **3.5645** |
| 3 | **0.3013** | 0.2162 | 3.6888 | 3.6480 |
| 4 | 0.2998 | **0.2154** | 3.6854 | 3.6541 |

Best velocity E2를 merge bag에 적용했을 때 AP `0.7562`, speed MAE `3.7982 m/s`,
signed bias `+2.4928 m/s`, error-step p95 `4.2041 m/s`였다. T1 E6의 val speed
MAE `2.3373 m/s`보다 나쁘고 rosbag jitter도 줄지 않았다. 학습 후에도 gate
평균이 current sweep에 약 99.5% 머물러, convex mixing이 현재 geometry를 보존하는
대신 순서가 있는 motion delta를 충분히 노출하지 못한 것으로 판단했다.

W&B:
<https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-highway-first-exp-t2-temporal-bev-e4-rtx5060ti-260922-193951>.

## T2R residual-delta 구조

Gate의 실패를 반영해 다음 입력을 1x1 convolution으로 합성한 후 current BEV에
residual로 더했다.

1. current BEV
2. current minus t-1 BEV
3. t-1 minus t-2 BEV

마지막 convolution을 zero-init해 초기 output이 current BEV와 정확히 같게 했다.
이로써 Base3/T1에서 학습한 geometry를 초기에 보존하면서, 순서가 있는 두 시간차를
모델이 직접 사용할 수 있게 했다.

구현:

- `MoraiTemporalBEVResidual`
- `configs/bevfusion_base4_t2r_4cam_lidar_temporal_bev_velocity.py`
- `configs/team/recipes/base4_highway_first_exp_t2r_temporal_delta_e4.yaml`

Inference server의 기존 warmup이 current-frame point만 만들어 temporal model이 기동하지
못하는 배포 버그도 같이 발견했다. 모델의 `temporal_num_sweeps`를 읽어 `0.0/0.1/0.2 s`
age group을 만드는 warmup으로 수정했고 Docker test로 고정했다.

## T2R 정식 E4 결과

| epoch | center mAP | mATE | mAVE | speed MAE |
|---:|---:|---:|---:|---:|
| 1 | 0.3015 | 0.2263 | **2.9730** | **2.9510** |
| 2 | **0.3054** | 0.2197 | 3.1714 | 3.1303 |
| 3 | 0.3047 | **0.2171** | 3.2515 | 3.2226 |
| 4 | 0.3033 | 0.2179 | 3.1508 | 3.1192 |

Velocity best E1을 `0.40/0.15/0.10`, 30 m, calibrated BBox-center GT의 동일
계약으로 평가했다.

| model | bag | AP | F1@1m | detector speed MAE | signed bias | error-step p95 |
|---|---|---:|---:|---:|---:|---:|
| T1 E6 | merge | 0.7503 | 0.8810 | 3.419 | -0.291 | 4.220 |
| T2R E1 | merge | **0.7774** | 0.8769 | **2.873** | +0.572 | **2.939** |
| T1 E6 | highway1 | 0.7742 | 0.9128 | 2.589 | -0.265 | 3.810 |
| T2R E1 | highway1 | **0.8109** | **0.9216** | **2.378** | -0.282 | **3.040** |

Position-only tracker(mode 0)와 Map-CV에서도 T1 E6 대비 두 bag 모두 개선했다.

| model | bag | tracking AVE | origin-aligned minFDE@3s |
|---|---|---:|---:|
| T1 E6 mode 0 | merge | 1.104 | 3.080 |
| T2R E1 mode 0 | merge | **1.053** | **2.051** |
| T1 E6 mode 0 | highway1 | 0.747 | 1.918 |
| T2R E1 mode 0 | highway1 | **0.699** | **1.073** |

반면 detector velocity를 track 초기화에만 쓰는 mode 1은 merge에서 AVE/FDE를
`1.201/2.734`로 악화시켰고 highway1에서도 FDE를 `1.139`로 소폭 악화시켰다.
따라서 **T2R을 연구 후보로 유지하되 velocity fusion은 off로 유지**한다. 이 결과는
제한된 dev bag이므로 새 held-out 고주로 bag과 online latency gate가 남아 있다.

RTX 5060 Ti Docker runtime에서 merge bag의 모델 보고 latency는 396 frame 기준
mean `128.0 ms`, p50 `125.0 ms`, p95 `170.0 ms`였다. 단순 역수는 약 `7.8 Hz`로
12 Hz LiDAR를 추종하지 못한다. 4090에서 다시 측정해야 하지만, 3회 encoder
구조를 그대로 production에 넣기보다 BEV cache를 재사용하는 구조가 필요하다.

W&B:
<https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-highway-first-exp-t2r-temporal-delta-e4-rtx5060ti-260922-201033>.

## 아직 해결하지 않은 부분

- 현재는 LiDAR encoder를 sweep마다 실행하므로 단일-frame보다 LiDAR encoder 비용이
  약 3배다. 실시간 production 구현이 아니다.
- online recurrent cache와 BEV feature warp는 아직 없다.
- object/query memory, 명시적 delta-time embedding, velocity consistency loss도 없다.
- T2R도 장기 occlusion이나 object association을 직접 모델링하지 않는다.
- 현재 W&B에는 gate 사용률의 epoch 평균을 별도 panel로 기록하지 않는다.

따라서 다음은 T2R을 더 길게 학습하는 것이 아니라, 새 held-out bag 재현과
recurrent cached BEV 또는 object/query memory 중 어느 축으로 실시간화할지 결정하는 것이다.

## 다음 판정 실험

동일 Base4 raw split, 동일 init checkpoint, 동일 4 epoch 조건으로 T1과 T2를 비교한다.
판정 순서는 다음과 같다.

1. val spatial detection이 T1 대비 붕괴하지 않는가.
2. Vehicle speed MAE와 2D AVE가 동시에 개선되는가.
3. 고주로 rosbag의 adjacent-frame velocity error-step p95가 T1의 약
   `3.8--4.2 m/s`보다 내려가는가.
4. tracking AVE와 Map-CV FDE가 실제로 개선되는가.
5. 정확도 이득이 있다면 4090 기준 latency를 측정하고 recurrent cached BEV로
   최적화할 가치가 있는가.

평균 speed MAE만 줄고 jitter나 downstream FDE가 악화되면 채택하지 않는다. T2에서도
속도 안정성이 확보되지 않을 때만 다음 단계인 delta-time embedding, velocity
consistency loss, object/query memory를 순차적으로 검토한다.
