# Base4 T2 sweep-separated temporal BEV P0

마지막 갱신: 2026-09-22.

## 결론

Base4 T1의 early point concatenation 다음 단계로, 세 LiDAR sweep을 공유 encoder로
각각 처리한 뒤 BEV 공간 gate로 합치는 `T2` P0를 구현했다. 16 train/16 val smoke에서
forward, backward, validation, best/last checkpoint 저장까지 통과했다.

이 결과는 **학습 가능성 확인**이지 성능 개선 확인이 아니다. Production은 계속
Base3 E25 7D이며, T2는 동일 dataset과 rosbag 계약으로 T1과 비교하기 전까지 연구용이다.

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

## 아직 해결하지 않은 부분

- 현재는 LiDAR encoder를 sweep마다 실행하므로 단일-frame보다 LiDAR encoder 비용이
  약 3배다. 실시간 production 구현이 아니다.
- online recurrent cache와 BEV feature warp는 아직 없다.
- object/query memory, 명시적 delta-time embedding, velocity consistency loss도 없다.
- 현재 gate는 마지막 위치별 feature를 섞는 최소 구조이므로 장기 occlusion이나
  association을 직접 모델링하지 않는다.
- 현재 W&B에는 gate 사용률의 epoch 평균을 별도 panel로 기록하지 않는다.

따라서 T2 P0의 목적은 **T1 early concat이 velocity generalization의 병목인지**를
최소 변경으로 검사하는 것이다. 이 검사를 건너뛰고 StreamPETR 전체 구조를 포팅하지
않는다.

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
