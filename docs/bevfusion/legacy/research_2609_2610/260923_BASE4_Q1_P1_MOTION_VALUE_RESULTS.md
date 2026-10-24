# Base4 Q1-P1 Motion-value Velocity Branch 결과

> 날짜: 2026-09-23.
> 상태: CURRENT 연구 결과. Production 미적용.

## 목적

Q1-P0는 과거 query의 위치 차이와 시간 간격을 attention logit에만 넣었다. 현재 query와
연결되는 과거 Vehicle query가 하나뿐이면 softmax weight는 항상 1이므로, 이동 방향과
크기가 value에 직접 전달되지 않는 한계가 있었다.

P1은 P0의 causal query memory를 유지하면서 다음 두 경로를 추가한다.

- Ego 정합 후 과거 center와 현재 center의 차이를 age로 나눈 `(vx, vy, age)`를
  motion-value MLP로 변환해 memory value에 더한다.
- motion context 전용 2D velocity head를 두고 기존 TransFusion velocity와 작은
  learnable gate로 융합한다.
- memory와 연결된 positive Vehicle query에만 기존 frame velocity GT를 사용한 Smooth-L1
  auxiliary loss를 준다.

이 단계는 아직 GT identity 기반 displacement-consistency loss가 아니다. 위치 차이와
delta-time은 입력 value에 명시적으로 들어가지만 auxiliary target은 기존 annotation의
velocity다.

## 구현 및 안전장치

- P0 config와 checkpoint key는 그대로 유지하며 P1 옵션의 기본값은 off다.
- temporal velocity fusion gate는 sigmoid(-2), 약 `0.119`에서 시작한다.
- temporal auxiliary loss weight는 `0.5`다.
- 유효한 과거 Vehicle pair가 없는 query에는 temporal correction을 적용하지 않는다.
- memory bank는 이전과 같이 detach하며 scene 변경, timestamp 역행, 큰 gap에서 reset한다.
- batch 1, ordered scene sampler와 identity global augmentation 계약을 유지한다.

주요 파일:

- `plugin/morai_query_memory.py`
- `plugin/morai_transfusion_head.py`
- `configs/bevfusion_base4_q1_p1_motion_value_4cam_lidar_velocity.py`
- `configs/team/recipes/base4_q1_p1_motion_value_*.yaml`

## 검증

- Python compile: 통과.
- motion-value tensor shape 및 gradient 단위시험: 통과.
- 실제 4-camera+LiDAR 16-sample forward/backward/checkpoint/validation smoke: 통과.
- smoke peak VRAM: 약 `3.2 GiB`. Production latency 수치로 해석하지 않는다.

## 64-sample exact overfit

동일 64 frame을 학습과 평가에 사용하므로 wiring과 학습 가능성만 판단한다.

| 평가 시점 | P1 speed MAE | P1 mAVE | center mAP |
| ---: | ---: | ---: | ---: |
| E2 | 7.342 | 7.350 | 0.9093 |
| E4 | 3.105 | 3.149 | 0.9508 |
| E6 | 2.743 | 2.775 | 0.9598 |
| E8 | 2.470 | 2.511 | 0.9602 |
| E10 | 2.239 | 2.286 | 0.9598 |
| E12 | **2.211** | **2.259** | **0.9600** |

P0 memory-on E12 `3.061 m/s` 대비 P1은 `2.211 m/s`로 약 `27.8%` 개선됐다. 명시적인
motion value와 velocity auxiliary branch가 실제로 optimization에 쓰인다는 근거다.

## Hardcase V2 scene-disjoint E4

Train 543 frame/43 scene, val 108 frame/8 scene을 사용했다.

| Epoch | speed MAE | mAVE | mATE | Vehicle center mAP |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 3.516 | 3.559 | 0.2867 | 0.8957 |
| 2 | 3.210 | 3.256 | 0.2956 | 0.8822 |
| 3 | **3.133** | **3.160** | 0.2794 | 0.8917 |
| 4 | 3.166 | 3.195 | **0.2745** | **0.8921** |

Best speed MAE 기준:

- P0 memory-off: `4.075 m/s`.
- P0 memory-on: `3.854 m/s`.
- P1 motion-value: `3.133 m/s`.

P1은 P0 memory-on보다 약 `18.7%`, memory-off보다 약 `23.1%` 개선했다. P1 E3 center
mAP `0.8917`은 P0 memory-on best-speed checkpoint의 `0.8874`보다 낮지 않으므로 공간
성능 보존 gate도 통과한다.

W&B:

- [P1 exact64 E12](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p1-motion-value-overfit64-w1-head5x-e12-rtx5060ti-260923-071413)
- [P1 hardcase V2 E4](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p1-motion-value-hardcase-v2-w1-head5x-e4-rtx5060ti-260923-071850)

## 결론과 다음 gate

P1은 P0보다 명확히 낫고 temporal motion을 직접 value로 주는 방향은 유효하다. 그러나
scene-disjoint speed MAE `3.133 m/s`는 첫 실용 gate `<1.0 m/s`와 공격 목표
`0.5 m/s`에 크게 못 미친다. 따라서 현재 checkpoint는 rosbag, tracker 또는 production
후보로 올리지 않는다. Production은 계속 E25 7D와 detector velocity fusion off다.

다음 단계는 architecture를 무작정 키우기 전에 다음을 분리해 판정한다.

1. GT identity를 학습/진단에만 사용한 frame displacement-consistency target을 만든다.
2. 동일 외형·위치에서 속도만 다른 counterfactual mini split으로 scene prior 의존도를
   측정한다.
3. query-level pair coverage가 낮은 원인을 분석하고, 필요하면 GT-matched positive query
   history 또는 object-token memory로 바꾼다.
4. 이 단계에서 `<1.0 m/s`에 접근할 때만 개발 rosbag과 latency gate로 진행한다.
