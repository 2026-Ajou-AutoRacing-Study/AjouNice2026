# Base4 Q1-P0 Sparse Query Memory 구현 결과

> 날짜: 2026-09-23.
> 상태: CURRENT 구현·smoke 근거. Production 미적용.

## 목적

Q0처럼 검출 center를 직접 미분해 velocity를 덮어쓰지 않고, TransFusion Vehicle query가
과거의 동일 공간 후보 feature를 읽어 9D velocity head를 학습할 수 있는 최소 causal
memory를 검증한다. StreamPETR 전체 포팅이 아니라 우리 BEVFusion에서 가설을 가장 작게
검증하는 P0다.

## 구현 계약

- 과거 최대 4 frame, `0.6 s`, frame당 Vehicle query 최대 64개를 detach해 저장한다.
- `lidar2global` Ego pose로 과거 center를 현재 LiDAR frame에 변환한다.
- class, score `>=0.20`, age와 거리 `<=6 m` gate 후 1-layer multi-head
  cross-attention을 적용한다.
- scene 변경, timestamp 역행, `0.30 s` 초과 gap이면 memory를 reset한다.
- residual gate는 sigmoid(-4), 약 `0.018`에서 시작해 E25 공간 검출을 거의 보존한다.
- P0 학습은 batch 1, shuffle off, 단일 GPU, scene 연속 순서를 강제한다.
- 독립 frame global augmentation은 금지하고 identity transform을 쓴다. 추후 shared
  clip augmentation을 만들기 전까지 이 제한을 유지한다.
- Pedestrian/Obstacle query는 P0 memory 대상이 아니다.

주요 파일:

- `plugin/morai_query_memory.py`
- `plugin/morai_transfusion_head.py`의 `MoraiQueryMemoryTransFusionHead`
- `configs/bevfusion_base4_q1_query_memory_4cam_lidar_velocity.py`
- `tools/audit_query_memory_contract.py`
- `tools/test_query_memory.py`

## 검증 결과

### Tensor 및 metadata 계약

- Ego rotation/translation, mask, FIFO/reset, gradient 단위시험: 통과.
- hardcase V2 train: 543 frame, 43 scene, causal edge 500개, 계약 위반 0건.
- hardcase V2 val: 108 frame, 8 scene, causal edge 100개, 계약 위반 0건.
- timestamp edge는 모두 `0.30 s` 이하여서 scene 내부 불필요 reset은 없었다.

### End-to-end smoke

실제 4카메라+LiDAR 연속 4 frame에서 다음을 확인했다.

- E25 로드 시 누락 key는 예상한 새 velocity head와 query-memory뿐이다.
- full preprocessing, loss, AMP backward, accumulation, checkpoint와 ordered val이 완료됐다.
- synthetic 2-frame head 시험에서 두 번째 frame memory가 활성화되고 memory projection에
  nonzero gradient가 흘렀다.
- smoke peak VRAM은 약 `3.2 GiB`였다. 이는 4-frame 기능 시험값이며 production latency나
  일반화 수치가 아니다.

### 첫 E4 진단의 실패와 교정

첫 64-sample E4는 E25에서 새 velocity head를 만들면서 velocity loss weight `0.2`, 전체
LR `2e-5`를 그대로 사용했다. speed MAE가 E1 `9.313 m/s`, E4 `9.570 m/s`였고 memory
residual gate도 약 `0.018`에 머물렀다. 이는 Q1 비교가 아니라 새 branch를 사실상
학습하지 않은 잘못된 optimization control이다.

따라서 기존 Base4 learnability 실험과 동일하게 다음을 교정했다.

- velocity loss weight `1.0`.
- base LR `1e-4`.
- velocity head와 query-memory LR multiplier `5x`.
- 64 exact-sample, 12 epoch, scene 순서 유지.

교정 E12 결과는 아래에 기록한다.

### 64-sample exact overfit matched A/B

두 실험은 같은 64개 frame, E25 초기화, batch 1/accumulation 4, velocity loss weight
`1.0`, base LR `1e-4`, velocity/memory branch `5x`를 사용했다. Memory-off는 동일한
head와 ordered loader를 유지하고 FIFO 길이만 0으로 만들어 비교했다.

| Epoch | Memory-on speed MAE | Memory-off speed MAE |
| ---: | ---: | ---: |
| 2 | 7.830 m/s | 7.891 m/s |
| 4 | 4.141 m/s | 4.234 m/s |
| 6 | 3.513 m/s | 3.571 m/s |
| 8 | 3.290 m/s | 3.342 m/s |
| 10 | 3.123 m/s | 3.229 m/s |
| 12 | **3.061 m/s** | **3.185 m/s** |

Memory-on이 모든 평가 epoch에서 일관되게 낮았고 최종 차이는 약 `3.9%`였다. 다만
동일 표본 평가이고 절대 오차가 크므로 이 결과는 wiring/learnability 증거일 뿐이다.

### Hardcase V2 scene-disjoint matched A/B

Train 543 frame/43 scene, val 108 frame/8 scene 전체를 사용해 E4를 수행했다.

| Epoch | Memory-on speed MAE | Memory-off speed MAE | Memory-on center mAP | Memory-off center mAP |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 4.719 m/s | 4.656 m/s | 0.9148 | 0.9134 |
| 2 | **3.854 m/s** | 4.175 m/s | 0.8874 | 0.8924 |
| 3 | 4.121 m/s | **4.075 m/s** | 0.8936 | 0.8899 |
| 4 | 4.157 m/s | 4.139 m/s | 0.8936 | 0.8840 |

각 run의 최적 speed checkpoint를 비교하면 query memory가 `4.075 → 3.854 m/s`, 약
`5.4%` 개선했다. 대응 center mAP은 `0.8899 → 0.8874`로 약 `0.3%` 감소해 공간 검출
5% 보존 gate는 통과한다. 반면 절대 speed MAE는 `<1.0 m/s` gate에 크게 못 미치며,
E2 이후 다시 악화됐다.

P0 결론은 **sparse query memory 신호는 존재하지만 memory만으로는 충분하지 않다**이다.
이 checkpoint를 rosbag 또는 production 후보로 올리지 않는다. 다음 단계는 GT identity를
학습/진단에만 사용해 연속 frame의 object displacement와 velocity head를 직접 연결하는
P1 auxiliary supervision이며, 동일 memory-off/on 대조 계약을 유지한다.

W&B:

- [memory-on exact64 E12](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-query-memory-overfit64-w1-head5x-e12-rtx5060ti-260923-063203)
- [memory-off exact64 E12](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-query-memory-off-control-overfit64-w1-head5x-e12-rtx5060ti-260923-064041)
- [memory-on hardcase V2 E4](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-query-memory-hardcase-v2-w1-head5x-e4-rtx5060ti-260923-064643)
- [memory-off hardcase V2 E4](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-query-memory-off-control-hardcase-v2-w1-head5x-e4-rtx5060ti-260923-065415)

## 판정 경계

- exact-sample overfit은 구현·최적화 가능성만 말하며 일반화 성능이 아니다.
- P0는 hardcase V2 scene-disjoint A/B까지 완료했고 절대 speed gate에는 실패했다.
- P1 auxiliary supervision 전에는 추가 full train 또는 rosbag gate를 시작하지 않는다.
- 이 둘을 통과하기 전에는 rosbag, tracker, Map-CV 성능 주장을 하지 않는다.
- Production은 계속 E25 7D + detector velocity fusion off다.
