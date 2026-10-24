# Base4 Q1-P3 Robust Motion 결과

> 날짜: 2026-09-23.
> 상태: CURRENT 연구 결과. Production 미적용.

## 목적과 구현

Q1-P2는 post-decoder center 계약을 바로잡아 평균 velocity를 크게 개선했지만 개발
highway1 bag에서 adjacent error-step p95가 `8.351 m/s`로 악화했다. 한 frame의 query
association과 learned velocity head에 지나치게 의존하는 것이 남은 문제였다.

P3는 최대 4개 causal memory frame의 다음 값을 활용한다.

- Ego pose로 현재 LiDAR frame에 정합한 과거 post-decoder center.
- `(current center - past center) / age`로 계산한 pair velocity.
- 기존 query feature, distance, age attention weight의 head 평균.

attention-weighted pair velocity를 temporal velocity의 기준값으로 사용하고, 기존 learned
head는 zero-init residual만 예측한다. 최종 출력은 P2와 동일하게 temporal 90%,
single-frame 10%로 융합한다. memory가 없는 query는 기존 single-frame velocity를 유지한다.

## 검증

- Python compile과 query-memory CUDA container 단위시험: 통과.
- exact64 E8와 hardcase V2 scene-disjoint E4: 완료.
- Q1 online metadata 계약으로 두 개발 rosbag 평가: 완료.

## 학습 gate

### Exact64

| Epoch | speed MAE | mAVE |
| ---: | ---: | ---: |
| 2 | 1.397 | - |
| 4 | 1.083 | - |
| 6 | 0.972 | - |
| 8 | **0.931** | **0.970** |

P2 gate90의 E8 `0.835 m/s`보다 최종 exact 암기 성능은 낮지만, P2 E2 `5.173 m/s`보다
훨씬 빨리 수렴했다. 직접 displacement가 유효한 초기값이고 learned branch가 작은 residual만
학습한다는 의도와 맞는다.

### Hardcase V2 scene-disjoint

| Epoch | speed MAE | mAVE | mATE | Vehicle center mAP |
| ---: | ---: | ---: | ---: | ---: |
| 1 | **0.845** | **0.912** | 0.2931 | **0.9106** |
| 2 | 0.911 | 0.969 | 0.3027 | 0.8959 |
| 3 | 0.977 | 1.046 | 0.2628 | 0.8950 |
| 4 | 0.984 | 1.049 | **0.2578** | 0.8991 |

E1이 첫 현실적 speed gate `<1.0 m/s`를 통과했다. P2 best `1.070 m/s`보다 `21.0%`,
P1 `3.133 m/s`보다 `73.0%` 낮다. 공간 mAP도 P2 best `0.8979`보다 높다. epoch가
늘수록 velocity가 악화하므로 연구 checkpoint는 E1이다.

W&B:

- [P3 exact64 E8](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p3-robust-motion-gate90-overfit64-w1-head5x-e8-rtx5060ti-260923-114744)
- [P3 hardcase V2 E4](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p3-robust-motion-gate90-hardcase-v2-w1-head5x-e4-rtx5060ti-260923-115103)

## 개발 rosbag 결과

Vehicle threshold `0.40`, detector velocity on, deskew off, query-memory context on으로 E1을
평가했다.

| Bag | center mAP | F1@1m | speed MAE | signed bias | error-step p50 / p95 | latency mean / p95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| highway1 | 0.7613 | 0.9585 | 1.582 m/s | +0.285 | 0.985 / 5.569 m/s | 92.8 / 101.8 ms |
| highway merge | 0.5569 | 0.7663 | 1.021 m/s | +0.281 | 0.689 / 3.367 m/s | 95.2 / 118.8 ms |

P2와 비교하면:

- highway1 speed MAE `2.288 -> 1.582`, p95 `8.351 -> 5.569 m/s`.
- merge speed MAE `1.277 -> 1.021`, p95 `4.030 -> 3.367 m/s`.
- center mAP도 highway1 `0.7459 -> 0.7613`, merge `0.5309 -> 0.5569`.

P3 개선은 val에만 머물지 않고 두 bag의 평균과 jitter에서 모두 재현됐다. 그러나
highway1 p95는 T1 E6의 `3.810 m/s`보다 여전히 크고, latency도 12 Hz budget `83 ms`를
넘는다.

## 판정

P3는 다음 Q1 연구 baseline으로 채택한다. 다만 tracker velocity fusion과 production에는
아직 적용하지 않는다.

다음 P4의 우선순위는 평균 MAE를 더 낮추는 것이 아니라 outlier association 제거다.

1. frame별 nearest 후보를 모두 평균하지 않고 feature/geometry mutual match를 적용한다.
2. pair velocity의 robust consensus 또는 innovation gate로 서로 크게 다른 memory pair를
   제외한다.
3. memory-active/fallback별 error와 association switch를 직접 로깅한다.
4. highway1 error-step p95가 T1 `3.81 m/s` 아래로 내려간 뒤에만 tracker/Map-CV A/B를 연다.
5. 정확도 gate 통과 후 5060 Ti runtime의 query count와 attention 비용을 줄인다.

Production은 계속 E25 7D, detector velocity fusion off다.
