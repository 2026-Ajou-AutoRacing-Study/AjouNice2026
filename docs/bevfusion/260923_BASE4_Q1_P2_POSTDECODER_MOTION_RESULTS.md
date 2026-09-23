# Base4 Q1-P2 Post-decoder Motion 결과

> 날짜: 2026-09-23.
> 상태: CURRENT 연구 결과. Production 미적용.

## 핵심 결론

Q1-P1에서 temporal motion을 계산할 때 서로 다른 의미의 center를 비교하고 있었다.

- 현재 frame: heatmap proposal에서 뽑은 **decoder 전** center.
- 과거 memory: bbox decoder가 보정한 **decoder 후** center.

실측 decoder center refinement 평균은 약 `0.48 m`였다. Base4의 대표 frame 간격
`~0.15 s`로 나누면 약 `3.2 m/s`이므로, 실제 객체 운동과 무관한 decoder correction이
속도처럼 temporal branch에 들어갈 수 있었다. 이 크기는 P1 scene-disjoint speed MAE
`3.133 m/s`와도 같은 규모다.

P2는 현재와 과거 center를 모두 decoder 후 기준으로 통일했다. 그리고 temporal velocity와
single-frame velocity의 초기 fusion 비율을 통제 실험했다. 최적 gate 90%에서 hardcase V2
scene-disjoint speed MAE가 P1 `3.133 -> 1.070 m/s`로 `65.8%` 개선됐다. 공간 검출은
보존됐다.

다만 첫 실용 gate `<1.0 m/s`를 근소하게 넘었으며 아직 개발 rosbag, jitter, latency,
tracker와 Map-CV 검증 전이다. 따라서 Production은 계속 `B3-BF0-F40 E25` 7D와 detector
velocity fusion off를 유지한다.

## 구현

`query_memory_after_decoder=True`일 때 다음 순서로 동작한다.

1. 기존 TransFusion decoder로 현재 query center를 먼저 보정한다.
2. 보정된 current center와 memory의 보정된 past center로 displacement/delta-time을 구한다.
3. query-memory의 motion context에서 temporal velocity를 회귀한다.
4. memory가 활성화된 Vehicle query에만 기존 velocity와 learnable gate로 융합한다.
5. 보정된 query feature와 center를 다음 frame memory에 저장한다.

기본값은 `False`라 기존 P0/P1 config와 checkpoint 의미는 변하지 않는다. W&B에는
`train/query_center_refinement_m`과 `epoch/query_center_refinement_m_mean`을 추가했다.

주요 파일:

- `plugin/morai_transfusion_head.py`
- `plugin/morai_wandb.py`
- `configs/bevfusion_base4_q1_p2_postdecoder_motion_*.py`
- `configs/team/recipes/base4_q1_p2_postdecoder_motion_*.yaml`

## 검증

- Python compile: 통과.
- P0/P1 기본 동작 보존 tensor 회귀시험: 통과.
- 실제 4-camera + LiDAR forward/backward/checkpoint smoke: 통과.
- smoke peak VRAM: 약 `3.2 GiB`.
- gate 12/50/90/99 exact overfit과 gate 50/90 scene-disjoint A/B 완료.

## 64-sample exact overfit

동일 64 frame을 학습과 평가에 사용하므로 wiring과 학습 가능성만 판단한다.

| 설정 | 비교 epoch | speed MAE (m/s) | mAVE (m/s) | mATE (m) | Vehicle center mAP |
| --- | ---: | ---: | ---: | ---: | ---: |
| P1, gate 12%, pre-decoder | E12 | 2.211 | 2.259 | - | 0.9600 |
| P2, gate 12% | E12 | 2.848 | 2.872 | 0.1434 | 0.9725 |
| P2, gate 50% | E12 | 1.825 | 1.870 | 0.1432 | 0.9683 |
| **P2, gate 90%** | **E12** | **0.806** | **0.905** | **0.1534** | **0.9737** |
| P2, gate 99% | E8 | 1.177 | 1.274 | 0.1905 | 0.9642 |

gate 99%는 gate 90%의 같은 E8 `0.835 m/s`보다 나빴다. temporal displacement를 주
관측으로 쓰되 single-frame velocity를 약 10% 남기는 것이 현재 optimization과 noisy
association에 더 안정적이다. `99%`를 강제로 쓰거나 temporal velocity로 완전히
치환하지 않는다.

## Hardcase V2 scene-disjoint E4

Train 543 frame/43 scene, val 108 frame/8 scene을 사용했다.

| 모델 | best epoch | speed MAE (m/s) | mAVE (m/s) | mATE (m) | Vehicle center mAP |
| --- | ---: | ---: | ---: | ---: | ---: |
| Q1-P0 memory-on | E4 | 3.854 | - | - | 0.8874 |
| Q1-P1 motion-value | E3 | 3.133 | 3.160 | 0.2794 | 0.8917 |
| Q1-P2 gate 50% | E2 | 2.096 | 2.145 | 0.2934 | 0.8952 |
| **Q1-P2 gate 90%** | **E3** | **1.070** | **1.214** | **0.2684** | **0.8979** |

gate 90%는 P1보다 `65.8%`, P0보다 `72.2%` 낮은 speed MAE를 냈다. center mAP도
P1 `0.8917`에서 `0.8979`로 유지·소폭 개선되어 공간 성능 보존 gate를 통과했다.

gate 90% epoch별 결과:

| Epoch | speed MAE | mAVE | mATE | Vehicle center mAP |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 1.228 | 1.363 | 0.3089 | 0.8892 |
| 2 | 1.129 | 1.220 | 0.3051 | 0.8977 |
| 3 | **1.070** | 1.214 | 0.2684 | **0.8979** |
| 4 | 1.079 | **1.195** | **0.2632** | 0.8916 |

## W&B

- [P2 gate 12% exact64](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p2-postdecoder-motion-overfit64-w1-head5x-e12-rtx5060ti-260923-105213)
- [P2 gate 50% exact64](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p2-postdecoder-motion-gate50-overfit64-w1-head5x-e12-rtx5060ti-260923-105728)
- [P2 gate 50% hardcase V2](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p2-postdecoder-motion-gate50-hardcase-v2-w1-head5x-e4-rtx5060ti-260923-110213)
- [P2 gate 90% exact64](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p2-postdecoder-motion-gate90-overfit64-w1-head5x-e12-rtx5060ti-260923-111124)
- [P2 gate 90% hardcase V2](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p2-postdecoder-motion-gate90-hardcase-v2-w1-head5x-e4-rtx5060ti-260923-111634)
- [P2 gate 99% exact64](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p2-postdecoder-motion-gate99-overfit64-w1-head5x-e8-rtx5060ti-260923-112539)

## 판정과 다음 단계

P2는 Q1에서 처음으로 실용 gate에 근접한 구조다. 단, `1.070 m/s`를 반올림해 합격으로
취급하지 않는다. 다음 순서로 진행한다.

1. gate 90% E3를 **연구 후보**로 고정한다.
2. memory-active query와 첫 frame/fallback query를 분리해 speed MAE와 adjacent-frame
   error-step p95를 진단한다.
3. 기존 `highway1`, `highway_merge`는 개발 bag으로만 사용해 detector velocity, jitter,
   latency를 확인한다.
4. 그 결과가 P1/T1보다 개선될 때만 tracker velocity mode 1/2와 Map-CV를 별도 A/B한다.
5. 새 다중차량 held-out 고주로 bag을 마지막에 한 번 평가한다.

현재 남은 차이는 더 큰 temporal network보다 association coverage, 첫 frame fallback,
불규칙 delta-time과 center jitter를 분리해 줄이는 것이 우선이다.
