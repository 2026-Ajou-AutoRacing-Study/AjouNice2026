# Base4 Q1-P4 Robust Association 결과

> 날짜: 2026-09-23.
> 상태: CURRENT 기각 실험. Production 미적용, P3 유지.

## 목적

P3는 개발 highway1 rosbag에서 speed MAE를 `1.582 m/s`까지 낮췄지만 adjacent
error-step p95가 `5.569 m/s`였다. P4는 잘못 연결된 query-memory pair를 제거해 평균
성능을 유지하면서 순간 velocity outlier를 줄이는 실험이다.

## 구현 및 진단

- memory frame별 현재/과거 Vehicle query mutual nearest matching.
- 둘 이상의 memory pair에는 component-wise median 기준 `3.0 m/s` consensus 적용.
- 엄격 P4는 최소 2개 frame의 합의가 없으면 single-frame head로 fallback.
- P4b는 coverage 회복을 위해 mutual 단일 pair도 추정 speed `<=35 m/s`이면 허용.
- association distance, pair dispersion, switch, mutual/consensus/fallback/single-pair 비율을
  train W&B와 online JSONL에 기록할 수 있게 했다.
- online 응답과 ROS bridge에는 진단이 명시적으로 요청된 경우에만 추가되며 기존
  production 기본 동작은 바뀌지 않는다.

CUDA container 단위시험은 다음을 통과했다.

- pair velocity `1, 1, 10 m/s`에서 `10 m/s` outlier 제거.
- 합의되지 않은 `1, 10 m/s` pair fallback.
- 단일 mutual pair `10 m/s` 허용, `40 m/s` physical-cap 거부.

## 실험 결과

### 엄격 P4: 최소 2-frame consensus

P3 checkpoint에서 시작했지만 exact64 E2 speed MAE가 `1.998 m/s`였다. P3 exact64
`0.931 m/s`보다 크게 나빠져 기각했다. 학습 중 Vehicle fallback 비율이 흔히
`0.66~1.00`으로 나타나 유효한 단일 memory pair까지 제거한 것이 직접 원인이었다.

### P4b: single mutual pair + 35 m/s cap

| 평가 | Epoch | speed MAE | mAVE | mATE | Vehicle center mAP |
| --- | ---: | ---: | ---: | ---: | ---: |
| exact64 | 2 | `0.974` | `1.013` | - | - |
| hardcase V2 | 1 | `1.274` | `1.340` | `0.3232` | `0.8841` |
| hardcase V2 | 2 | `1.388` | `1.466` | `0.2712` | `0.8901` |

exact64는 P3 `0.931` 대비 `+4.6%`로 사전 gate `+10% 이내`를 통과했다. 그러나
scene-disjoint hardcase best는 P3 E1 `0.845 → 1.274 m/s`로 `50.8%` 악화했고 공간
mAP도 `0.9106 → 0.8841`로 낮아졌다.

추가 학습의 영향을 분리하기 위해 P3 E1 checkpoint를 P4b config로 재학습 없이 직접
평가했다.

| Config / weight | speed MAE | mAVE | mATE | Vehicle center mAP |
| --- | ---: | ---: | ---: | ---: |
| P3 / P3 E1 | `0.845` | `0.912` | `0.2931` | `0.9106` |
| P4b / 동일 P3 E1 | `1.419` | `1.474` | `0.2931` | `0.9106` |

동일 가중치에서 공간 지표는 정확히 유지되지만 velocity만 `67.9%` 악화했다. 따라서
주원인은 LR 재시작이나 추가 학습이 아니라 P4 association gate 자체다. 현재 query는
track identity가 아니므로 frame별 mutual nearest가 올바른 motion pair를 보장하지 못하며,
hard fallback은 P3의 유용한 다중 후보 평균을 지나치게 제거한다.

W&B:

- [P4b exact64 E2](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p4-singlecap35-gate90-overfit64-w1-head5x-e2-rtx5060ti-260923-155555)
- [P4b hardcase V2 E2](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base4-q1-p4-singlecap35-gate90-hardcase-v2-w1-head5x-e2-rtx5060ti-260923-160125)

첫 hardcase 실행은 checkpoint 저장 시 디스크 부족으로 중단됐으며, 실패 run과 앞선 P4
실패 산출물만 제거한 뒤 동일 조건으로 다시 실행했다.

## 판정

P4는 hardcase gate를 통과하지 못했으므로 계획한 highway1/merge rosbag 재평가와 tracker
A/B를 열지 않는다. Production은 계속 E25 7D, Q1 연구 baseline은 P3 E1이다.

P4 코드는 기본 비활성인 연구·진단 도구로 남긴다. 다음 시도에서 hard fallback을 다시
조정하는 것보다 다음 순서가 우선이다.

1. P3 출력을 유지한 채 association 진단을 shadow mode로 실제 rosbag에서 수집한다.
2. per-query identity가 없는 nearest matching 대신 tracker innovation 또는 feature
   similarity를 confidence로 사용한다.
3. velocity를 교체하지 않고 confidence에 따라 P3와 single-frame/tracker를 연속적으로
   blend한다.
4. hardcase `<0.93 m/s`를 먼저 통과한 뒤에만 동일 rosbag p95와 tracker/Map-CV를 평가한다.
