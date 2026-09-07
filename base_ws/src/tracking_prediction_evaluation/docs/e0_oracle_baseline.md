# E0 Oracle prediction baseline

## 결론

새 causal E0 평가에서 **CTRV가 production kinematic baseline**이다. CTRA는 세
장면 모두에서 평균 FDE가 더 높고, horizon이 길수록 열화가 커졌다. 과거 문서의
“CTRA median 우세”는 현재 기준으로 재현된 결론이 아니다.

E0는 GT의 현재 pose와 과거 궤적을 사용하므로 detector와 tracker 오차를 포함하지
않는다. 따라서 아래 값은 runtime 전체 성능이 아니라 motion model의 상한이다.

## 평가 조건

- GT: MORAI `/Object_topic`
- state history: 과거 0.5초만 사용한 causal pose fit
- evaluation origins: 10 Hz
- trajectory interval: 0.1초
- horizons: 0.5, 1.0, 1.5, 2.0, 3.0초
- metrics: FDE/ADE mean, RMSE, p50, p90, p95, MR@1/2/3m
- actor identity: position prediction + Hungarian matching으로 재구성
- future interpolation gap: 최대 0.15초

`ObjectStatus.velocity`는 bag에 따라 unit이 달라 직접 사용하지 않는다. MORAI
`unique_id` 역시 물리 actor 사이에서 재사용되므로 GT track ID로 사용하지 않는다.

## Vehicle FDE 결과

### Roundabout 1

| Horizon | CTRV mean / p50 / p90 / p95 (m) | CTRA mean / p50 / p90 / p95 (m) |
|---:|---:|---:|
| 0.5 s | 0.136 / 0.117 / 0.215 / 0.260 | 0.145 / 0.126 / 0.245 / 0.291 |
| 1.0 s | 0.350 / 0.268 / 0.786 / 0.945 | 0.396 / 0.333 / 0.708 / 0.886 |
| 1.5 s | 0.665 / 0.456 / 1.752 / 2.081 | 0.818 / 0.684 / 1.521 / 1.860 |
| 2.0 s | 1.089 / 0.676 / 3.139 / 3.618 | 1.442 / 1.249 / 2.665 / 3.259 |
| 3.0 s | 2.245 / 1.263 / 6.795 / 7.847 | 3.337 / 2.752 / 6.504 / 7.399 |

### Roundabout 2

| Horizon | CTRV mean / p50 / p90 / p95 (m) | CTRA mean / p50 / p90 / p95 (m) |
|---:|---:|---:|
| 0.5 s | 0.139 / 0.119 / 0.224 / 0.268 | 0.149 / 0.126 / 0.264 / 0.304 |
| 1.0 s | 0.368 / 0.276 / 0.826 / 0.955 | 0.419 / 0.341 / 0.751 / 0.905 |
| 1.5 s | 0.706 / 0.485 / 1.823 / 2.137 | 0.875 / 0.712 / 1.540 / 1.916 |
| 2.0 s | 1.163 / 0.724 / 3.192 / 3.696 | 1.562 / 1.316 / 2.760 / 3.402 |
| 3.0 s | 2.392 / 1.359 / 6.825 / 7.965 | 3.677 / 3.068 / 6.734 / 7.770 |

### Highway 1

| Horizon | CTRV mean / p50 / p90 / p95 (m) | CTRA mean / p50 / p90 / p95 (m) |
|---:|---:|---:|
| 0.5 s | 0.182 / 0.179 / 0.266 / 0.304 | 0.201 / 0.185 / 0.324 / 0.425 |
| 1.0 s | 0.417 / 0.363 / 0.810 / 0.988 | 0.490 / 0.403 / 0.890 / 1.234 |
| 1.5 s | 0.718 / 0.554 / 1.621 / 2.140 | 0.894 / 0.673 / 1.786 / 2.502 |
| 2.0 s | 1.083 / 0.749 / 2.681 / 3.774 | 1.422 / 1.022 / 2.997 / 4.188 |
| 3.0 s | 1.986 / 1.163 / 5.422 / 8.380 | 2.861 / 1.971 / 6.241 / 9.043 |

Roundabout에서는 CTRA p90/p95가 일부 중장기 horizon에서 근소하게 낮지만 mean과
p50이 크게 나빠진다. Highway에서는 tail까지 전반적으로 나빠진다. 단일 production
모델을 선택할 근거로는 CTRV가 더 안정적이다.

## 과거 평가를 폐기한 이유

Git 이력에 남은 구 `tracking_prediction` 평가 결과에는 다음 문제가 확인됐다.

1. bag별로 다른 velocity unit을 모두 km/h로 가정해 3.6으로 나눴다.
2. 재사용되는 MORAI `unique_id`를 영구 physical actor ID로 사용했다.
3. 중복 timestamp 처리 방식이 일관되지 않았다.
4. 일부 smoothing은 미래 표본을 포함해 online predictor보다 유리했다.
5. 실제 detector·EKF 대신 GT state와 합성 Gaussian noise를 사용했다.

따라서 과거의 CTRA/CTRV 수치와 horizon 결정값은 현재 성능 근거로 사용하지 않는다.
혼동을 막기 위해 구 package·평가 스크립트·결과 문서는 작업트리에서 제거했으며,
필요할 때만 Git 이력으로 확인한다.

## 해석과 다음 단계

- E0에서는 tracker가 존재하지 않으므로 tracking 평가가 불가능하다.
- Highway bag의 pedestrian은 한 정적 actor에 편중되어 VRU 결론을 낼 수 없다.
- CTRV도 3초 vehicle FDE p90이 5.4~6.8m로 커진다.
- P1은 Lanelet2 v60을 사용해 curve/roundabout의 lateral drift와 3초 tail을 줄이는
  것이 핵심 가설이다.
- P1 통과 기준은 같은 E0 sample에서 CTRV 대비 paired FDE, off-road rate와
  lane-boundary violation이 개선되는 것이다.
- E1 tracking과 E2 detector-to-prediction 평가는 detector/tracker 안정화 이후 진행한다.
