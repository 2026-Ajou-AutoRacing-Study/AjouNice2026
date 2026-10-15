# Base4 Q1-P5 Shadow Association Soft Blend 결과

> 날짜: 2026-09-23.
> 상태: CURRENT 기각 실험. Production 미적용, P3 연구 baseline 유지.

## 목적

P4의 mutual matching과 robust consensus는 유효한 P3 memory pair까지 제거해 hardcase
speed MAE를 악화시켰다. P5는 P3의 attention과 direct velocity를 그대로 유지하면서
robust association을 shadow로만 계산하고, confidence가 있는 query에 한해 robust
velocity를 작게 섞는 보수적 실험이다.

## 구현

- `shadow_robust_association=true`일 때 P3의 base valid pair, attention과 active mask를
  변경하지 않는다.
- robust mutual/consensus 후보와 confidence는 별도로 계산한다.
- 최종 direct velocity는 다음 연속식으로만 보정한다.

  `v = v_p3 + alpha * confidence * (v_robust - v_p3)`

- 단일 mutual pair의 confidence는 다중-frame consensus보다 낮게 제한한다.
- robust blend가 실제 적용된 Vehicle query 비율과 평균 confidence를 W&B 및 online
  diagnostic에 추가했다.
- `alpha=0` 회귀시험에서 P3 feature output, direct velocity와 active mask가 동일함을
  CUDA container 단위시험으로 확인했다.

## 재학습 없는 통제 평가

동일 P3 hardcase E1 checkpoint를 사용해 association 변경만 비교했다. 데이터는
`morai_base4_velocity_hardcase_v2_raw16` scene-disjoint val이며, 모든 공간 지표와
checkpoint가 같다.

| P5 blend alpha | speed MAE | mAVE | mATE | Vehicle center mAP |
| ---: | ---: | ---: | ---: | ---: |
| `0.00` | **`0.8451`** | **`0.9117`** | `0.2931` | `0.9106` |
| `0.10` | `0.8472` | `0.9138` | `0.2931` | `0.9106` |
| `0.25` | `0.8508` | `0.9171` | `0.2931` | `0.9106` |

`alpha=0`은 기존 P3 수치를 정확히 재현했다. 그러나 blend를 키울수록 공간 지표는
완전히 동일한 채 velocity만 단조롭게 악화했다. `alpha=0.10`도 speed MAE가 약 `0.25%`,
`alpha=0.25`는 약 `0.67%` 나빠졌다.

## 판정

P5는 사전 hardcase 합격선 `<0.93 m/s` 자체는 넘지만 P3 대비 개선이 전혀 없고 방향도
일관되게 나쁘다. 따라서 rosbag p95와 tracker/Map-CV 평가를 열지 않는다. 작은 alpha의
차이가 수치상 작더라도, 동일 가중치 통제에서 robust 후보를 더 신뢰할수록 악화한다는
점이 핵심이다.

P4/P5를 합치면 현재의 frame별 mutual nearest, 위치 거리, pair dispersion만으로는
올바른 object identity confidence를 만들 수 없다는 결론이다. 다음 단계에서 같은 gate의
threshold 또는 blend 비율을 다시 조정하지 않는다.

1. P3를 Q1 연구 baseline으로 고정한다.
2. shadow diagnostic은 기본 비활성 연구 도구로만 보존한다.
3. 다음 association 개선은 detector query feature similarity 또는 tracker innovation처럼
   identity를 직접 구분할 정보가 있을 때만 연다.
4. 새 방식도 hardcase P3보다 좋아진 뒤에만 기존 두 개발 rosbag과 tracker A/B를 수행한다.
5. Production은 계속 E25 7D, detector velocity fusion off다.
