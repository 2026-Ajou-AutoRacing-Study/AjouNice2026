# Base4 Q1-P6 Age-weighted Motion 결과

> 날짜: 2026-09-23.
> 상태: Detector 연구 후보 통과, production tracker 통합은 보류.

## 목적

P4/P5는 잘못 연결된 memory pair를 제거하려 했지만 P3보다 나빠졌다. P6는 먼저 실제
병목이 object identity 오매칭인지, 같은 객체 query center 자체의 frame jitter인지
분리했다. 이후 짧은 `dt`의 noisy displacement가 속도를 과도하게 흔들지 않도록 여러
memory pair를 시간 간격에 따라 가중 결합했다.

## Association 진단

Hardcase V2 scene-disjoint val의 P3 E1 checkpoint를 사용했다. GT instance ID는 추론이
끝난 query pair의 정답 여부를 진단하는 데만 썼고 detector association 입력에는 넣지
않았다.

| 항목 | 결과 |
| --- | ---: |
| 평가 frame | `108` |
| association 기회 | `98` |
| 후보 수 mean / p50 / p95 | `1.010 / 1 / 1` |
| distance top-1 identity 정확도 | `100%` |
| cosine top-1 identity 정확도 | `100%` |
| hybrid top-1 identity 정확도 | `100%` |
| 동일 ID feature cosine mean | `0.970` |
| GT-ID query-pair speed MAE | `0.846 m/s` |
| GT-ID query-pair vector error p95 | `2.593 m/s` |

대부분 query에는 후보가 하나뿐이었고 위치, feature와 hybrid matching이 모두 같은 GT
identity를 골랐다. 그런데 정답 identity끼리 연결한 center displacement에도 이미
`0.846 m/s`의 speed error가 있었다. 따라서 이 split에서는 오매칭이 주 병목이 아니며,
짧은 `dt`에서 decoder center noise를 미분하는 것이 순간 속도 튐의 주 원인이다.

## 구현

P3 direct velocity의 pair attention weight에 `dt^power`를 곱한 뒤 다시 정규화한다.
P6의 기본 `power=2`는 현재 center를 trajectory intercept로 고정한 weighted least
squares slope와 같다.

`v = sum(w_i * dt_i^2 * delta_i / dt_i) / sum(w_i * dt_i^2)`

따라서 같은 attention confidence라면 오래된 유효 pair의 더 긴 displacement baseline을
더 신뢰하고, center noise에 취약한 매우 짧은 baseline의 영향은 줄인다. 옵션 기본값은
`0`이라 기존 P3 checkpoint와 동작을 보존한다.

단위시험에서는 `0.1 s` pair가 `3 m/s`, `0.5 s` pair가 `2 m/s`일 때 단순 평균이
`2.5 m/s`인 반면 age-WLS 결과가 장기 baseline의 `2 m/s`에 더 가까워짐을 확인했다.

## 재학습 없는 hardcase A/B

동일 P3 E1 checkpoint로 motion aggregation만 바꿨다.

| age power | speed MAE | mAVE | mATE | Vehicle center mAP |
| ---: | ---: | ---: | ---: | ---: |
| `0.0` | `0.8453` | `0.9119` | `0.2930` | `0.9106` |
| `0.5` | `0.8091` | `0.8736` | `0.2931` | `0.9106` |
| `1.0` | `0.7808` | `0.8430` | `0.2931` | `0.9106` |
| `2.0` | **`0.7383`** | **`0.7987`** | `0.2931` | `0.9106` |
| `3.0` | `0.7103` | `0.7712` | `0.2931` | `0.9106` |

`power=3`이 이 split에서는 더 낮았지만 일정 속도 비중이 큰 val에 과적합할 위험과
설명 가능한 estimator 계약을 고려해 `power=2`를 P6로 선택했다. P3 대비 speed MAE는
약 `12.7%` 개선됐고 공간 성능은 유지됐다.

## 개발 rosbag paired A/B

P3와 P6를 현재 코드와 동일 evaluator로 각각 새로 추론했다. threshold는 모두
Vehicle/Pedestrian/Obstacle `0.40/0.15/0.10`, query memory context on, detector velocity
on이다.

| Bag | 설정 | center mAP | F1 | speed MAE | speed bias | error-step p50 / p95 | AVE |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| highway1 | P3 | `0.8408` | `0.9635` | `1.581` | `+0.285` | `0.982 / 5.569` | `4.367` |
| highway1 | P6 WLS2 | `0.8395` | `0.9635` | **`0.872`** | **`-0.104`** | **`0.454 / 1.492`** | **`2.254`** |
| highway merge | P3 | `0.6423` | `0.7973` | `1.021` | `+0.281` | `0.689 / 3.367` | `1.863` |
| highway merge | P6 WLS2 | `0.6423` | `0.7973` | **`0.829`** | **`+0.241`** | **`0.426 / 2.511`** | **`1.530`** |

P6는 highway1 speed MAE를 약 `44.9%`, error-step p95를 약 `73.2%` 낮췄다. 합류에서도
speed MAE와 p95가 함께 개선됐다. matched 수와 F1은 동일하고 center mAP 차이는
`0.0013` 이하라 velocity post-aggregation이 공간 검출을 바꾸지 않았음을 확인했다.

paired inference latency는 다음과 같다.

| 설정 | sample | mean | p50 | p95 |
| --- | ---: | ---: | ---: | ---: |
| P3 | `873` | `88.93 ms` | `87.67 ms` | `99.00 ms` |
| P6 WLS2 | `873` | `89.43 ms` | `88.23 ms` | `101.43 ms` |

평균 증가는 약 `0.5 ms`로 작지만 현재 5060 Ti 한-PC 조건에서는 12 Hz mean latency
gate `83 ms`를 통과하지 못한다. 최종 4090/분리 MORAI 환경에서 다시 측정해야 한다.

## Tracker와 Map-CV A/B

P6 detection을 동일 tracker와 prediction pipeline에 넣어 두 개발 bag의 equal-weight
macro를 비교했다.

| Tracker 입력 | F1 | center p50 | speed MAE | AVE | stable minFDE-K 1/2/3s |
| --- | ---: | ---: | ---: | ---: | ---: |
| detector init only | `0.9263` | `0.476` | `0.777` | `0.910` | `0.919 / 1.442 / 2.017` |
| detector update, noise 1 | `0.8795` | `0.567` | `0.742` | `1.463` | `1.009 / 1.736 / 2.527` |
| position only | **`0.9329`** | **`0.435`** | **`0.671`** | **`0.821`** | **`0.810 / 1.237 / 1.708`** |

Detector velocity 자체는 크게 좋아졌지만 매 frame fusion은 center, F1, lateral error,
AVE와 stable prediction을 악화했다. init-only도 position-only를 넘지 못했다. 현재 tracker가
position history로 만든 속도가 P6 detector보다 여전히 정확하고 안정적이다.

## 판정과 다음 단계

1. P6는 Q1 detector 연구 후보로 채택한다. P3의 identity association threshold를 더
   튜닝하지 않는다.
2. Production은 계속 E25와 position-only tracker를 유지한다. detector velocity fusion은
   기본 off다.
3. P6를 production으로 승격하려면 새 held-out 고주로 bag과 4090 latency 평가가 필요하다.
4. 다음 tracker 연구는 detector velocity를 매 frame 강제하지 않고 innovation, track age,
   uncertainty로 선별하는 조건부 update만 검토한다.
5. 새 teammate bag은 업로드가 복구되면 tuning에 쓰지 않고 우선 held-out 평가군으로
   보존한다.
