# Localization experiment template

상태: **TEMPLATE**

새 실험 문서는 이 파일을 복사해 `YYMMDD_<topic>_evidence.md`로 만든다.

## 결론

- 한 문단으로 채택/기각/보류를 먼저 쓴다.

## 가설과 단일 변경 변수

- 가설:
- baseline:
- 변경 변수:
- 변경하지 않은 항목:

## 재현 정보

| 항목 | 값 |
|---|---|
| branch / commit | |
| bag path | |
| acquisition | `MR / MRP / LAN_FIELD / OFFLINE_REPLAY` |
| alias | |
| `gt_source` | `competition / ego` |
| estimator heading correction | |
| evaluator heading offset | |
| time policy | |
| real-time factor | |
| localization provider count | `1` |
| controller revision | |

## 입력·출력 계약

- 입력 topic과 rate:
- output topic과 rate:
- TF owner:
- GPS 음영 여부와 구간:

## 결과

| moving metric | baseline p50 | baseline p95 | candidate p50 | candidate p95 | 변화 |
|---|---:|---:|---:|---:|---:|
| lateral error | | | | | |
| longitudinal error | | | | | |
| lateral frame jump | | | | | |
| longitudinal frame jump | | | | | |
| yaw error | | | | | |
| steering-rate | | | | | |

- steering sign flips/sec:
- localization rate/gap:
- NaN, duplicate, out-of-order:
- GPS reacquisition max step/time:

## 판정

- `ADOPT / REJECT / HOLD`
- 근거:
- production 변경 여부:
- rollback:

## 한계와 다음 gate

- route/controller/runtime 차이:
- 아직 확인하지 못한 항목:
- 다음 실험:
