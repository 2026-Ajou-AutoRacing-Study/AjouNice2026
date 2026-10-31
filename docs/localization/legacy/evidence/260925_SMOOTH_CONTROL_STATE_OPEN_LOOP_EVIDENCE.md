# Smooth controller state open-loop controller evidence

상태: **EVIDENCE / CLOSED-LOOP GATE REQUIRED**
검증일: 2026-09-25

## 결론

동일한 기록 trajectory와 동일한 Global/Local state를 재생하고 controller 입력만
기존 Global state와 `tau=0.5 s` smooth state로 바꿔 pure-pursuit/longitudinal PI를
다시 실행했다. 두 provenance가 다른 bag 모두에서 steering-rate와 sign flip이
감소했고, 장주행에서는 controller lateral error p50/p95도 함께 개선됐다.

따라서 smooth state를 새 MR 폐루프의 단일 1순위 후보로 확정한다. 차량 반응과
planning 재계산이 없는 open-loop이므로 production 기본값은 아직 바꾸지 않는다.

후속 quantization-aware state도 동일 controller 계약으로 비교했다. Global보다는
일관되게 개선됐지만 두 bag 전체 균형에서 Smooth 0.5초를 넘지 못했다. 따라서
폐루프 1순위는 Smooth 0.5초, quantized는 2순위 진단 후보로 유지한다.

추가 tau sweep 결과 전체 균형은 `0.5 s`가 가장 좋았다. `0.2 s`는 장주행
steering-rate p95가 더 낮았지만 sign flip과 lateral error 개선이 작았고, `1.0 s`는
짧은 bag에서 steering-rate 이득을 거의 잃었다. 따라서 `0.5 s`를 1순위로 유지하고
폐루프에서 correction 지연이 관찰될 때만 `0.2 s`를 fallback 후보로 사용한다.

## 실험 격리

두 실행에서 같은 항목:

- 기록된 `/planning/trajectory`.
- 기록된 `/localization/kinematic_state`와 `/localization/odometry/local`.
- trajectory follower config와 vehicle config.
- 1× sim-time replay.

유일한 차이:

- baseline controller: `/localization/kinematic_state`.
- candidate controller: `/localization/control_kinematic_state`.

Planning과 TF는 재실행하지 않았다. 따라서 결과는 controller 입력 연속성 효과만
분리하며 후보가 planning 경로를 바꾸는 효과를 주장하지 않는다.

## 결과

### 52초 짧은 반복 bag

| 지표 | Global | Smooth 0.5 s | 변화 |
|---|---:|---:|---:|
| steering abs p95 | 0.2980 rad | 0.2961 rad | -0.7% |
| steering-rate abs p95 | 0.3829 rad/s | 0.3685 rad/s | **-3.8%** |
| steering sign flip | 0.3990 /s | 0.3376 /s | **-15.4%** |
| controller lateral error p50 | 3.91 cm | **2.71 cm** | **-30.8%** |
| controller lateral error p95 | **42.05 cm** | 43.24 cm | +2.8% |

### 242초 production 장주행 bag

| 지표 | Global | Smooth 0.5 s | 변화 |
|---|---:|---:|---:|
| steering abs p95 | 0.2197 rad | 0.2160 rad | -1.7% |
| steering-rate abs p95 | 0.3331 rad/s | 0.3176 rad/s | **-4.7%** |
| steering sign flip | 0.3397 /s | 0.2830 /s | **-16.7%** |
| controller lateral error p50 | 3.95 cm | **2.68 cm** | **-32.2%** |
| controller lateral error p95 | 27.65 cm | **24.14 cm** | **-12.7%** |

두 실행 모두 state 약 `50 Hz`, trajectory 약 `10 Hz`를 유지했고 controller log에
WARN/ERROR/FATAL이 없었다. 장주행 마지막 GPS 음영은 복귀 없이 종료되므로 재획득
근거로 사용하지 않는다.

## Tau 민감도

짧은 bag에서 `0.2/0.5/1.0 s`를 같은 방식으로 비교했다.

| tau | steering-rate p95 | sign flip | lateral error p50/p95 |
|---:|---:|---:|---:|
| Global | 0.3828 rad/s | 0.3990 /s | 3.91 / 42.17 cm |
| 0.2 s | **0.3704 rad/s** | 0.3376 /s | 3.12 / **42.70 cm** |
| 0.5 s | 0.3729 rad/s | **0.3376 /s** | 2.69 / 43.24 cm |
| 1.0 s | 0.3824 rad/s | 0.3377 /s | **2.42** / 43.19 cm |

장주행에서 `0.2 s`도 추가 확인했다.

| tau | steering-rate p95 | sign flip | lateral error p50/p95 |
|---:|---:|---:|---:|
| Global | 0.3331 rad/s | 0.3397 /s | 3.95 / 27.65 cm |
| 0.2 s | **0.3129 rad/s** | 0.3019 /s | 3.21 / 25.88 cm |
| 0.5 s | 0.3176 rad/s | **0.2830 /s** | **2.68 / 24.14 cm** |

ROS timer 기반 open-loop 재실행 간 p95에는 소폭 변동이 있어 근소한 순위 자체를
과해석하지 않는다. 0.5초는 sign flip과 lateral error에서 일관되게 더 강하고,
0.2초는 correction 지연 위험이 작은 보조 후보다.

## Quantization-aware state 비교

UTM GNSS cell 축 `-1.347 deg`를 적용한 후보를 같은 recorded trajectory와 controller로
실행했다.

### 52초 짧은 반복 bag

| 지표 | Global | Smooth 0.5 s | Quantized |
|---|---:|---:|---:|
| steering abs p95 | 0.2980 | **0.2961** | 0.2971 rad |
| steering-rate abs p95 | 0.3829 | **0.3728** | 0.3787 rad/s |
| steering sign flip | 0.3990 | 0.3376 | **0.3376 /s** |
| controller lateral error p50 | 3.91 | 2.70 | **2.59 cm** |
| controller lateral error p95 | **42.05** | 43.24 | 42.84 cm |

### 242초 production 장주행 bag

| 지표 | Global | Smooth 0.5 s | Quantized |
|---|---:|---:|---:|
| steering abs p95 | 0.2197 | **0.2160** | 0.2207 rad |
| steering-rate abs p95 | 0.3331 | 0.3176 | **0.3171 rad/s** |
| steering sign flip | 0.3397 | **0.2830** | 0.2925 /s |
| controller lateral error p50 | 3.95 | **2.68** | 2.84 cm |
| controller lateral error p95 | 27.65 | **24.14** | 25.08 cm |

Quantized output은 두 bag 모두 50.00 Hz였다. 장주행에서는 steering-rate가 Smooth와
사실상 같았지만 sign flip과 lateral error는 Smooth가 더 좋았다. GNSS cell 경계에서
최소 correction을 즉시 적용하는 특성이 pose jump는 강하게 줄여도 controller의
lookahead/deadband 이후 출력에는 추가 이득으로 모두 이어지지 않는 것으로 해석한다.

## 다음 승인 gate

1. 같은 route에서 baseline과 smooth의 실제 폐루프 주행.
2. 좌회전·로터리에서 phase lag와 차선 오차 확인.
3. GPS 음영 진입·유지·재획득 완주.
4. checkpoint 순간이동에서 `5 m / 20 deg` 불연속 reset 확인.
5. LAN-to-LAN 동일 controller 재현.

재현:

```bash
scripts/evaluation/localization/run_controller_state_open_loop_ab.sh INPUT.bag
```

기본은 Global과 `0.5 s`를 비교한다. tau sweep은
`LOCALIZATION_CONTROL_AB_TAUS='0.2 0.5 1.0'`으로 실행할 수 있다.

새 폐루프 recorder는 Global과 control state를 동시에 담고 adjacent metadata 파일에
commit, branch, 실제 controller input topic과 runtime parameter를 저장한다. 안정성
평가기 한 번으로 두 pose와 실제 controller chatter를 함께 보고한다.
