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
