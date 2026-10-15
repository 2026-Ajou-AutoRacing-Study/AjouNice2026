# Smooth controller state 전체 경로 폐루프 A/B

상태: **EVIDENCE / PRODUCTION ACCEPTED**  
검증일: 2026-09-25

## 결론

동일한 actor-free 전체 경로에서 controller 입력만 바꾼 폐루프 A/B 결과,
`translation/yaw tau=0.5 s` Smooth state가 완주 속도 손실 없이 횡 tracking 오차와
조향 채터링을 줄였다. Planning과 TF는 기존 Global state를 유지하고 controller만
`/localization/control_kinematic_state`를 사용하는 구성을 1차 production으로
승격한다.

## 비교 계약

- commit: `8889293f73f182c26457bb0838b142ee15b7b8ee`
- launcher: MR, `gt_source=competition`, heading correction `-1.347 deg`
- Q/R: GNSS sigma `0.10 m`, Global Qxy `0.0005`
- scenario: `localization_full_route_no_actors`
- 공통: 같은 planning/control, `od_gt`, 신호등·객체 없음
- Smooth controller: `/localization/control_kinematic_state`, tau `0.5/0.5 s`
- baseline controller: `/localization/kinematic_state`
- 두 주행 모두 약 `7.25 s` GNSS 음영과 재획득 포함

평가 bag:

- `260925_194856_smooth_tau0p5_full_route_no_actors.bag`
- `260925_200740_global_full_route_no_actors.bag`

두 bag 모두 localization 약 `50 Hz`, 이동 중 controller-valid `100%`였다.

## 폐루프 결과

| 지표 | Smooth | Global baseline | 변화 |
|---|---:|---:|---:|
| controller 횡오차 p50 | **5.20 cm** | 5.36 cm | **-2.8%** |
| controller 횡오차 p95 | **39.75 cm** | 42.52 cm | **-6.5%** |
| 조향 변화율 평균 | **0.0444 rad/s** | 0.0525 rad/s | **-15.4%** |
| 조향 변화율 p95 | **0.259 rad/s** | 0.284 rad/s | **-8.7%** |
| 조향 변화율 p99 | **0.348 rad/s** | 0.396 rad/s | **-12.2%** |
| 좌우 조향 반전 빈도 | **0.172 /s** | 0.225 /s | **-23.8%** |
| 조향 크기 p95 | 0.247 rad | 0.246 rad | 동등 |

GNSS 재획득 직후 첫 1초에는 횡오차 p95가 `63.03 -> 37.82 cm`, 조향 변화율
p95가 `0.698 -> 0.221 rad/s`로 감소했다. 이는 각각 `40.0%`, `68.3%` 개선이다.

## Controller-facing localization 정확도

속도 `1 m/s` 이상 전체 이동 구간이며 GNSS 음영을 포함한다.

| 지표 | Smooth mean / p50 / p95 | Global mean / p50 / p95 |
|---|---:|---:|
| 평면 XY 오차 | **2.55 / 1.66 / 6.19 cm** | 3.41 / 2.44 / 7.25 cm |
| 종방향 오차 | **1.16 / 0.88 / 2.98 cm** | 1.28 / 0.95 / 3.58 cm |
| 횡방향 오차 | **1.87 / 0.88 / 5.14 cm** | 2.81 / 1.77 / 6.72 cm |
| yaw 오차 | 0.007 / 0.003 / 0.034 deg | **0.005 / 0.002 / 0.025 deg** |

첫 Smooth bag 내부에서 변경되지 않은 Global state와 Smooth state를 직접 비교해도
XY mean/p50/p95는 `3.22/2.38/7.17 -> 2.55/1.66/6.19 cm`로 감소했다. 따라서
두 반복 주행의 우연한 Global 차이만으로 설명되는 결과가 아니다.

## 완주 시간

Global bag은 원점 약 `11.37 m` 전에 기록이 끝나 정확한 원점 crossing을 비교할 수
없다. 양쪽 모두 기록된 시작점 반경 `15 m`의 outbound/inbound gate를 사용했다.

| 지표 | Smooth | Global baseline |
|---|---:|---:|
| gate-to-gate 시간 | 149.755 s | 149.339 s |
| 경로 길이 | 2155.99 m | 2155.50 m |
| 시간 가중 평균속도 | 14.396 m/s | 14.433 m/s |

시간 차이는 `0.416 s`, 약 `0.28%`로 의미 있는 속도 손실이 아니다.

## Production 계약과 rollback

- Planning/TF: `/localization/kinematic_state` 유지.
- Controller: `/localization/control_kinematic_state`.
- Smooth: translation/yaw tau `0.5/0.5 s`.
- quantization-aware 후보와 relocation active reset은 기본 OFF 유지.

문제가 생기면 localization은 그대로 둔 채 controller만 다음으로 되돌린다.

```bash
traj_fol_global
```

후속 `yaw tau=0`, Competition yaw-rate, ZUPT 실험은 이 production 승격과 분리한다.
