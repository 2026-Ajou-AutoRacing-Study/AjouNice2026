# GNSS 음영·재획득 반복 replay와 smooth-state 안전 reset

상태: **EVIDENCE / HOLD**
검증일: 2026-09-25

## 목적

최신 raw Competition bag에서 GPS만 선택 구간 동안 제거해 같은 입력으로 다음 과정을
반복 검증했다.

```text
정상 GNSS -> 8초 음영 -> IMU + Competition heading/vx dead reckoning
-> GNSS 재획득 -> Global correction -> controller smooth state
```

원본은 `260822_005713_competition_full_route_raw_hz_run01.bag`이며 replay는 1×다.
Ego, Competition, GPS, IMU와 static TF만 남겨 detector/GPU 부하를 제거했다.

## 발견한 안전 문제와 수정

`tau=0.5 s` smooth state는 Global의 한 프레임 correction을 작은 step으로 바꿨지만,
planning trajectory는 즉시 Global map pose로 갱신된다. 음영 drift가 크면 trajectory와
controller state가 최대 1초 이상 서로 다른 map 위치를 사용했다.

기존 discontinuity 판정도 잘못된 좌표에서 이뤄졌다. `map -> odom` alignment x/y는
odom 원점에서 멀수록 작은 yaw 변화만으로 수 m 변할 수 있다. alignment parameter
차이를 차량 순간이동으로 간주하면 정상 회전을 순간이동으로 오인한다.

수정은 현재 Local pose에 기존/목표 alignment를 각각 합성하고, 두 **차량 map pose**
사이 거리가 `0.5 m`를 넘을 때만 즉시 reset한다. 그 이하는 기존 `tau=0.5 s`
smoothing을 유지하며 yaw 불연속 기준 `20 deg`도 유지한다.

## 결과

단위는 횡 오차 cm다. “복귀 step”은 재획득 주변 단일 frame 횡 오차 변화의 최대값이다.

| 8초 음영 구간 | Global 음영 max | Global 복귀 step | Smooth 수정 전 복귀 0~1s p95 | Smooth 수정 후 복귀 0~1s p95 | 수정 후 복귀 step |
|---|---:|---:|---:|---:|---:|
| 첫 회전, drift 약 0.53 m | 53.1 | 53.7 | 41.9 | **1.33** | 48.2 |
| 두 번째 회전, drift 약 1.38 m | 137.7 | 136.8 | 107.0 | **2.9** | 121.8 |

두 경우 모두 reset은 재획득 시점에 정확히 1회 발생했다. 정상 52초 replay에서는
reset 0회, control state `49.92 Hz`, 횡 오차 p95 `5.40 -> 4.84 cm`, 횡 jump p95
`1.30 -> 0.18 cm`를 유지했다.

TF co-time 수정도 함께 적용된 replay에서 두 음영 결과의 final/control state는 약
`49.98 Hz`, TF closure position p95는 `0.11~0.13 cm`였다.

## 해석

큰 재획득 correction 자체를 없앤 것은 아니다. drift가 차선 폭에 의미 있는 수준이면
오래된 controller state를 부드럽게 유지하는 것보다 planning과 즉시 같은 map pose로
재정렬하는 편이 안전하다. 작은 GNSS quantization correction은 계속 smoothing한다.

이 threshold는 실제 GPS 음영 mission과 checkpoint 이동 폐루프를 아직 통과하지
않았으므로 production controller 기본 입력을 smooth state로 승격하는 근거는 아니다.
다만 후보를 켰을 때 생기던 가장 큰 cross-frame failure mode는 제거했다.

## 재현

```bash
LOCALIZATION_OUTAGE_OUTPUT_DIR=/tmp/localization_outage \
LOCALIZATION_OUTAGE_TAU_SEC=0.5 \
scripts/evaluation/localization/run_gnss_outage_reacquisition.sh \
  rosbags/raw_sensor_w_cam/competition_full_run/\
260822_005713_competition_full_route_raw_hz_run01.bag \
  15 8 turning_8s
```

각 결과의 `metadata.txt`, `analysis.json`, `outage_analysis.json`을 함께 보관한다.

