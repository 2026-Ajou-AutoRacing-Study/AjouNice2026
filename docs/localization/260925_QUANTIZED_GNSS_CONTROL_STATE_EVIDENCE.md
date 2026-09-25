# Quantized-GNSS controller state evidence

상태: **EVIDENCE / EXPERIMENTAL / DEFAULT OFF**  
검증일: 2026-09-25

## 결론

GNSS를 정확한 점이 아니라 양자화 cell 내부의 interval measurement로 취급하는
controller 전용 state 후보를 구현했다. 두 production Q/R bag에서 Global EKF의
종방향 위치와 yaw를 그대로 유지하면서 횡 frame jump p95를 약 `80~83%` 줄였다.

그러나 기록된 입력에 대한 counterfactual/ROS replay 결과이며 실제 controller
폐루프가 아니다. 따라서 다음 상태를 유지한다.

- Planning과 TF: 기존 Global EKF state.
- Production controller: 기존 `/localization/kinematic_state`.
- 후보 topic: `/localization/control_kinematic_state`.
- `publish_control_state=false` 기본값 유지.
- `use_quantized_gnss_control_state=false` 기본값 유지.

## 알고리즘

MORAI GNSS 한 cell은 K-City에서 대략 다음 크기다.

| 축 | cell width | 후보 half-width (`scale=1.25`) |
|---|---:|---:|
| UTM east | `14.8 cm` | `9.25 cm` |
| UTM north | `18.5 cm` | `11.56 cm` |

세 raw bag의 위·경도 격자를 UTM 52N으로 직접 투영한 결과 cell 축은 map x/y 대비
약 `-1.348 deg` 회전한다. 후보는 `grid_yaw=-1.347 deg`를 사용한다. 폭과 회전각의
근거는 `260925_GNSS_QUANTIZATION_LATTICE_AUDIT.md`에 있다.

Local EKF pose를 현재 map→odom alignment로 map에 투영한다.

1. 예측 위치가 최신 GNSS cell 안이면 translation correction을 하지 않는다.
2. cell 밖이면 가장 가까운 경계까지만 최소 correction을 적용한다.
3. yaw는 Global EKF와 동일하게 유지한다.
4. 최종 출력은 후보의 횡 displacement만 사용하고, 차량 heading 방향 종 위치는
   Global EKF를 그대로 보존한다.
5. GNSS가 `0.2 s` 이상 stale이면 과거 cell은 무효화하고 Global과 동일하게
   fallback한다.
6. 재획득 후 `1.0 s`는 Global settling을 그대로 따르고 이후 cell correction을
   재개한다.
7. GNSS와 Global의 거리가 `0.5 m`보다 크면 quantization correction을 생략한다.
8. 큰 Global discontinuity는 기존 reset threshold로 alignment를 재초기화한다.

새 node는 TF를 발행하지 않으며 planning state도 바꾸지 않는다.

## 평가 계약

- MR heading oracle offset: `-1.347 deg`.
- production Q/R: GNSS sigma `0.10 m`, Global Qxy `0.0005`.
- 입력: 기록된 Global state, Local odometry, GNSS odometry.
- 출력: controller 후보만 counterfactual 생성.
- 단위: 아래 위치·jump는 `cm`, yaw는 `deg`.
- 4× replay는 callback 위상 때문에 출력이 약 18 Hz로 떨어져 폐기했다.
- 유효 transport 평가는 반드시 1× sim-time replay로 수행했다.

## 52초 반복 bag, 실제 ROS 1× replay

bag: `260920_003403_q0005_r010_gt_object_lightweight_no_traffic_v2_test.bag`

| state | 횡 오차 p50 / p95 | 종 오차 p50 / p95 | 횡 jump p50 / p95 | 종 jump p50 / p95 | yaw p50 / p95 | rate |
|---|---:|---:|---:|---:|---:|---:|
| Global production | 1.503 / 5.417 | 1.443 / 5.356 | 0.173 / 1.263 | 1.718 / 6.990 | 0.0049 / 0.0666 | 49.997 Hz |
| Quantized lateral blend | **1.140 / 4.746** | **1.443 / 5.356** | **0.023 / 0.213** | **1.719 / 6.990** | **0.0049 / 0.0666** | 49.998 Hz |

횡 오차 p95는 `12.4%`, 횡 jump p95는 `83.1%` 감소했다. 종·yaw는 baseline과
수치 정밀도 수준에서 같다.

## 242초 장주행 bag, 실제 ROS 1× + 동일 blending

bag: `260919_231906_q0005_r010_gt_object_lightweight_no_traffic.bag`

| state | 횡 오차 p50 / p95 | 종 오차 p50 / p95 | 횡 jump p50 / p95 | 종 jump p50 / p95 | yaw p50 / p95 | rate |
|---|---:|---:|---:|---:|---:|---:|
| Global production | 1.786 / 5.562 | 1.769 / 6.282 | 0.208 / 1.307 | 1.881 / 7.528 | 0.0071 / 0.0671 | 49.999 Hz |
| Quantized lateral blend | **0.994 / 3.785** | **1.769 / 6.283** | **0.034 / 0.259** | **1.910 / 7.528** | **0.0071 / 0.0671** | 49.999 Hz |

횡 오차 p95는 `31.9%`, 횡 jump p95는 `80.2%` 감소했다. bag 끝의 GNSS 중단
구간에서도 마지막 alignment를 유지하며 output rate가 끊기지 않았다. 다만 이 구간은
차량이 정지하며 GNSS 재획득이 없어 recovery 승격 근거는 아니다.

## 8초 GNSS 음영·재획득 replay

원본 raw Competition bag의 GPS만 선택 구간에서 제거하고 Local/Global EKF부터 다시
1× 실행했다. 구형 bag의 큰 GNSS innovation에서 cell correction과 Global reset이
경합해 reset이 연속 7회 발생하는 prototype 결함을 먼저 확인했다. 다음 안전장치를
추가한 뒤 같은 입력을 다시 실행했다.

- GNSS–Global innovation `0.5 m` gate.
- stale GNSS 즉시 Global fallback.
- 재획득 `1.0 s` grace period.

단위는 횡 오차와 step 모두 `cm`다.

| 8초 음영 구간 | state | 음영 횡 p95 / max | 복귀 0~1s 횡 p95 | 복귀 1~4s 횡 p95 | 복귀 step p95 / max | rate |
|---|---|---:|---:|---:|---:|---:|
| offset 15 s 회전 | Global | 41.58 / 53.99 | 3.31 | 10.78 | 1.68 / 54.46 | 49.97 Hz |
| offset 15 s 회전 | 후보 | **41.58 / 53.99** | **3.31** | **4.74** | **1.68 / 54.46** | 49.97 Hz |
| offset 83 s, 약 89° 회전 | Global | 52.83 / 53.29 | 6.96 | 8.87 | 1.65 / 58.26 | 49.99 Hz |
| offset 83 s, 약 89° 회전 | 후보 | **52.83 / 53.29** | **6.96** | **2.28** | 1.68 / **58.26** | 49.99 Hz |

후보는 stale/recovery 초기에 Global보다 나빠지지 않고, grace 이후 정상 cell update가
돌아오면 횡 오차를 다시 줄였다. 재획득 때 Global 자체가 만드는 큰 단일 correction은
그대로 따른다. Planning과 controller의 map pose가 장시간 분리되는 것보다 안전한
정책이다. 연속 reset loop는 제거됐고 각 재획득에서 한 번만 재정렬됐다.

## 구현 중 제거한 결함

초기 prototype은 map→odom yaw를 odom 원점 기준으로 변경해 멀리 있는 차량 pose까지
움직였다. yaw 변경 전후 현재 차량 XY가 같도록 alignment translation을 재계산하고,
이 후보의 yaw smoothing은 `0 s`로 분리했다. 수정 전 결과는 폐기했다.

## 다음 gate

1. 동일 trajectory controller open-loop는 통과했으나 Smooth 0.5초보다 종합 우위가
   없었다. `od_gt` 경량 폐루프 1순위는 Smooth, quantized는 2순위로 비교한다.
2. steering-rate p95, sign flip, lateral control error와 lane departure 비교.
3. GPS 음영 replay는 통과했으며, 실제 GPS 음영 폐루프 비교.
4. checkpoint relocation/reset과 후보 alignment 재초기화 확인. **통과:** 합성
   20 m 이동에서 guard reset 1회, reset 순간 Global–control 차이 0.0 m, 두 state
   모두 49.997 Hz였다. 상세는 체크포인트 guard EVIDENCE를 따른다.
5. LAN-to-LAN 동일 controller 반복.

위 gate 전에는 production alias나 controller 기본 topic을 변경하지 않는다.
