# Controller용 smooth global/local state 사전 검증

상태: **EVIDENCE / HOLD**
검증일: 2026-09-25

## 결론

Global EKF의 전역 정확도와 Local EKF의 단기 연속성을 분리해 쓰는 구조는 현재
rosbag 세 종류 모두에서 횡방향 frame jump를 크게 줄였다. `0.5 s` 보정 시상수는
production 장주행에서 횡 오차 p95도 `5.56 -> 3.94 cm`로 줄이면서 횡 jump p95를
`1.31 -> 0.20 cm`로 줄였다. 다만 이것은 기록된 출력에 대한 counterfactual이며
controller 폐루프 결과가 아니므로 production 기본값은 바꾸지 않는다.

## 가설과 구조

기존 controller는 GNSS correction이 포함된 Global EKF pose를 직접 소비한다. 후보는
다음처럼 차량의 빠른 움직임 자체가 아니라 `map -> odom` 정합만 저역통과시킨다.

```text
Global EKF map pose + Local EKF odom pose
  -> raw map-to-odom planar alignment
  -> exponential smoothing (candidate tau=0.5 s)
  -> smoothed alignment x current Local EKF pose
  -> /localization/control_kinematic_state (map, base_link)
```

- planning과 TF는 기존 `/localization/kinematic_state`를 계속 쓴다.
- 후보 node는 TF를 발행하지 않는다.
- production launch 기본값은 `publish_control_state=false`이다.
- `/localization/odometry/local`을 map trajectory와 직접 섞지 않는다.

## rosbag별 provenance

| 구분 | bag | 사용 목적과 제한 |
|---|---|---|
| production 장주행 | `260919_231906_q0005_r010_gt_object_lightweight_no_traffic.bag` | 최신 Q/R 폐루프. 마지막은 GPS 음영 안에서 종료되어 재획득 검증 불가 |
| production 짧은 반복 | `260920_003403_q0005_r010_gt_object_lightweight_no_traffic_v2_test.bag` | 같은 Q/R의 반복 smoke |
| negative control | `260919_233559_sigma0p05_qxy0p05_gt_object_lightweight_no_traffic.bag` | 기각된 sigma/Q 조합에서도 구조 효과가 재현되는지만 확인 |

모두 MR heading oracle `-1.347 deg`를 적용했다. acquisition과 controller revision이
서로 완전히 같다는 가정은 하지 않는다.

## 결과

단위는 cm이며 이동 중 sample만 사용했다.

| bag | 후보 | 횡 오차 p95 | 횡 frame jump p95 | 판정 |
|---|---:|---:|---:|---|
| production 장주행 | 기존 Global | 5.56 | 1.31 | baseline |
| production 장주행 | tau 0.5 s | **3.94** | **0.20** | 폐루프 후보 |
| production 장주행 | tau 1.0 s | 4.15 | 0.15 | 보조 후보 |
| production 짧은 반복 | 기존 Global | 5.42 | 1.26 | baseline |
| production 짧은 반복 | tau 0.5 s | 4.77 | 0.18 | 폐루프 후보 |
| production 짧은 반복 | tau 1.0 s | **4.50** | **0.13** | 보조 후보 |
| negative control | 기존 Global | 7.06 | 5.15 | baseline |
| negative control | tau 0.5 s | **4.28** | **0.26** | 구조 효과 확인 |
| negative control | tau 1.0 s | 4.54 | 0.16 | 구조 효과 확인 |

`tau=5 s`는 bag에 따라 횡 오차 p95가 약 `7.6~10.5 cm`까지 악화해 제외했다.
종방향 frame jump는 약 `7 cm` 수준이 유지됐다. 이 값은 map correction보다 Local EKF
motion/timestamp에 주로 기인하므로 별도 가설로 다룬다.

### 종·횡 correction 분리 가설

차량 진행축 correction은 즉시 반영하고 횡축만 `0.5~1.0 s`로 늦추는 후보도
사전 검증했다. production 두 bag에서는 등방성 `0.5 s`와 차이가 거의 없었고,
negative-control에서는 진행축을 즉시 반영할 때 종 오차/jump가 오히려 크게
악화했다.

| bag | 후보 | 횡 / 종 오차 p95 | 횡 / 종 jump p95 |
|---|---|---:|---:|
| production 장주행 | 등방성 0.5 s | **3.93 / 6.26 cm** | 0.20 / 7.49 cm |
| production 장주행 | 종 0 s, 횡 0.5 s | 4.02 / 6.28 cm | 0.20 / 7.53 cm |
| negative control | 등방성 0.5 s | **4.27 / 5.52 cm** | 0.26 / 6.85 cm |
| negative control | 종 0 s, 횡 0.5 s | 4.40 / 8.92 cm | 0.26 / 9.63 cm |

따라서 차량축별 파라미터를 추가해 복잡도를 늘리지 않고, 현재 등방성 `0.5 s`를
단일 폐루프 후보로 유지한다.

## 구현 검증

- pure-function unit test `5/5` 통과.
- 기존 GNSS contract test `5/5` 통과.
- localization package install build 통과.
- 격리한 ROS master에서 production 짧은 bag 10초 transport replay:
  - 출력 `499`개, `49.994 Hz`.
  - header stamp 모두 strictly increasing.
  - frame `map -> base_link`.
  - NaN 없음, quaternion norm 정상.

### 구형 GPS 음영 기계 시험

`gps_blank1/2`는 취득 alias와 launcher provenance가 불완전하므로 정확도 채택 근거로
쓰지 않고, 출력 연속성과 복귀 순간의 횡 jump만 확인했다. `gt_source=ego`, heading
offset `0 deg`로 전체 estimator를 다시 실행했다.

| bag | zero-GNSS | Global / Control 출력 수 | 출력 rate | 복귀 후 횡 jump p95 |
|---|---:|---:|---:|---:|
| `gps_blank1` | 5.55 s | 1041 / 1041 | 49.72 Hz | 2.66 / **0.83 cm** |
| `gps_blank2` | 10.93 s | 1659 / 1659 | 49.72 Hz | 3.03 / **0.48 cm** |

후보 토픽은 음영 중에도 한 frame도 추가로 끊기지 않았고 복귀 순간 횡 correction을
완화했다. 다만 `gps_blank1`에서는 오래된 heading/runtime 조건 아래 횡 절대 오차가
커졌으므로 이 결과를 accuracy 개선으로 해석하지 않는다. 새 production 폐루프의
음영 진입·복귀 bag이 여전히 필수다.

## 판정과 다음 gate

판정은 **HOLD**이다. 다음 새 MR rosbag/폐루프에서는 production Global state와
`tau=0.5 s` controller state를 동시에 기록하고, controller 입력만 후보 토픽으로
바꿔 다음 항목을 비교한다.

1. 횡·종 제어 오차와 steering-rate/sign flip.
2. 저속·좌회전·로터리에서 phase lag.
3. GPS 음영 유지와 재획득 correction.
4. checkpoint 순간이동 시 reset 동작.

위 gate 전에는 `localization` alias나 controller 기본 입력을 변경하지 않는다.
