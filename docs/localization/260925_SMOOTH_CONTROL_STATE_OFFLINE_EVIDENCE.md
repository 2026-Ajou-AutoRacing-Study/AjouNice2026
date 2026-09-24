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
- controller만 후보 토픽으로 바꾸는 launch 인자를 추가했으며 planning/TF 입력은
  바뀌지 않는다.
- alignment가 `5 m` 또는 `20 deg` 이상 불연속이면 저역통과를 중단하고 즉시
  재정렬한다. 이는 checkpoint 이동 때 과거 위치를 천천히 따라가는 것을 막는
  안전장치이며, 자동 EKF reset을 의미하지 않는다.

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
종방향 frame jump 약 `7 cm`는 후속 oracle 감사에서 `/Ego_topic` 자체의 프레임별
종방향 운동학 residual에 크게 오염된 값으로 확인됐다. 따라서 아래 종 jump는
구조 채택 근거로 사용하지 않고 횡 jump와 실제 controller 지표를 우선한다.

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

### 불연속 reset 기준

정상 주행을 checkpoint 이동으로 오인하지 않도록 translation 기준 `1/2/5/10 m`를
세 bag에서 sweep했다. `1 m`는 장주행에서 74회, negative control에서 37회,
`2 m`도 각각 16회와 5회 잘못 발동했다. `5 m`와 `10 m`는 세 bag 모두 0회였다.
따라서 후보 기본값은 더 작은 false-trigger-free 값인 `5 m`로 정했다. 실제
checkpoint 이동 bag이 없으므로 이 기준은 HOLD이며 새 bag에서 반드시 재검증한다.

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

## 다음 폐루프 A/B 실행 계약

기존 planning 상태는 바꾸지 않고 다음 두 인자만 후보로 켠다.

```bash
# localization terminal
localization publish_control_state:=true

# planning/control terminal: 기존 나머지 인자는 그대로 유지
roslaunch ajounice_planning_integration morai_planning_stack.launch \
  start_controller:=true \
  controller_kinematic_state_topic:=/localization/control_kinematic_state
```

Baseline은 두 인자를 생략한 기존 실행이다. 후보 실행에서도 planning과 TF는 계속
`/localization/kinematic_state`를 사용하며 pure-pursuit와 longitudinal PI만 후보
토픽을 구독한다. 두 controller node가 서로 다른 상태를 보게 하는 설정은 금지한다.
