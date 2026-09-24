# 체크포인트 순간이동 감지·reset 가드 사전 검증

상태: **EVIDENCE / HOLD**
검증일: 2026-09-25

## 결론

사고 복구로 차량이 수 m 이상 재배치됐을 때 low-Q Global EKF가 과거 위치를 오래
유지하는 위험을 막기 위한 가드를 구현했다. 정상 주행 두 bag에서 5 m 기준 오탐은
0건이고, 합성 20 m 이동은 세 번째 연속 GNSS sample에서 감지됐다. ROS transport에서
Global EKF reset용 `/localization/initialpose`가 1회 발행되는 것도 확인했다.

실제 체크포인트 이동 bag과 폐루프 검증이 없으므로 기능은 **기본 OFF**이며, 켜더라도
기본은 **monitor-only**다. Production 자동 reset으로 승격하지 않는다.

## 가설과 단일 변경 변수

- 가설: GNSS와 fused map pose가 5 m 이상 3 sample 연속 분리되면 정상 양자화나 짧은
  재획득 correction이 아니라 checkpoint 재배치일 가능성이 높다.
- baseline: relocation 감지와 명시적 Global EKF reset 없음.
- 후보: `localization_relocation_guard.py`.
- 유지: heading `-1.347 deg`, GNSS sigma `0.10 m`, Global Qxy `0.0005`, receipt-time,
  Local/Global EKF 구조.

## 안전 계약

```text
GNSS map x/y - /localization/kinematic_state map x/y
-> 5 m 이상, co-time age 0.1 s 이하, 3 sample 연속
-> monitor-only 경고
-> 명시적으로 monitor_only=false인 실험에서만 Global EKF set_pose
```

- corrected `/localization/gt/orientation`이 0.1초 이내일 때만 active reset한다.
- reset pose는 GNSS x/y/z와 corrected orientation을 사용한다.
- Local EKF는 reset하지 않는다. Local odom 연속성을 유지하고 Global `map->odom`이 새
  위치를 흡수한다.
- 5초 cooldown으로 reset 반복을 막는다.
- 자동 reset은 `enable_relocation_guard=false`와
  `relocation_guard_monitor_only=true` 두 겹으로 차단한다.

## 정상 bag 오탐 감사

GNSS sample마다 가장 가까운 fused state를 0.1초 이내에서 매칭했다.

| bag | pair | age p95 | innovation p50 / p95 / p99 / max | 5 m 이상 |
|---|---:|---:|---:|---:|
| `260919_231906_q0005_r010_gt_object_lightweight_no_traffic.bag` | 6,420 | 9.64 ms | 0.066 / 0.198 / 0.264 / **0.471 m** | **0** |
| `260920_003403_q0005_r010_gt_object_lightweight_no_traffic_v2_test.bag` | 1,560 | 9.38 ms | 0.047 / 0.165 / 0.219 / **0.479 m** | **0** |

이전 8초 인위적 GNSS 음영의 재획득 drift `0.53 / 1.38 m`도 5 m보다 작다. 따라서
현재 증거 범위에서는 양자화, 정상 주행, 시험한 재획득 correction과 분리된다.

## 합성 이동과 transport 검증

짧은 정상 bag 후반 절반의 GNSS x에 `+20 m`를 주입한 비파괴 offline audit에서
offset 시작 input index `780`, 최초 trigger index `782`였다. 즉 정확히 세 번째 연속
sample에서 감지했다.

별도 rostest는 실제 ROS message로 state/orientation/GNSS를 발행했다. 20 m innovation
두 sample까지 reset 0회, 세 번째 sample에서 `/localization/initialpose` 1회와 올바른
map pose/covariance를 확인했다.

## 재현

```bash
source /opt/ros/noetic/setup.bash

python3 scripts/evaluation/localization/analyze_relocation_guard_bag.py \
  rosbags/eval/localization/local_mr_stability/\
260920_003403_q0005_r010_gt_object_lightweight_no_traffic_v2_test.bag

python3 scripts/evaluation/localization/analyze_relocation_guard_bag.py \
  rosbags/eval/localization/local_mr_stability/\
260920_003403_q0005_r010_gt_object_lightweight_no_traffic_v2_test.bag \
  --synthetic-offset-m 20
```

## 다음 gate

1. 실제 사고 후 checkpoint 이동 bag에서 이동 거리·heading·topic 순서를 확인한다.
2. monitor-only로 detection 시점과 오탐 0건을 확인한다.
3. 경량 `od_gt` 폐루프에서 active reset 후 route/controller 복구를 확인한다.
4. LAN-to-LAN에서 동일 reset 계약을 확인한다.
5. 가능하면 휴리스틱보다 주최 측의 명시적 checkpoint/reset event를 우선한다.

