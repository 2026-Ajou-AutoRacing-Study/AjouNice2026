# GPS 음영 횡드리프트 — Validator 폐루프 개선 실험

상태: **EVIDENCE / soft 사용자 승인 승격 / 개선 Smooth MR 실제 폐루프 확인 / LAN·장기 음영 별도**.
2026-10-15, Asia/Seoul.

## 결론

현재 Global EKF의 미관측 횡가속도 `ay`가 GPS 음영에서 유지되고 `vy`와 횡위치를
계속 적분하는 현상을 실제 폐루프 bag에서 확인했다. 약한 `vy=0` 모델 제약을
**Global EKF에만** 추가한 후보가 동일 raw replay 및 실제 장애물 폐루프에서 이를 줄였다.
초기 실험에서는 후보를 기본 OFF로 분리했다. 이후 사용자 승인으로 soft를 기본 ON으로
승격하고, 아래 마지막 절의 vehicle-pivot/종횡 분리 Smooth를 기본값에 반영했다.
alias·Q/R·heading·시간 정책·Local EKF·판단/제어 정책은 유지한다.
현재 실행 계약은 [CURRENT](COMPETITION_LOCALIZATION_CURRENT_STATE.md)가 기준이며,
아래 과거 단계의 OFF/HOLD와 `0.5/0.5 s`는 당시 조건이지 현재 기본값이 아니다.
최종 반복·회귀 결과는 아래 표와 Git 제외 분석 폴더에서 관리한다.

## 실행·평가 계약

| 항목 | 실제 사용 조건 |
|---|---|
| branch / 기반 HEAD | `dataset/base5` / `d9ce8d425563b6e4fcdb402ff31c32e5583b50ee` + 이번 수정 |
| acquisition | MR live, `v.S4.251001.MolitComp03_Linux`, 개인 PC |
| 실행 | source `scripts/workspace/source_aw149.sh` → source validator CLI |
| 폐루프 | Competition GNSS/IMU localization → `od_gt` → tracking/Map-CV → ROS1 Planning → `/planning/trajectory` → PP/PI → `/ctrl_cmd` → MORAI UDP |
| motion / 초기 위치 | `gt_source=competition` / `initial_position_source=ego` |
| estimator / evaluator heading | 각각 raw 입력에서 `-1.347°` 한 번 적용 |
| state 소비 | Planning·TF: Global; Controller: Smooth `0.5/0.5 s`. GT 대조만 보정된 Ego provider |
| time policy | live receipt wall-time, `use_sim_time=false`; offline replay는 동일 기록 시각의 1× sim clock |
| 센서·부하 | 같은 `Sensor_competition_local.json`, validator 전용 Network; od_gt·신호등/RViz/영상 OFF, 카메라/LiDAR bag 제외 |
| 녹화 | `localization_analysis`, bag ON, `localization_safety.enabled=true` |
| 모델 변경 | Global-only soft body `vy=0` prior, σ `0.20 m/s`; 기존 GT twist·Local·Q/R·yaw/Smooth 동일 |

관련 launch/Python 설치본 80개를 source와 대조했다. 최신 제어기 acceleration-map 코드가
실제 설치 binary에 있고 devel/install ELF Build ID 및 text/rodata가 일치함도 확인했다.
mtime만으로 stale이라고 판단하지 않았다. 측위 후보 수정 후 `localization_manager`만
증분 설치했다. 실제 명령·파라미터·소스/설치 SHA는 각 실행의 원본 receipt가 기준이다.

첫 공유 입력 `localization_baseline.yaml`, `localization_gt_pose.yaml` **전체**를 개인
`config/missions/gps_shadow/workspace/`로 복사했다. 무장애물·박스 1개·좌우 배치 박스 2개,
무음영 일반 장면·다른 진입을 각각 새 실행으로 만들었다. 이전 snapshot을 강제 resume하지 않았다.

## 원인 확인

박스 1개 기준선의 실제 11.424초 좌표 0 음영에서:

- Global `ay=-0.027383639 m/s²`가 사실상 일정했다.
- Global `vy=-0.06319 → -0.37591 m/s`였고, 기울기는 같은 `-0.027383638 m/s²`였다.
- 독립 Ego 대비 횡오차는 `-0.071 → -2.485 m`로 커졌다.
- `vy₀T + ½ayT² ≈ -2.51 m`로 관측 발산의 크기·부호를 설명한다.
- Local `vy=0`은 관측된 0이 아니라 초기 미관측 상태다. 큰 covariance를 실제 오차로 동일시하지 않는다.
- co-time TF closure는 mm 수준이었다. 이 2.5m 오차를 TF 정합 문제로 설명할 근거는 없다.

음영 진입 시 추정 `ay`가 달라 같은 무장애물도 횡 p95 `0.057~0.599 m`로 달랐다.
따라서 한 번의 성공이나 Smooth만으로 발산 해결을 판단하면 안 된다.

IMU `ay`를 바로 다시 켜는 후보는 우선 사용하지 않았다.
현재 [Noetic EKF 모델](https://github.com/cra-ros-pkg/robot_localization/blob/noetic-devel/src/ekf.cpp)은
차체 횡속도에 `ay·dt`를 적분한다. 실제 IMU 횡가속도에는 회전의 구심가속도가 크게 포함돼
단순 추가가 현재 모델에 적합한지 따로 검증해야 한다. IMU lever-arm 보정도 기존 필터와
중복 적용하지 않는다. raw Ego velocity.y와 위치의 기준점 차이 가능성이 있어 이를 그대로
base_link 횡속도 정답으로 쓰지 않았다.

## 후보 구현

`morai_gt_motion_adapter.py`는 `noise/lateral_model/enabled=true`에서만 별도
`/localization/constraints/lateral_velocity`를 발행한다. 이는 **차량 운동 모델 prior**이며
Competition의 미제공 `velocity.y=0`을 GT 측정으로 쓰는 것이 아니다.

- `TwistWithCovarianceStamped`, `base_link`, source seq/stamp 유지.
- y=0, y variance=`0.20²=0.04 (m/s)²`; 다른 축 variance는 큰 값이며 EKF mask도 OFF.
- Global 후보 overlay의 `twist1_config`에서 state index 7(vy)만 ON.
- 기존 GT orientation/twist는 후보 ON/OFF에서 byte-identical.
- 초기 후보 단계에서는 매 launch에서 모델 prior OFF를 먼저 설정하고, 개인 noise config가 명시적으로 켤 수 있었다.
  기존 roscore의 후보 파라미터가 다음 baseline에 잔류하는 것을 막는다.

개인 후보 파일은 workspace의 `candidates/noise_lateral_sigma020.yaml`,
`candidates/global_lateral_sigma020.yaml`이다. 공유 base/production config를 후보로 덮어쓰지 않았다.
`loc_agent_soft_lateral*.yaml`로 실행하며 기본 `localization` alias에는 활성화하지 않았다.

## `ay OFF`의 정확한 의미와 전체 코스 회귀 가설

다음 세 조작은 다르다. 이번 후보는 **세 번째**다.

| 조작 | 의미 | 현재/실험 상태 |
|---|---|---|
| IMU `ay` 측정 mask OFF | 센서가 보고한 횡가속도를 직접 EKF update에 넣지 않음 | 기존 production Local/Global 모두 이미 OFF |
| EKF 내부 `ay` 상태 제거/항상 0 고정 | 상태 전파 자체를 바꾸거나 별도 제약으로 내부 상태를 제한 | 이번 구현에서 하지 않음 |
| soft body `vy≈0` prior 추가 | 통상 주행에서 옆으로 미끄러지는 속도가 작다는 모델 정보를 융합 | Global-only, σ=0.20m/s; 초기 후보 OFF → 사용자 승인 ON(마지막 절) |

GT CTRA 최초 commit `b70d3ec3`(2026-08-02)부터 IMU `ay` mask는 OFF였다.
base GNSS+IMU YAML만 읽으면 ON으로 보이지만 그 뒤에 로드하는 CTRA overlay가 OFF로
덮어쓴다. 과거 설계는 body `vy≈0`을 의도했으나, 관측을 끄는 것만으로 15-state
필터 안의 `vy/ay`가 고정되는 것은 아니다. GNSS 위치 update와 교차 covariance로
내부 횡상태가 변하고, GNSS가 끊기면 마지막 추정값이 전파될 수 있다.
`two_d_mode=true`도 z/roll/pitch 등 평면 밖 상태를 제한하는 설정이지,
평면 안의 y/vy/ay를 모두 0으로 만드는 설정은 아니다.

`y`는 **차체 왼쪽 축**이며 지도 고정 Y축이 아니다. `vy≈0`이어도 차체 yaw가 변하면
`map`에서 좌우로 이동하고 곡선·차선 변경을 할 수 있다. 예를 들어 `vy=0`인 평면
모델에서도 `map` 이동은 `X_dot=vx*cos(yaw)`, `Y_dot=vx*sin(yaw)`다.

물리적인 IMU 횡가속도와 이 필터 내부 `ay`의 전파 의미도 구별한다. 중력·센서 위치
보정을 마친 평면 가속도는 대략 다음 관계를 가진다.

```text
IMU body ay = d(body vy)/dt + yaw_rate * body vx
```

따라서 일정한 속도로 미끄러짐 없이 좌회전해도 body `vy`는 거의 0인 반면 IMU `ay`는
양의 구심가속도를 보인다. 예를 들어 10m/s, yaw-rate 0.2rad/s이면 약 2m/s²다.
[현재 설치 버전 2.7.7의 EKF 코드](https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/2.7.7/src/ekf.cpp)는
`vy += ay*dt`로 전파하며 위의 회전 좌표계 항을 별도로 빼지 않는다.
[같은 버전의 IMU 입력 처리](https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/2.7.7/src/ros_filter.cpp)의
TF lever-arm 보정은 센서 설치 위치 효과를 다루는 것이며 이 차체 회전 항과는 다르다.
그래서 raw IMU `ay`를 그대로 켜는 것을 드리프트의 당연한 해결책으로 취급하지 않는다.
신호 자체가 틀렸거나 실제 횡가속도가 0이라는 주장도 아니다.

최신 팀장님 E24 원본 bag의 정상 회전 2,068표본(`vx>1m/s`,
`|IMU yaw-rate|>0.15rad/s`)에서도 raw IMU `ay`와 `yaw_rate*vx`의 상관계수는
0.9857, raw ay 절대 p95는 5.33m/s²였다. 두 신호는 서로 다른 원점·시각 및 센서
설치 효과를 포함하므로 그 차이를 전부 센서 오류로 보지 않는다. 원본 IMU TF는
base_link 전방 1.528m, 높이 0.2m이고 회전은 identity다. 이 raw 상관은 모델 설명의
근거이지 prior 성능 또는 lever-arm 보정 완료의 근거가 아니다.

[공식 설정 가이드](https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/noetic-devel/doc/configuring_robot_localization.rst)도
바퀴형 비홀로노믹 로봇의 횡속도 0 정보를 융합하는 사례와 IMU 횡가속도 제외를 설명한다.
다만 실제 sideslip·충돌·속도/heading 기준점 오차가 있는 우리 차량에서는 강한 0 고정보다
soft prior를 비교해야 한다. σ는 허용오차의 경계가 아니며 50Hz 반복 update가 실측
횡속도도 0 쪽으로 편향시킬 수 있다.

같은 E24 bag의 Ego 위치 차분을 보정 heading에 투영한 약 0.20초 평균 횡속도도
확인했다(실제 평균 창 0.225초, gap을 넘는 차분 제외). 절대 body 횡속도 p95는
이동 전체 0.099m/s, 선회 0.119m/s, 고속 선회 0.132m/s였다. 반면 raw Ego
`velocity.y` 전체 p95는 0.628m/s다. `raw vy − 위치 차분 body vy`는
`1.305*yaw_rate`와 상관계수 0.9982, 잔차 MAE 0.0096m/s로 연결됐다.
회전 기준점 차이와 맞는 패턴이지만 이 회귀만으로 물리적 기준점을 확정하지 않는다.
차분은 평균 운동이고 시간·heading 품질에도 의존하므로 실제 순간 슬립 상한으로
해석하지 않는다. soft prior의 타당성 보조 근거이지 전체 코스 accuracy 검증을
대신하지 않는다. 재현 JSON은 같은 ignored 분석 폴더의 `ego_body_lateral_physical_audit.json`이다.

후속 검증은 같은 최신 팀장님 E24 전체 코스 raw 입력으로 (1) 기존 OFF-prior,
(2) soft prior ON, (3) 별도 Global-only IMU `ay` ON 진단을 비교한다. (3)은 offline
모델 진단이지 live 운용 후보가 아니다. 이어 actor-free 전체 코스를 실제 od_gt 판단·제어
폐루프로 A/B한다. 선회/고속/직진과 GPS 음영을 분리해, 한 음영에서만 좋아진 후보를
전체 코스 개선이라고 과장하지 않는다. 상세 결과는 같은 문서에 누적한다.

## 1차 정량 결과

단위 m, 절대 횡오차 p95. 실제 차량 지표는 raw Ego와 당시 수신된 유효한 planning
trajectory의 유한 선분 거리다. 정상 회피 자체를 측위 드리프트로 세지 않는다.

| 조건 | Global 측위 기존 → 후보 | Smooth 측위 기존 → 후보 | 실제 Ego→trajectory 기존 → 후보 |
|---|---:|---:|---:|
| 동일 박스 raw replay, 11.424초 음영 | 2.271 → 0.0557 | 2.101 → 0.0524 | 원래 주행 재사용; 후보 폐루프 효과로 세지 않음 |
| 무장애물, 실제 음영 | 0.0570 → 0.0875 | 0.0565 → 0.0875 | 0.154 → 0.164 |
| 박스 1개, 실제 음영 | 2.265 → 0.0488 | 2.096 → 0.0494 | 1.944 → 0.1778 |
| 좌우 배치 박스 2개, 실제 음영 | 1.805 → 0.0637 | 1.660 → 0.0637 | 1.542 → 0.1626 |
| 무음영 일반 장면, 움직임 표본 | 0.0645 → 0.0205 | 0.0280 → 0.0399 | 0.0156 → 0.0199 |
| 시작점 10m 상류, 실제 음영 | 0.2652 → 0.0939 | 0.2450 → 0.0939 | 0.2361 → 0.1320 |
| 박스 1개 반복, 실제 음영 | 0.9517 → 0.0741 | 0.8762 → 0.0745 | 0.8089 → 0.2066 |

박스 1개에서 Global 종오차 p95는 `0.234 → 0.243 m`, yaw p95는 `0.081 → 0.101°`로
소폭 악화했다. 박스 2개는 종 `0.289 → 0.166 m`, yaw `0.084 → 0.077°`였다.
다른 실제 주행의 종오차 변화는 physics/시간축 차이까지 포함한다. raw Ego 이동거리와
speed×time의 차이로 설명되는 부분을 velocity scale로 보정하지 않는다.

박스 2개는 좌우에 놓았지만 실제 planner와 Ego는 같은 쪽으로 우회했다.
**두 객체 회귀이지 실제 양방향 회피 검증은 아니다.** planning policy를 변경해 이를 강제하지 않았다.
무장애물의 약 3cm 악화와 일반 장면의 Smooth 약 1cm 악화도 함께 기록한다.
일반 장면의 조향 변화율 p95는 `0.209 → 0.180 rad/s`지만 단일 최대값은
`1.31 → 1.68 rad/s`로 커졌다. 평균·p95가 나아도 모든 순간의 개선이라고 주장하지 않는다.
다른 진입은 validator 주행시간이 두 조건 모두 약 27.44초였다.
박스 반복 기준선에서는 `ay=+0.01362 m/s²`가 남아 첫 기준선과 반대 방향으로
발산했다. 후보는 양쪽 반복에서 횡 p95 `4.88/7.41 cm`였고, 주행시간은
기존 `30.96/30.46 s`, 후보 `30.20/30.19 s`였다. 완전한 물리 seed 통제는 아니다.

재획득 이후 후보 Global 횡오차가 0.10m 이내로 1초 유지되기 시작하는 시간은
관측 첫 표본 수준(약 `0.003~0.020 s`)이었다. 반면 실제 차량의 경로 추종 오차는
이후 곡선에서 다시 커질 수 있다. XY 회복·횡측위 회복·실제 경로 추종 회복을
한 숫자로 합치지 않는다. 박스 후보에서도 후속 곡선의 약 `0.3~0.4m` 추종 오차는
남아 있어 현재 planning/control 전체 문제를 해결했다고 판단하지 않는다.

## 기록·분석 재현

원본 실행은 `planner_logs/validation/loc_agent_*_261015_*/`에 있다. case별 bag,
실제 사용 YAML/JSON/config, runtime receipt, parameter dump, 안전/녹화 audit를 보존한다.
일회성 표·전체 수치·그림·동일입력 replay는
`planner_logs/analysis/261015_gps_shadow_validator/README.md` 아래에 모았다.
`STUDY_SUMMARY.json`은 13개 실행의 판정·안전·필수 count·실제 parameter/통계를 연결한다.

```bash
source scripts/workspace/source_aw149.sh
/usr/bin/python3 scripts/evaluation/localization/analyze_gps_shadow_validator.py \
  <실제 case bag> --heading-offset-deg -1.347 \
  --output planner_logs/analysis/261015_gps_shadow_validator/<name>.json \
  --markdown planner_logs/analysis/261015_gps_shadow_validator/<name>.md
```

분석기는 raw GPS zero/invalid/stale와 재획득 1초를 구분하고, Ego gap·teleport·역행을
넘겨 보간하지 않는다. 실제 `CONTROLLER_START~RUN_FINISHED` 구간으로 준비·종료 정지를
제외하며, Global/Local/Smooth·TF·입력·제어의 실제 count/timing을 검사한다.
평가 전체 JSON에는 mean/p50/p90/p95/max·표본 수·recovery·steering이 있다.

## 남은 gate

13회 실제 폐루프의 목적지/충돌 판정과 bag audit, 종료 P·실제 정지 확인을 통과했다.
후보/기준선 뒤에 다른 장면을 실행하기 전 `/rosout`만 남음을 확인했다. 끝에는 실험
prior parameter도 OFF로 정리했다. source/install adapter·launch 일치와 관련 27개
단위/회귀 테스트(신규 prior 9, 분석기 10, 기존 측위 계약 8)를 통과했다.

현재 근거를
대회 LAN·장시간 음영 정차·고속 슬립·정상 체크포인트 복귀의 통과로 확대하지 않는다.
당시 Production 승격은 별도 승인 사항이었다. 이후 사용자 승인은 마지막 절에 기록하며,
이 승인을 미완료 LAN·장시간 정차·슬립 검증의 통과로 해석하지 않는다.

## 후속 전체 코스 폐루프 (같은 조건 4회)

별도 `full_course/workspace/loc_fullcourse_baseline.yaml`,
`loc_fullcourse_soft_lateral.yaml`을 사용했다. 시작점·목적지·16개 Route A 경유점과
600초 상한은 공유 full_course 예제 그대로다. tracked
`localization_full_route_no_actors.json`을 개인 사본으로 사용해 배우만 없는 전체
코스를 주행했고, 실제 od_gt → tracking/Map-CV → planning → PP/PI → UDP 경로는 유지했다.
가까운 목적지 shortcut, 정속 직진 명령, 별도 제어기, planning 속도/회피 변경은 없다.
신호등/RViz/영상 OFF와 센서 생성 부하는 네 조건 모두 동일했다.

| 새 execution | 조건 | 결과 | Validator 주행 시간 | GPS 좌표 0 표본 | 충돌 |
|---|---|---|---:|---:|---:|
| `loc_fullcourse_baseline_261015_011433` | 기존 | 경로 이상·정차 후 agent graceful abort, ERROR/MANUAL_ABORT | 완주 아님 | 0 | 0 |
| `loc_fullcourse_soft_lateral_261015_011945` | 후보 | PASS | 151.22s | 217 | 0 |
| `loc_fullcourse_baseline_261015_012417` | 기존 반복 | PASS | 151.19s | 212 | 0 |
| `loc_fullcourse_soft_lateral_261015_012811` | 후보 반복 | PASS | 150.82s | 210 | 0 |

첫 기존 주행은 controller 시작 약 104.05초 후, 실제 Ego가 map
`(200.467,633.226)m`에서 26.95m/s로 이동할 때 당시 궤적이 목적지 부근 두 점만
남는 형태로 바뀌었다. 유한 궤적까지 거리는 약 580.3m였다. GPS는 계속 유효했고,
Global 횡측위 오차는 이동 표본 p95 0.055m, Smooth는 0.034m였다. 이 정지를 GPS
음영 횡발산으로 분류하지 않는다. ERROR의 elapsed=0은 abort 결과 필드이므로 실제
주행 시간을 0으로 쓰지 않고 events의 controller 시작/CASE_RESULT를 사용한다.
분석기에도 abort의 `CASE_RESULT`에서 평가를 끝내는 가드를 추가해 종료 brake/P 구간을
제외했다. 경로 이상이 어떤 planning 내부 분기에서 생겼는지는 이 측위 실험에서
수정하거나 확정하지 않았다.

같은 기존 설정의 반복은 완주했으므로 첫 실패를 prior 효과의 증명으로 삼지 않는다.
완주 시간도 약 151초로 사실상 같다. 후보 두 주행은 약 7초 GPS 음영·재획득을
포함하지만 실제 모든 미션/차선 규정/충돌 슬립 검증은 아니다. 원래 E24 bag의 배우·신호등·
영상 및 software 조건은 이 actor-free 주행과 달라 완주 시간의 직접 비교 대상이 아니다.

네 주행 모두 녹화 audit와 fresh 실제 P·정지 종료 gate를 통과했다. 끝에 host graph는
`/rosout`만 남으며 user master/Simulator는 보존한다. 후보 파라미터는 idle host에서
OFF로 정리한다. source/install adapter·launch SHA가 일치하고 관련 57개 단위/회귀
테스트를 통과했다. 상세 원본은 `planner_logs/validation/loc_fullcourse_*_261015_*/`,
재현·안전 요약은 `planner_logs/analysis/261015_gps_shadow_fullcourse/LIVE_STUDY.md`다.

### 완주 3회의 독립 GT 정확도와 실제 차량 추종

준비·종료 구간을 제외하고 GT 속도 `>0.5m/s`인 표본을 사용한다. 위치 단위 m,
yaw 단위 deg이며 각 셀은 **절대 mean / p50 / p95**다. 실제 다른 주행이므로 같은 입력의
인과 검증과 구분한다. 첫 abort를 정상 완주 표본에 섞지 않는다.

| 측위 지표, 전체 이동 구간 | 기존 반복 | 후보 1 | 후보 2 |
|---|---:|---:|---:|
| Global XY | 0.2127 / 0.1670 / 0.5275 | 0.1922 / 0.1606 / 0.4560 | 0.1807 / 0.1646 / 0.4110 |
| Global 횡 | 0.0475 / 0.0182 / 0.0698 | 0.0148 / 0.0096 / 0.0578 | 0.0147 / 0.0098 / 0.0527 |
| Global 종 | 0.1870 / 0.1579 / 0.4429 | 0.1900 / 0.1596 / 0.4555 | 0.1783 / 0.1637 / 0.4100 |
| Global yaw | 0.0116 / 0.0045 / 0.0484 | 0.0124 / 0.0043 / 0.0519 | 0.0105 / 0.0042 / 0.0419 |
| Smooth XY | 0.2069 / 0.1654 / 0.5140 | 0.1902 / 0.1590 / 0.4372 | 0.1800 / 0.1632 / 0.4152 |
| Smooth 횡 | 0.0390 / 0.0136 / 0.0626 | 0.0257 / 0.0142 / 0.0782 | 0.0257 / 0.0158 / 0.0770 |
| Smooth 종 | 0.1863 / 0.1566 / 0.4398 | 0.1856 / 0.1546 / 0.4368 | 0.1751 / 0.1578 / 0.4141 |
| Smooth yaw | 0.0127 / 0.0047 / 0.0526 | 0.0140 / 0.0050 / 0.0594 | 0.0097 / 0.0034 / 0.0395 |

| 구간별 p95, m | 기존 반복 | 후보 1 | 후보 2 |
|---|---:|---:|---:|
| GPS 음영 Global 횡측위 | 1.3426 | 0.0842 | 0.0851 |
| GPS 음영 Smooth 횡측위 | 1.1984 | 0.0842 | 0.0851 |
| GPS 음영 실제 Ego → 당시 유효 trajectory 횡추종 | 1.0869 | 0.1584 | 0.1544 |
| 선회 Global 횡측위 (`|IMU wz|>0.15rad/s`) | 0.0586 | 0.0385 | 0.0333 |
| 고속 Global 횡측위 (`vx>15m/s`, 구간 중첩 허용) | 0.4982 | 0.0754 | 0.0746 |
| 유효 GNSS Global 횡측위 | 0.0497 | 0.0323 | 0.0315 |
| 전체 실제 Ego → 당시 유효 trajectory 횡추종 | 0.4055 | 0.3634 | 0.3472 |
| 선회 실제 Ego → 당시 유효 trajectory 횡추종 | 0.5062 | 0.4182 | 0.3981 |
| 고속 실제 Ego → 당시 유효 trajectory 횡추종 | 0.5776 | 0.3771 | 0.3715 |
| 직진 실제 Ego → 당시 유효 trajectory 횡추종 | 0.2680 | 0.1751 | 0.1786 |

세 완주에서 Global state는 약 50Hz이며 header 최대 gap은 24.92~27.34ms,
역행·중복은 없었다. 조향 변화율 p95는 기존 0.3279rad/s, 후보 0.3205/0.3091rad/s다.

**관측된 trade-off:** 음영 횡측위와 실제 추종은 크게 좋아지고 정상 Global 선회도
악화하지 않았지만, Smooth 전체 횡 p95는 6.26cm에서 7.82/7.70cm로 약 1.4~1.6cm
커졌다. 종·yaw도 두 후보가 모든 통계에서 일관되게 개선된 것은 아니다. Global을
바꾸더라도 Local+alignment smoothing의 controller 위치가 자동으로 모든 구간에서
좋아지는 것은 아니다. 이번 실험에서는 Smooth tau를 함께 튜닝하지 않았다.

전체 코스 XY는 종방향 오차의 영향이 크며, 기존 저속/다른 부하의 보고서 cm-level 표를
이 고속 주행에 그대로 적용하지 않는다. 차량별 주행시간/physics/센서 cadence 변동을
새 GT velocity bias라고 단정하거나 임의 scale로 보정하지 않는다. 동일 원시 입력
replay가 candidate 자체의 손익을 확인하는 추가 근거다.

추가 시간축 진단에서 세 완주의 음영 전 이동거리/speed 적분 비율은
0.99978/0.99937/0.99941, 음영 중은 1.00009/1.00000/1.00000이었다.
이 주행에서 과거 약 22% 속도 적분 불일치가 재현됐다는 근거는 없다.
Ego 위치 변화 대비 per-step 종방향 운동 residual 절대 p95는 약 0.090~0.092m,
raw header–bag receipt 차이 p95는 약 1.25~1.30ms였다. 이 진단은 전체 종측위 오차의
원인을 한 가지로 확정하지 않으며 UDP timestamp 정책이나 velocity scale은 바꾸지 않았다.

## 최신 E24 동일 입력 — 초기화 정합 후 최종 비교

원본은 최신 팀장님 `261014_teammate_fullstack/.../planbag_full_baseline_00_sample_P_20261014_012302.bag`다.
최초 whole-bag replay는 로드 전 첫 Ego로 bag+0.284초에 초기화됐다. 원본 주행은
SCENARIO_READY 이후 bag+4.692초(`1791908596.106376886`)에 초기화했으므로 초기 조건이
달랐다. 이 차이를 고친 **`baseline_ready / lateral_ready / imuay_ready`만 최종 비교**다.
이전에 생성한 `baseline_confirmed / lateral / imuay`는 초기화 불일치 진단용이며
최종 성능 표에서 제외한다. 최초 `baseline`은 실행 중 래퍼 수정으로 종료 parameter
dump가 누락된 기술 실패도 있어 별도로 제외한다. 모든 원본/실패 artifact는 보존했다.

동일한 derived input은 실제 초기화 시각 이후의 raw GPS/IMU/Competition/Ego와 원본
static TF 3개 전체를 **메시지/header/receipt 시각 변경 없이** 복사했다. 원본/결과 SHA와
제외 count를 보존했다. 초기 pose의 XY/quaternion/stamp/covariance가 원본 post-ready
입력과 정확히 일치하고, raw topic count와 센서 TF도 각 조건에서 동일함을 확인했다.
실제 설치 `robot_localization=2.7.7`, installed/source adapter·launch SHA를 고정했다.
master 24515의 1× sim clock estimator replay이며 UDP·planning·controller는 실행하지 않는다.
원래 Ego 움직임·trajectory는 참조일 뿐 후보 폐루프 성능으로 세지 않는다.

단위 m, 전체 이동 표본의 각 셀은 **절대 mean / p50 / p95**다.

| 지표 | 기존 | Global soft vy prior 후보 |
|---|---:|---:|
| Global XY | 0.3334 / 0.3322 / 0.6475 | 0.3100 / 0.3177 / 0.5681 |
| Global 횡 | 0.0469 / 0.0186 / 0.0863 | 0.0194 / 0.0126 / 0.0665 |
| Global 종 | 0.3091 / 0.3198 / 0.5675 | 0.3069 / 0.3170 / 0.5679 |
| Smooth XY | 0.3227 / 0.3220 / 0.6390 | 0.3035 / 0.3104 / 0.5666 |
| Smooth 횡 | 0.0412 / 0.0153 / 0.0800 | 0.0340 / 0.0208 / 0.1108 |
| Smooth 종 | 0.3021 / 0.3112 / 0.5658 | 0.2980 / 0.3062 / 0.5657 |

| 구간별 횡 p95, m | 기존 | 후보 |
|---|---:|---:|
| 음영 Global | 1.2610 | 0.1165 |
| 음영 Smooth | 1.1228 | 0.1165 |
| 선회 Global | 0.0652 | 0.0496 |
| 고속 Global | 0.4464 | 0.1108 |
| 선회 Smooth | 0.0700 | 0.1185 |
| 유효 GNSS Smooth | 0.0549 | 0.1005 |
| 재획득 첫 1초 Global | 0.0715 | 0.0372 |
| 재획득 첫 1초 Smooth | 0.0640 | 0.1323 |
| 재획득 이후 Global | 0.0647 | 0.0389 |
| 재획득 이후 Smooth | 0.0636 | 0.1133 |

음영 중 기존 Global `ay≈-0.03946m/s²`와 `vy=-0.03069→-0.31577m/s`가 남았다.
후보의 ay 절대 평균은 약 0.001205m/s², vy는 `-0.001666→-0.0000108m/s`였다.
이는 AY 센서를 끄는 추가 변경이 아니라 횡속도 모델 prior의 결과다.

양쪽 state는 약 49.999Hz, 최대 header gap 약 26.0/25.1ms이며 역행·중복이 없었다.
Global yaw p95는 0.09065→0.08925°, Smooth는 0.07955→0.07545°로 큰 변화가 없었다.

**재획득 손익도 공개한다:** Global 횡오차가 0.10m 이내로 1초 유지되기 시작하는 시간은
기존 0.0205초, 후보 0.0176초였지만 Smooth는 0.0205→1.1846초로 느려졌다.
기존 로그에는 GNSS 복귀 약 30ms 뒤 `translation_discontinuity` alignment reset이
있고 후보에는 없었다. 큰 기존 오차가 reset으로 즉시 줄어든 것과, 작은 후보 오차가
기존 0.5초 smoothing으로 천천히 줄어드는 동작을 구분한다. 따라서 "모든 복귀 지표
개선"이라고 쓰지 않는다. 다음 개선의 대상은 Global 관측 부족보다 Smooth alignment
정상 선회/재획득 손익이며, 이번에는 이 기본값을 함께 바꾸지 않았다.

같은 원본 input의 거리/speed 적분 비율은 음영 전/중/후 약 0.99878/0.99747/0.99945다.
과거 0.82 real-time regime의 속도 scale 불일치가 재현된 근거가 없어 velocity rescaling은
하지 않았다. 올바른 초기화 후에도 종오차 평균 약 0.31m가 남았으므로 이를 초기화
차이 하나로 설명하지 않는다. AY ON 진단은 아래에서 별도로 판단한다.

### Global IMU ay 입력 ON — 독립 진단 결과: REJECT

세 번째 동일 입력 조건은 **Global IMU mask index 13(ay)만 ON**이며 Local과 prior는
기존 OFF다. 따라서 별도 물리 모델/입력 adapter를 시험한 것이 아니라 현재 필터에
기존 IMU 횡가속도를 직접 추가하는 한 변수 진단이다. 실제 MORAI 제어로 실행하지 않았다.
세 조건 모두 동일한 초기화/원시 count/TF/config gate를 통과했다.

| Global 비교, 위치 단위 m | 기존 | soft vy prior | IMU ay ON 진단 |
|---|---:|---:|---:|
| 전체 이동 XY 평균 | 0.3334 | 0.3100 | 0.7824 |
| 전체 이동 횡 평균 | 0.0469 | 0.0194 | 0.5771 |
| 음영 횡 p95 | 1.2610 | 0.1165 | 16.6928 |
| 선회 횡 p95 | 0.0652 | 0.0496 | 0.5846 |
| 고속 횡 p95 | 0.4464 | 0.1108 | 8.2592 |
| 전체 yaw p95, deg | 0.0907 | 0.0892 | 0.2664 |

AY ON 음영 횡오차 최대는 17.768m, Global `vy=-1.248→-3.605m/s`였다.
해당 구간 내부 ay 절대 평균은 0.457m/s², p95는 2.001m/s²다.
Smooth 음영 횡 p95도 16.236m로 무너졌다. 그런데 출력은 49.998Hz,
header 최대 gap 25.022ms로 정상이라 frame skip/output starvation으로 설명할 수 없다.

앞서 확인한 구심가속도·몸체 좌표계 미분 관계와 현재 2.7.7의 `vy += ay*dt` 전파가
정합하지 않는다는 가설을 지지한다. 센서 ay가 거짓이거나 무잡음 IMU가 쓸모없다는
의미는 아니다. 다른 물리 모델, 회전 항을 정합시킨 입력, 실제 횡속도 관측이 있는
시스템이라면 사용 여부를 다시 판단할 수 있다. **현재 모델에 ay를 그대로 추가하는
변경은 기각하고 기존 IMU ay OFF를 유지한다.** ax/yaw-rate는 이 실험에서 끄지 않았다.

### 당시 후속 판정과 남은 승인

- Global soft vy prior는 장애물·무장애물·고속·선회·전체 코스 및 동일 입력에서 음영
  발산 억제 근거가 반복됐다. 당시 기본 OFF는 후보 기각을 뜻하지 않았으며 유력한 채택 후보였다.
- 동시에 Smooth 정상 선회·10cm recovery 기준의 악화도 확인됐다. Global의 정확도와
  controller-facing alignment 손익을 나누고 필요하면 다음 단일 변수 실험으로 조정한다.
- 대회 LAN, 장시간 음영 정차, 실제 슬립/충돌 및 체크포인트 reset은 아직 검증하지 않았다.
  production 승격은 별도 승인 사항이며 이번에 alias/Q/R/heading/receipt-time/기본
  Smooth/IMU mask를 바꾸지 않았다.
- 원본과 실패 포함 결과는 같은 Git 제외 폴더에 보존한다. 최종
  `STUDY_SUMMARY.md/json`, `same_input_lateral_prior.png`, `imu_ay_diagnostic.png`가
  `baseline_ready/lateral_ready/imuay_ready`와 세 완주를 연결한다. 모든 실험 프로세스를
  종료했고 isolated master 24515도 닫았다. live master는 `/rosout`만 남으며 마지막
  실제 차량 P·정지와 prior OFF 상태를 확인했다.

## 수동 관찰용 어려운 전체 코스 준비 (PLAN / 주행 미검증)

사용자 요청으로 full_course/workspace에 `scenarios/full_loc_static_shadow_hard_01.json`과
`loc_fullcourse_static_shadow_hard_baseline.yaml`을 준비했다. 기존 full_course 예제 전체를
복사해 출발점/16개 경유점/목적지/600초를 유지하며 shared base와 기존 결과는 수정하지 않았다.

- 일반 정적 구간: 기존 corridor 예제 위치를 사용한 PE Drum, CargoBox, TrafficBarrel
  좌우 교대 3개. 객체 scale은 전부 1이며 샘플 CargoBox의 비정상 큰 scale은 가져오지 않는다.
- GPS 음영: `gps_shadow/base/scenarios/hid_soa_stress_21.json`의
  concrete-barrier/CargoBox/concrete-barrier 3개를 기존 위치·회전·scale 그대로 사용했다.
  UID는 전체 장면과 충돌하지 않게 분리했다. 공식 음영 영역은 그대로다.
- 고주로: spawn UID 6/8/10의 목표속도와 소속 배치 NPC 목표속도를 각각 35/55/75km/h로
  바꿨다. 초기 속도, spawn/종료 링크와 10초 생성 간격, 로터리 설정은 유지했다.
- 실행: **기존 localization**, 새 lateral prior OFF, od_gt, 신호등 OFF, Planviz ON,
  최초 준비는 영상 OFF였으나 사용자 관찰/녹화 요청에 따라 후속 개인 YAML 세 개는
  영상 ON(10fps, display `:0`), split 화면으로 통일했다.
  localization_analysis bag ON(카메라/LiDAR 녹화 제외), safety ON.
  candidate noise/global config는 전달하지 않는다. 관찰은 사용자가 실제 주행으로 수행한다.

정적 schema/UID/map spawn-link 검사와 CLI `--check`/`--dry-run`을 통과했다.
실제 회피 가능성·양쪽 회피·NPC 목표속도 도달·음영 체류는 아직 검증하지 않았다.
어려운 장면이므로 정지/실패도 원본 결과로 남기고 planning 정책을 바꿔 성공으로 맞추지 않는다.
개인 JSON/YAML/`metadata/full_loc_static_shadow_hard_01.json`은 Git 제외 대상이다.

사용자가 무객체 코스 대신 실제 미션 배치 상태의 측위/회피를 관찰하기로 해,
동일 JSON을 사용하는 `loc_fullcourse_static_shadow_hard_soft_lateral.yaml`을 추가했다.
`loc_fullcourse_static_shadow_hard_baseline.yaml`부터 실행한 뒤 같은 장면 후보를
새 실행으로 대조한다. 후보만 noise/global config 두 인자로 Global vy≈0 prior를 켜며
나머지 YAML 설정은 campaign 이름을 제외하고 동일함을 구조적으로 확인했다.
차량 9개, 보행자 1개, spawn 4개와 정적 장애물 6개를 포함한다. 기존 출발점/음영
영역은 원본과 동일하고 일반/음영 객체 scale은 모두 1이다. 두 YAML은 od_gt,
신호등 OFF, Planviz/영상 ON, 10fps split, bag/safety ON으로 설정 검사와 dry-run을
통과했다. 실제 새 미션 코스의 회피·완주는 아직 미검증이며, 객체가 있다고 앞선
차선 변경/최적화 문제를 해결했다고 간주하지 않는다. 자동 실주행/production 변경은
하지 않았고 사용자 수동 관찰이 다음 검증이다.

## 사용자 반복의 고주로 경로 붕괴 — 측위 후보와 분리한 통합 진단

MR, Competition heading 보정 `-1.347°`, receipt wall-time의 기존 계약을 유지한다.
`loc_fullcourse_soft_lateral_261015_024445`, `_024803`은 같은 무장애물 full-course,
od_gt, 후보 설정, Planviz/영상 ON 실행이다. 소스/설치 측위 SHA와 Planning/PP/PI
파라미터는 앞선 완주 후보 `_012811`과 동일하다. 원본 실행/수동 중단/종료 기록은
보존하고 실패 뒤 안전 정지 구간을 정상 주행 평가에 섞지 않는다.

현재까지 확인된 사실:

- 두 실행의 GPS invalid 표본은 0개다. 목적지 2점 경로 직전 4초의 독립 Ego 대비
  Global 절대 횡오차 p95는 각각 약 2.2cm다. 이를 GPS 음영 횡발산으로 분류하지 않는다.
- 51개 Route segment의 preferred/all primitive 배열은 두 실패와 앞선 기존/후보
  완주에서 같으며 각 bag에 route 메시지는 한 번이다. 최종 목적지로 새 route가
  요청된 현상이 아니다.
- 첫 경로 최적화 primal-infeasible는 bag+134.095 / 134.830초이며, 직전 최적화 경로가
  현재 허용 주행 영역 안에 없다는 재검사로 emergency brake가 먼저 나온다.
  이어 Normal Lane Change의 내부 경로 이탈 판정, 활성 모듈 해제, out-of-route
  fallback이 나타나 bag+134.963 / 135.596초에 최종 trajectory가 목적지 2점이 된다.
- 목적지 주변 경로는 `createGoalAroundPath()`가 만든 명시적인 zero-speed fallback이다.
  처음부터 목적지 좌표를 잘못 계산하거나 localization/제어 프로세스가 크래시한
  상황과 구분한다. `MANUAL_ABORT` 결과의 elapsed=0은 실제 주행 시간이 아니다.
- 완주 후보는 같은 map Y≈650m에서 약 28.42m/s, 두 번째 실패는 약 26.90m/s였다.
  따라서 단순 과속 하나로 재현 차이를 설명하지 않는다. 영상/RViz 부하는 조건 차이지만
  이전 영상 OFF 기준선에서도 같은 실패가 있었으므로 부하만을 원인으로 단정하지 않는다.

일회성 세부 timeline, geometry, 코드/최적화 진단은 Git 제외
`planner_logs/analysis/261015_fullcourse_route_collapse/`에 둔다. 주행 영역의 급격한
전환과 이전 경로/차선 변경 참조/최적화 제약 사이의 정합을 우선 확인한다. 다음 녹화에는
lane-change 원본 상태/참조 경로와 최적화 debug가 추가로 필요하다. 이번 분석을 근거로
도로/충돌 제약, out-of-route 판정, 제어 안전 정지를 끄거나 production 측위 설정을
바꾸지 않는다.

### 최초 최적화 실패 시점의 원인 분리

독립 GT는 recorded `map -> morai_map` static TF와 raw Ego heading `-1.347°`로
변환한 뒤 같은 시각에 보간한다. 준비/종료 구간이 아니라 첫 MPT 실패를 기준으로 한다.

| 첫 MPT 실패 시점 | 후보 `024445` | 후보 `024803` |
|---|---:|---:|
| bag 시작 후, s | 134.095 | 134.830 |
| 실제 속도, m/s | 28.21 | 26.82 |
| Global 횡측위 오차, cm | -1.15 | -1.94 |
| Smooth 횡측위 오차, cm | +2.31 | +3.12 |
| 실제 Ego와 당시 최종 trajectory 거리, cm | 21.0 | 19.3 |
| 실제 Ego와 새 Behavior path 거리, cm | 49.5 | 61.1 |

Global 종오차 59/48cm는 남아 있어 XY 정확도가 완벽하다는 주장은 하지 않는다.
그러나 이 횡/yaw 오차와 유효 GPS는 미터급 횡측위 발산이 먼저 발생했다는 해석을
지지하지 않는다. 새 Behavior/차선 변경 경로와 optimizer가 유지한 이전 trajectory가
서로 갈라졌고 실제 차량은 이전 trajectory 쪽을 따라갔다.

기하 진단에서 Global pose heading에 수직인 단면 폭은 첫 실패의 직전 frame에서
6.936→3.471m, 두 번째에서 6.952→3.484m로 바뀌었다. 왼쪽 경계는 지도
lanelet 201466의 왼쪽에서 201467의 왼쪽으로 바뀐 것과 수치적으로 일치한다.
두 lanelet 모두 recorded route segment 27의 primitive에 있지만 선택된 current lane
ID/모듈 내부 상태는 bag에 없어 그 선택 자체를 잘못됐다고 확정하지 않는다.

첫 실패 frame의 Global pose로 계산한 차량 사각형 footprint는 새 영역 안에 남아 있으나 좌측 여유는
약 19.9/5.0cm뿐이다. 같은 pose를 optimizer의 차량 covering circles로 표현하면
경계를 약 4.0/18.0cm 넘는다. 이는 **차량 근사 모델과 급격한 영역 전환의 민감성**
증거이지, 안전 margin을 줄여도 된다는 결론이나 이후 전체 경로가 충돌 없이
유효하다는 증명이 아니다. 기존 trajectory의 covering circle은 새 영역을 더 크게
넘어 비상 제동 재검사와 부합한다. 앞선 완주는 동일한 영역 전환이 실제 차량 기준
약 2.4/3.5m 앞서 발생했고 circle 여유가 약 11.7cm였다.

RViz/영상 OFF 완주 대비 ON 실패의 raw Ego rate는 약 48→42Hz로 낮았지만,
Global 수신 최대 gap <29ms, GPS 수신 최대 gap <90ms, GPS stamp age <4ms였다.
고주로 이동거리/속도 적분 비율도 실패 0.990/0.996으로, 과거 0.82 비실시간율
현상과 같다고 판단하지 않는다. 부하와 frame 도착 순서는 재현 조건이지만 단독 원인은
아직 입증되지 않았다. 다음 진단은 격리된 경로 최적화 replay의 제약별 solver 결과이며
실제 UDP·제어기·production graph에는 연결하지 않는다.

### 격리된 경로 최적화 replay 결과

별도 master 24541에서 원본 Behavior path, Global odom, static TF와 실행 당시
ElasticBand/PathOptimizer 파라미터를 사용해 기존 설치 binary를 1×로 재실행했다.
bag에 `path_smoother/path`와 `static_obstacle_scene`이 없으므로 ElasticBand를
재실행하고, known actor-free 장면의 empty scene을 재구성했다. 입력 경로 전체가
원래 PO 입력과 byte-identical하다는 주장은 하지 않는다. 그럼에도 첫 실패는 원본
`1792000229.834166`에 대해 replay `1792000229.831427`로 약 2.7ms 차이에서
재현됐고 `clear distance 0.00 < 51.10` 비상 제동 메시지도 일치했다.

동일 실패 QP의 제약 한 가족씩만 분석용으로 완화한 결과:

| replay 진단 | solver 결과 |
|---|---|
| 원래 제약 전체 | primal infeasible (-3) |
| hard 도로 경계 제약만 제외 | solved (1) |
| 과거 fixed-state equality만 제외 | primal infeasible (-3); 해당 frame fixed count=0 |
| 차량 운동/전파 equality만 제외 | solved (1) |

이는 **hard 주행 영역 제약과 차량 운동 제약의 동시 만족 실패**를 직접 지지한다.
최초 frame의 과거 fixed-state equality 충돌 가설은 기각한다. 바뀐 경계와 차선 변경
참조/시간 정합/차량 근사 여유의 어느 생성 단계가 근본 결함인지는 추가 디버그 입력으로
분리해야 한다. 이 진단은 제약을 제거한 해를 실제 출력에 적용하지 않는다.
`AJOUNICE_MPT_QP_DIAG` probe는 로그용이며 원래 제약의 출력/안전 제동을 유지한다.
정책/config/source 변경·재빌드·실제 주행은 하지 않았고, private replay 프로세스는
모두 종료했다. artifacts는 `po_replay_024803_1x/summary.json`, `po.log` 및
`corridor_first_failure.png`다.

## 미션 배치 기준선 `031947` — 일반 정적 구간 정지와 로컬 시간축 혼합

`loc_fullcourse_static_shadow_hard_baseline_261015_031947`은 사용자 실제 주행이며
localization 기존 설정, od_gt, 신호등 OFF, Planviz/영상 ON이다. 원본 bag은
`error/full_loc_static_shadow_hard_01_20261015_031949_01/`에 보존됐고
약 146.99초, MANUAL_ABORT, 충돌 0으로 종료했다. 음영에 도달한 결과가 아니다.

- 최종 실제 Ego map XY `(71.191,381.051)m`에서 정지했다. 첫 PE Drum은
  `(70.649,386.661)m`, size `0.49×0.49×0.80m`로 raw Object와 prediction 모두에
  존재했다. 객체 미수신/미인지 정지가 아니다. 중심까지 약 5.6m다.
- Planning 로그 `1792002060.645`에 newest static obstacle이 기존 출력을 무효화해
  같은 geometry로 제동하고, `2060.779`부터 MPT 회피 후보 최적화 실패가 반복됐다.
  정지 후에도 이전 경로의 clear=1.20m/stop=0.20m 제동을 유지했다. 마지막
  `/planning/trajectory`는 177점 모두 속도 0이다. 고주로의 먼 목적지 2점 fallback과
  다른 정지다. 코드 crash는 관측하지 않았다.
- 첫 MPT 실패 시 보간한 독립 Ego 대비 Global XY/종/횡오차는
  `3.296m / +3.296m / +0.014m`, Smooth는 `3.498m / +3.498m / -0.002m`다.
  이후 10초 Global XY p95 약 6.44m로 악화했고, 정차 안정화 뒤에는 최종 XY 약
  7cm로 수렴했다. 정차 뒤 정확도를 전체 주행 정확도로 쓰지 않는다.
- 첫 MPT 실패 기준 직전 `[-12,-2]s`, `[-3,0]s`, 이후 `[0,7]s`의 실제 Ego
  이동거리/receipt wall-time GT vx 적분 비율은 `0.836/0.798/0.727`이었다.
  raw Ego는 약 `29.2/27.1/24.9Hz`, GPS는 `14.9/13.5/12.4Hz`로 낮았고 GPS
  header-receipt p95는 `1.7/0.6/1.5ms`다. zero GPS 표본은 0개다.

이번 정지의 직접 동작은 회피 최적화 실패에 대한 planner 제동이다. 그러나 주행 중
큰 종측위 오차와 position-speed 시간축 불일치도 실제 존재하므로, 앞선 고주로 bag처럼
"측위는 정상이고 planner만 문제"라고 단정하지 않는다. 이 비율은 과거 로컬
비실시간 MORAI+receipt-time 과대 적분 현상과 부합하지만, MORAI `/clock` 없는
bag만으로 정확한 simulator time이나 GPU 단독 원인을 확정하지 않는다. timestamp
정책/velocity scale/production Q/R은 변경하지 않는다.
회피 불가능한 배치인지 planner 구현 결함인지도 아직 구분하지 못했다. 같은 장면의
GT pose 대조와 카메라/LiDAR **생성 부하**를 줄인 localization+od_gt 반복이 원인
분리의 다음 후보이며, bag에서 카메라/LiDAR를 제외하는 것과 생성 중단은 다르다.

## 최신 무객체 사용자 bag — Global와 Smooth 손익 분리

사용자 요청의 error baseline을 감사해 `loc_fullcourse_baseline_261015_023536`과
최신 `loc_fullcourse_soft_lateral_261015_034133`을 주 비교로 사용했다. MR/Competition
heading `-1.347°`/receipt-time, 기존 σ=0.10m/Qxy=0.0005/Smooth 0.5/0.5s,
동일 actor-free scene SHA `a20d99b7…`, od_gt, RViz/영상 split 10fps는 같다.
후보만 Global-only soft vy prior가 실제 ON이다. 단일 publisher와 원시 GPS/IMU,
Global/Local/Smooth/TF/초기 pose 녹화 및 Park 종료 audit를 확인했다.
실제 취득 시각/입력/주행은 다르며 동일 입력의 인과 replay는 아니다.

### 실행 식별 주의

- baseline `023536`: ERROR/MANUAL_ABORT지만 실제 종료 전 차량은 약 16.9m/s로
  주행했다. 최종 경로도 차량 근처에 남았고 목표속도 약 17m/s/accel>0/brake=0이다.
  감속 과도 warning은 있지만 원거리 목적지 2점 경로나 MPT 실패/route 이탈은 없다.
  따라서 "두 bag 모두 같은 방식으로 고주로에서 정지"라고 쓰지 않는다.
- soft `034133`: 횡가속도 과도 warning 후 약 0.82초 뒤 route 이탈과 원거리 목적지
  2점 경로가 발생했다. 이 실행에는 앞선 `024803`의 MPT primal-infeasible 로그가
  없어 같은 최초 원인으로 단정하지 않는다. 첫 지속 soft-stop 직전 Global 횡오차
  약 -0.8cm, 종오차 -38.2cm였다.
- 최신 baseline `023918`은 151.14초 완주 PASS인 별도 보조 기준선이다.
- 앞선 baseline `011433`에는 실제 고주로 원거리 2점 경로/정지가 있지만 당시
  RViz/영상 OFF라 현재 두 bag과 동일 부하 A/B로 묶지 않는다.

### 정상 주행 공통 prefix 비교

첫 soft-stop까지의 실제 Ego 누적 주행거리 약 1,266.8m에 맞춰 baseline도 자른다.
시작은 각 CONTROLLER_START, 정상 이동 표본은 GT 속도>0.5m/s이며 최초 SceneLoad,
planner 정지 이후 및 shutdown 구간은 제외한다. 보정된 raw Ego를 co-time 보간해
독립 위치 오차를 계산한다. 아래는 절대 mean/p50/p95, **단위 cm**다.

| 위치 오차 | 기존 error baseline `023536` | soft `034133` |
|---|---:|---:|
| Global XY | 23.25 / 22.09 / 42.11 | 24.82 / 25.26 / 37.83 |
| Global 횡 | 2.11 / 1.74 / 5.34 | 1.23 / 0.95 / 3.32 |
| Global 종 | 23.03 / 21.92 / 42.05 | 24.77 / 25.18 / 37.82 |
| Smooth XY | 21.05 / 19.56 / 41.06 | 24.10 / 24.01 / 36.95 |
| Smooth 횡 | 1.43 / 1.10 / 4.00 | 2.54 / 1.25 / 9.17 |
| Smooth 종 | 20.93 / 19.43 / 40.93 | 23.83 / 23.83 / 36.94 |

Global 횡 정확도 개선과 Smooth 횡 tail 악화가 동시에 재현됐다. XY 평균/p50도
일괄 개선이 아니므로 Global 이득만으로 candidate를 production 승격하지 않는다.
GT 누적거리 대응 위치 차이 p95는 약 39.5cm여서 동일 경로/입력의 완벽한 통제 실험이
아니다. controller-facing 손익은 다음 동일 입력 Smooth 정합 분석에서 확인한다.

두 비교 window의 실제 거리/vx 적분 비율은 0.9948/0.9985로 거의 일치한다.
종오차는 대체로 실제보다 뒤쪽이며, 정적 미션 bag의 0.73~0.84 비율과 수 m 전방
과대 적분이 재현됐다고 해석하지 않는다. Global/Smooth는 약 50Hz, raw Ego는
약 39.75/41.69Hz, GPS는 23.33/24.47Hz다. 두 주행은 zero-GNSS 표본 0개이며
GPS 최대 header gap 약 0.10초로 **실제 음영/재획득 성능 비교는 불가능**하다.
UDP header는 receipt-time이므로 header-receipt ms 차이를 원래 MORAI simulator
clock 지연으로 잘못 해석하지 않는다.

전체 구간 보조 분석, 실행/녹화 audit, 독립 시간축/거리 대응 script와 JSON은 Git 제외
`planner_logs/analysis/261015_actor_free_localization_pair/`에 모았다. 이 분석 단계에서는
production 코드, Q/R, heading, UDP 시간 정책, Smooth 설정을 변경하지 않았다.

## 사용자 승인 승격 및 localization-only Smooth 개선

사용자가 soft의 즉시 production 승격과 **제어기 튜닝이 아닌 localization 내 개선**을
승인했다. 기반 branch/HEAD는 위 실행 계약과 같고, 변경은 아직 worktree에 있다.
기존 Global-only prior의 장애물 폐루프 근거를 유지하면서, 정상 선회의 Smooth tail과
재획득 복귀 지연을 동일 입력으로 개선했다. 이번 추가 검증은 새로운 MORAI 폐루프가 아니다.

### 현재 구현 계약

| 항목 | 승격·개선 기본값 |
|---|---|
| Global soft body `vy≈0` prior | ON, σ `0.20m/s`, 별도 topic; Local EKF 변경 없음 |
| Smooth 종/횡/yaw tau | `0.5 / 0.1 / 0.5s` |
| Smooth correction 기준점 | 현재 차량 pose(vehicle pivot) |
| 횡 correction rate cap | `0.3m/s`; 실제 차량 횡움직임·조향 속도 제한이 아님 |
| Planning / TF | 기존 Global state |
| Controller | 기존 `/localization/control_kinematic_state` 메시지·frame |
| 유지 | GNSS σ=0.10m, Qxy=0.0005, IMU ay OFF, heading, receipt-time, controller gains/속도 정책 |

noise profile와 Global overlay를 로드한 **뒤** launch 인자가 prior publisher와 Global
vy mask를 함께 설정한다. 구형 custom YAML도 새 기본값을 적용받으며,
`use_lateral_velocity_constraint:=false`는 발행기와 mask를 함께 OFF한다.
Competition 미제공 `vel_y=0`을 실측으로 쓰지 않고, 기존 GT orientation/twist는
prior ON/OFF에서 byte-identical이다. Global 내부 ay를 직접 0으로 고정하는 변경도 아니다.

기존 Smooth의 같은 translation/yaw tau=0.5s에서는 odom 원점 선택에 따른 artifact가
p95 약 0.3mm여서 9cm tail의 원인으로 보지 않는다. 그러나 tau를 따로 튜닝하면
origin-based 회전·이동 필터가 원점에 민감해질 수 있다. 합성 원점 이동 `(1000,500)m`에서
translation/yaw tau=0.1/0.5s로 단순 변경한 경우 약 27.7cm 차이가 생겼다.
따라서 unequal tau를 적용하기 전에 차량 기준점에서 correction을 계산하도록 정리했다.

계산은 다음과 같다. `L_t`는 Global stamp까지 co-time 외삽한 Local pose, `G_t`는 Global,
`A_(t-1)`는 이전 control 정합이다.

```text
예측 control pose = A_(t-1) ∘ L_t
correction = G_t.xy − 예측 control pose.xy
Global yaw 기준으로 correction을 종/횡 성분으로 분해
각 correction step = [1 − exp(−dt/tau)] × 해당 correction
횡 step만 |step| ≤ 0.3 × dt로 제한
yaw 정합은 기존 tau=0.5s 유지
보정한 현재 차량 pose에서 새 control 정합 A_t를 역산
```

즉 Local 운동을 지연시키는 것이 아니라 **Global 보정의 횡성분을 덜 늦게 받는다**.
reset 기준 `0.5m / 20deg`, gap `0.5s`, Local 최대 age `0.05s`는 유지한다.
Global callback이 Local message를 한 번만 snapshot하여 age/외삽/twist 사이 thread
경합을 막았고, NaN/Inf pose·외삽 입력이 재귀 정합을 오염시키지 않게 거부한다.
출력은 Global stamp/frame/z/covariance와 Local twist를 유지한다. inherited covariance는
새 Smooth의 통계적 오차를 재보정한 covariance가 아니다.

### 5개 bag 동일입력 비교

모두 Global soft ON의 **같은 recorded Global/Local/Ego**에 구형 Smooth와 현재 소스
알고리즘을 적용했다. 아래는 이동 표본의 독립 Ego 대비 **Smooth 절대 횡위치 오차 p95,
단위 cm**이며 Global 필터나 실제 차량을 다시 주행한 결과가 아니다.
이동은 GT 속도>0.5m/s, 정상 선회는 여기에 |IMU yaw-rate|>0.15rad/s이면서 실제
음영 및 재획득 첫 1초를 제외한 표본이다. 원본 controller start/end와 유효 oracle
구간만 사용하고 초기 ScenarioLoad·shutdown·GT gap을 넘지 않는다. 앞 단계의
별도 replay/phase 창과 수치가 조금 다르면 이 조건을 먼저 비교한다.

| bag / 실행 | 구형 Smooth | 개선 Smooth | 정상 선회 구형 → 개선 |
|---|---:|---:|---:|
| 무객체 최신 중단 `034133` | 9.13 | 4.21 | 10.39 → 5.02 |
| 무객체 완주 `012811` | 7.70 | 5.85 | 8.47 → 4.10 |
| 박스 1개 `004111` | 8.43 | 7.03 | 9.31 → 4.94 |
| 박스 2개 `002129` | 8.38 | 6.11 | 8.87 → 4.63 |
| 팀장님 E24 동일입력 soft replay | 11.08 | 7.78 | 11.42 → 5.88 |

- 종오차 p95는 전체 코스 bag에서 거의 동일(차이 약 0.01cm 수준), 짧은 박스 bag에서는
  약 0.5~0.6cm 악화했다. yaw는 동일했고 추가 reset은 5개 모두 0회였다.
- 횡 motion-residual p95는 약 0.05~0.07cm 증가했다. 최고값도 0.38~0.98cm에서
  0.54~1.13cm로 증가했다. **정확도와 모든 순간의 부드러움이 동시에 개선된다고 쓰지 않는다.**
- E24 재획득 뒤 횡오차가 10cm 이내로 1초 유지되기 시작하는 시간은
  구형 soft Smooth `1.185s → 0.199s`였다. 같은 raw 입력의 prior-OFF Global `0.021s`와
  서로 다른 비교임을 주의한다. 이는 측위 회복이지 실제 조향/차량 회복 시간은 아니다.
- 횡 cap=0.2m/s는 E24 p95 약 10.25cm로 너무 느렸고, 0.5m/s는 큰 frame 변화가 증가했다.
  선택한 0.3m/s는 이 비교의 절충값이지 다른 차량·슬립·LAN에서 보편적인 최적값이 아니다.

actual source의 `PlanarAlignmentSmoother`와 독립 prototype은 5개 bag 모두 pose 차이
최대 `5.8e-13m`, yaw `5.6e-16rad` 이내에서 일치했다. 구형 재구성도 실제 기록 Smooth와
p95 약 nm~0.77mm 차이였으며, callback 도착 순서 차이에 따른 replay 한계를 보존했다.
원본 실행·소스 snapshot은 변경하지 않는다.

재현과 전체 mean/p50/p95/count/phase/reset은 Git 제외
`planner_logs/analysis/261015_soft_production_smooth_improvement/`의
`evaluate_candidates.py`, `candidate_results.json`, `selected_candidate_summary.json`,
`candidate_series.npz`, `smooth_candidate_lateral_comparison.png`에 있다.

### 설치·검증 및 다음 실주행

`localization_manager`, `bringup`, `planning_scenario_validator`만 증분 설치했다.
installed roslaunch parameter dump에서 기본 prior ON·σ=0.20·Global vy mask와
vehicle-pivot/횡 tau=0.1/cap=0.3을 확인했다. 명시적 prior OFF + legacy Smooth도
publisher OFF/Global mask all-false로 확인했다.
새 계약·원점/회전 불변성·rate cap·reset·비유한 입력·기존 메시지 호환성을 단위 검증했다.
localization 일반 단위 76개 및 원래 ROS runner의 transport 2개, Validator 측위 계약
69개, 녹화 옵션/가상 화면 GUI 17개, GPS 음영 분석기 12개가 모두 통과했다.
격리 master의 설치본 Smooth 20초 transport에서는 입력 각 1,000개에 출력 1,000개,
50.005Hz, 최대 header gap 23.58ms, stamp/frame/z/covariance/twist 보존과 quaternion을
확인했다. source/install SHA도 일치했고 UDP·planner·controller는 실행하지 않았다.
transport receipt는 `planner_logs/analysis/261015_soft_production_smooth/installed_transport_smoke/`다.

```bash
# 새 production: 별도 추가 인자 없이 기존 alias
ajou
localization

# 과거 prior-OFF + 구형 Smooth 비교/rollback에만 사용
localization use_lateral_velocity_constraint:=false control_state_use_vehicle_pivot:=false
```

Validator도 `stack.localization_args: {}`이면 새 production이다. 과거 baseline을
재현하려면 두 OFF 인자를 명시하고 새 실행을 만든다. 기존 결과에 바뀐 소스로 강제
resume하지 않는다. 다음은 새 Smooth로 무객체 → 음영 박스 → 일반 선회/고속을 실제
od_gt 폐루프에서 확인하는 것이다. LAN·장시간 음영 정차·고속 슬립·MRP·checkpoint는
미완료다. 알려진 고주로 경로 붕괴와 로컬 MORAI 부하의 종방향 시간축 문제는 이 변경으로
고쳤다고 하지 않는다. 판단/제어 정책과 안전 종료는 변경하지 않았다.

새 실주행용 개인 사본(공유 base 변경 없음):

- `full_course/workspace/loc_fullcourse_soft_production.yaml`: 무객체 전체 코스.
- `full_course/workspace/loc_fullcourse_static_shadow_hard_production.yaml`: 기존 정적/음영 미션 배치.

둘 다 새 campaign 이름과 `resume=false`, 현재 prior/Smooth 인자를 명시했다.
od_gt·신호등 OFF·Planviz/영상 ON·10fps split·측위 bag/safety ON이다. 장면·목적지·
경유점·판단/제어 정책은 각 원본과 같다. `--check`/`--dry-run`은 통과했지만 아직
실제 실행하지 않았다. 기존 과거 baseline 사본은 명시적 OFF로 보존했다.
`localization_analysis.json`에도 prior topic을 추가해 새 기본값의 발행·녹화 count를
확인할 수 있게 했다. prior-OFF 대조에서는 이 토픽의 0 count가 정상이다.

### 사용자 관찰용 음영 미션 표시 옵션 통일

사용자 요청으로 `gps_shadow/base/` 예제 6개와 현재 개인 `workspace/` 캠페인 12개의
RViz/영상 ON, 10fps, split, focus_planviz=false, bottom_bar_px=auto를 맞췄다.
기존 명시 display는 유지하고 누락된 display는 `:0`을 설정했다. 후보 config YAML은
대상이 아니며 장면·미션·측위 인자·bag preset·판단/제어 정책·안전 gate는 그대로다.
표시 옵션을 제외한 parsed YAML의 동일성을 비교했고 18개 `--check`를 통과했다.
공유 예제와 개인 사본을 구분하며 원본 실행 snapshot/결과는 수정하지 않았다.
영상/RViz OFF였던 과거 폐루프와 지금 ON 주행을 같은 부하 A/B로 합치지 않는다.
GUI에서는 수정한 YAML을 다시 불러오고 새 실행으로 관찰한다.

## 새 production의 사용자 폐루프 — 무객체 전체 코스와 음영 pair

같은 날 사용자가 취득한 전체 코스 `044421/045214`와 pair `045709/050237`을
분석했다. 모두 localization → od_gt → tracking/Map-CV → ROS1 planning →
기존 PP/PI → MORAI UDP의 실제 폐루프다. 이번 분석에서는 새 MORAI 주행·제어
송신을 시작하지 않고, 추가 원인 분리만 격리 estimator replay로 수행했다.
MR, Competition heading `-1.347°`, receipt-time, GNSS σ=0.10m/Qxy=0.0005,
soft σ=0.20m/s, 새 Smooth 0.5/0.1/0.5s·횡 cap=0.3m/s 계약을 확인했다.

### GT 위치로 바뀐 것이 아님

- Runtime `pose_source=localization`, motion adapter `source=competition`이고
  GT pose provider는 실행되지 않았다.
- `/localization/initialpose`는 실행당 1개다. Ego **위치만 초기화**에 사용하며
  초기·연속 yaw와 vx는 Competition에서 받는다. 실제 두 controller도 Smooth topic을 구독했다.
- Global/Local/Smooth bag connection의 유일한 publisher가 각각 실제 EKF·state
  publisher·Smooth 노드였다. source/installed/실행 receipt의 SHA도 일치했다.
- pair 음영 중 `/localization/gnss/odom`은 0개였지만 prior는 561/601개 계속 들어왔다.
  유효 GNSS가 몰래 들어오거나 운영 중 GT position/reset을 반복한 결과가 아니다.
- 모든 prior header stamp가 raw Competition에 존재했고 body `base_link`, vy=0,
  covariance[7]=0.04였다. 실제 prior cadence는 source callback을 따르며 독립 50Hz
  타이머가 아니다. ROS publisher는 출력 topic의 header.seq를 별도로 부여하므로
  raw source와 seq 차이만으로 손실·시간 오류를 판정하지 않는다.

### 측위 정량 결과

독립 oracle은 raw Ego를 recorded static map↔morai_map TF로 변환하고 MR heading
offset을 한 번 적용했다. controller 시작부터 run 종료까지, GT 속도>0.5m/s,
유효 co-time oracle 표본만 평가했다. 아래는 **단위 cm, 절대 mean / p50 / p95**다.
일반 정지·ScenarioLoad teleport·shutdown을 제외했다.

| 실제 주행 | Global 횡 | Controller Smooth 횡 | Controller Smooth XY | Controller Smooth 종 |
|---|---:|---:|---:|---:|
| 전체 코스 `044421` | 1.70 / 1.17 / 5.58 | 1.94 / 1.22 / 6.44 | 27.37 / 26.45 / 58.13 | 27.22 / 25.97 / 58.08 |
| 전체 코스 `045214` | 1.82 / 1.16 / 6.88 | 2.09 / 1.23 / 8.12 | 26.50 / 25.32 / 58.94 | 26.31 / 25.07 / 58.91 |
| 음영 pair `045709` | 2.94 / 2.91 / 5.00 | 3.23 / 3.26 / 5.11 | 14.69 / 15.83 / 21.57 | 14.10 / 15.37 / 21.39 |
| 음영 pair `050237` | 2.44 / 2.46 / 5.82 | 2.70 / 2.73 / 5.71 | 19.10 / 22.19 / 24.43 | 18.67 / 21.92 / 24.16 |

**XY 전체가 수 cm**라는 결과는 아니다. 안정된 횡방향과 남은 종방향 오차를 구분한다.
Smooth yaw p95는 같은 순서로 약0.071/0.065/0.075/0.074deg이며, 4회 모두 state 약50Hz,
충돌0·목적지PASS였다. validator 주행시간은151.07/150.95/31.81/32.65초다.
주행시간과 event 사이의 시작·도착 hold를 포함한 평가창 길이는 다른 값이다.

### 실제 GPS 음영과 회피

| 실행 | raw GPS invalid 시간 | Global 횡 p95 / 최대 | Smooth 횡 p95 / 최대 | 실제 Ego→당시 계획 궤적 횡 p95 |
|---|---:|---:|---:|---:|
| full `044421` | 7.270초 | 9.33 / 12.84 | 9.36 / 12.90 | 13.92 |
| full `045214` | 7.242초 | 10.89 / 14.38 | 10.89 / 14.42 | 14.78 |
| pair `045709` | 11.918초 | 5.01 / 5.30 | 5.01 / 5.30 | 12.51 |
| pair `050237` | 12.844초 | 6.61 / 6.79 | 6.60 / 6.75 | 8.20 |

위 표의 위치 오차 단위는cm다. pair는 무객체 full보다 약5초 길게 GNSS 없이
회피했지만 수 m급 횡발산이 재현되지 않았다. 최신 pair의 Global vy는
약−0.00286→0m/s로 수렴했고 내부ay 절대p95는0.00210m/s², vy covariance는 약0.005였다.
각 phase의 실제 이동거리/vx 시간 적분 비율은 거의1.0으로, 이전 로컬 과부하 bag의
0.73~0.84 시간축 불일치와 다르다. 이를 production velocity scale로 적용하지 않는다.

pair의 두 물체는 좌우에 배치됐지만 실제 Ego와 planned path는 모두 물체 왼쪽
약2.2m를 통과했다. 실제 조향을 동반한 회피 검증은 맞지만 **좌우 양방향 연속 회피**로
부르지 않는다. actor yaw 기준의 통과 거리와 lane-center 오차도 구분한다.
정상 회피 변위와 estimator−Ego 오차를 같은 drift로 세지 않는다.

### 원인 분리: 동일 입력 A/B

새 pair `050237` raw 입력을 원본 post-ScenarioLoad 초기pose와 완전히 일치하는
시점에서 gate하고 source header/payload를 보존해1× replay했다. static TF8관계도 보존했다.
양쪽 모두 **같은 개선 Smooth**이며 Global-only vy prior ON/OFF만 바꿨다.

| 같은12.844초 GPS 음영 | prior OFF | prior ON |
|---|---:|---:|
| Global 횡 p95 | 56.44cm | 6.59cm |
| Smooth 횡 p95 | 55.82cm | 6.58cm |
| 내부vy 시작→끝 | −0.02710→−0.07097m/s | −0.00277→약0m/s |
| 내부ay | −0.003422m/s²로 거의 고정 | +0.001253m/s²에서 약0으로 수렴 |

OFF의 vy 기울기−0.003422m/s²는 ay와 일치했다. 기존 IMU·Competition 정보에
soft prior를 더한 ON은 미관측 내부 ay/vy의 지속 적분을 억제해 횡p95를 약88% 줄였다.
따라서 "IMU/GT만으로 이미 안정적이었다"거나 "GT 위치를 직결했다"는 해석은 아니다.
초기pose 전체field/covariance, raw payload/header5topic SHA가 ON/OFF에서 일치했다.
ON replay와 실제 ON 원본의 XY 차이p95는 Global4.13mm, Smooth1.59mm였으며,
최대값에는 callback 도착 순서 차이도 남는다. 같은 Ego 움직임에서 estimator 효과를
분리한 비교이지, OFF 조건의 실제 차량을 다시 주행한 결과가 아니다.
24535/24536 master는 안전 종료했으며 실제11311·UDP·controller에 연결하지 않았다.

### 전체 코스 손익과 남은 한계

같은 무객체scene·od_gt·RViz/영상10fps에 controller source/install binary/config SHA도
같은 과거 baseline `023918`(prior OFF+구형Smooth)을 보조 비교했다. 입력·physics·
초기 상태가 달라 엄밀한 동일 입력 A/B는 아니다.

- 전체 Smooth 횡mean/p50은2.43/1.67→2.09/1.23cm로 감소했다.
- 반면 횡p95는6.70→8.12cm로 약1.42cm 악화했다. 첫 반복에서는6.44cm였으므로
  모든 구간·통계가 일괄 개선됐다고 결론 내리지 않는다.
- 음영 Smooth 횡p95는21.49→10.89cm, 재획득 뒤10cm 이내를1초 유지할 때까지의
  시간은0.758→0.286초였다. 재획득 첫1초 실제Ego→궤적p95도54.69→41.27cm였다.
- steering-rate mean/p95는0.0704/0.3438→0.0664/0.3144rad/s였다. 완주시간은
  151.14→150.95초로 거의 같으며0.19초를 유의한 속도 개선으로 해석하지 않는다.
- 최신 pair도 재획득 뒤 곡선에서 실제Ego→궤적p95 약37.9cm가 남았다.
  같은 phase의 Smooth 측위 오차 수cm와 구분하고 전부 GPS drift로 부르지 않는다.
- pair `045709`는 RViz OFF·영상ON, `050237`은 RViz/영상10fps split ON이다.
  full `044421`은 궤적 시각화 노드가 조기 종료했고 `045214`는 정상 종료까지 유지됐다.
  반복의 부하 차이를 숨기거나 같은 조건으로 묶지 않는다.

최신 pair/full의 Global/Local·GT 혼입·body/map 축·heading 이중보정·단위·TF 계약에서
새 critical 불일치는 발견하지 못했다. 무잡음이어도 Competition 절대yaw/vx 보조와
vy 모델prior를 쓰는 방식이며 IMU-only나 SLAM은 아니다. vy≈0은 map Y 고정이 아니라
차체 왼쪽 slip이 작다는 가정이므로 yaw 변화에 따라 정상적인 map X/Y 회피 운동이 가능하다.
새 실제 폐루프와 동일입력 원인 분리 근거가 충분해 이번에는 추가 MORAI 주행이나
Q/R·heading·timestamp·제어 정책 조정을 하지 않았다.

판정은 **현재 production 유지**다. 다음 gate는 실제 반대 방향 회피, GPS 음영 중
장시간 정지·재출발, LAN-to-LAN/대회PC, slip/충돌·checkpoint다. 새4회 성공을
이 gate의 완료로 확대하지 않는다.

재현/artifact는 Git 제외 `planner_logs/analysis/261015_soft_smooth_closed_loop_validation/`
아래 `CLOSED_LOOP_SUMMARY.json/md`, `fullcourse/`, `pair/PAIR_SUMMARY.json`,
`causal_replay/CAUSAL_SUMMARY.json`·원인 비교 그림,
`contract/LOCALIZATION_NOT_GT_AUDIT.md`에 모았다. 원본bag·실행config·snapshot은 보존했다.

### 일반 주행의 세 단계 동일입력 비교

최신 팀장님 E24 원시 입력을 원본 post-ScenarioLoad 초기pose와 맞춰 재생한
`baseline_ready/lateral_ready`와, 같은 soft Global/Local 위의 새 Smooth 출력 재구성을
비교했다. **세 번 새로 주행한 폐루프가 아니라 estimator replay + 출력 counterfactual**다.
원본 CONTROLLER_START/RUN_FINISHED 창은 header stamp
`1791908615.145~1791908777.468`이며, MR/Competition, heading −1.347°,
원본 receipt-time 입력, 1× replay 계약을 유지했다. 단계2와3의 soft σ는 모두
0.20m/s다. 최종 단계에서 σ를 새로 바꾼 비교가 아니다.

아래는 Controller Smooth의 **절대 횡오차, 단위 cm, mean / p50 / p95**다.
정상 GNSS 이동은 GT |vx|>0.5m/s에서 실제 GPS invalid/stale와 재획득 첫1초를
제외했다. 정상 선회는 여기에 |raw IMU yaw-rate|>0.15rad/s를 적용했다.

| 같은 입력의 평가 구간 | soft OFF + 구형 Smooth | soft ON + 구형 Smooth | soft ON + 개선 Smooth |
|---|---:|---:|---:|
| 정상 GNSS 이동 | 1.913 / 1.438 / 5.353 | 2.979 / 1.865 / 9.783 | 1.724 / 1.289 / 4.923 |
| 정상 선회 | 2.932 / 2.431 / 6.898 | 5.442 / 4.754 / 11.452 | 2.636 / 2.257 / 5.880 |

정상 이동 표본 수는7502/7504/7504, 정상 선회는2434/2435/2435다.
공통 최종 header grid에 OFF를 보간해도 p95는 정상5.356cm/선회6.937cm로
거의 같았다. 기존 "선회7.0→11.9cm" 비교에는 음영·재획득이 포함됐으므로
위의 정상 선회와 혼용하지 않는다. 전체 이동의 종오차 p95는 세 단계
56.58/56.57/56.54cm로 거의 같았다.

판정은 **soft만 추가한 중간 단계의 일반 주행 손해를 개선 Smooth가 회복했고,
이 입력에서는 pre-soft보다 정상 횡p95도 소폭 개선**이다. 각각 약8%/15% 감소하며,
별도 실제 주행 전체의 모든 p95가 일괄 개선됐다는 뜻은 아니다. 앞의 실제 전체 코스
횡p95 6.70→8.12cm 보조 비교와 함께 해석한다. 구형 Smooth 재구성과 recorded 출력의
차이p95는0.768mm이며 replay callback 순서의 작은 차이는 남는다.

세 단계 원본 수치와 gate는 같은 분석 폴더의
`E24_THREE_STAGE_COMPARISON.json`에 보존했다.

### 다음 개선 대상: 종방향 원인 분리

새 전체 코스의 종오차 mean/p50/p95는 Global에서
`27.57/26.25/58.78cm`, `26.55/25.39/58.62cm`였고, Smooth에서
`27.22/25.97/58.08cm`, `26.31/25.07/58.91cm`였다. 두 출력이 거의 같으며
signed mean은 약−26~−27cm로 평균적으로 oracle보다 뒤에 있다.
따라서 잔여 종오차를 Smooth tau=0.5s 하나로 설명하지 않는다.

최신 실제 주행은 이동거리/속도 적분 비율이 거의1이고 state 약50Hz이므로,
과거 큰 MORAI 비실시간율이나 임의 vx scale 보정의 근거가 아니다.
입력의 물리 샘플 시각·가감속 모델 응답·위치/속도 기준점 중 어느 성분이
얼마를 기여하는지는 아직 분리하지 않았다. 다음 후보는 기존 bag의 속도별,
가속/정속/감속별, GNSS 정상/음영별 종오차와 Global/Smooth co-time 비교 및
oracle lag 진단이다. GT는 평가용으로만 사용하고 임의 pose offset이나
UDP receipt-time 변경을 production에 넣지 않는다. 장시간 음영 정차·재출발,
LAN-to-LAN·슬립·checkpoint gate는 별도로 남아 있다.

## 최종 hardening: 장기 정차·checkpoint·종방향 epoch (2026-10-15)

판정: **기존 soft/Smooth production 유지.** 추가 ZUPT는 보류, heuristic checkpoint reset은
OFF/monitor-only, GNSS motion projection은0초가 기본이다. 대회 LAN·공식 reset 이벤트·
충돌/slip을 전부 해결했다는 뜻은 아니다. branch `dataset/base5`, 기반 runtime
`d834e2ad` + 이번 source, 규정 문서 commit `d376ed1c` 이후 실행을 구분한다.
case별 source/installed SHA·명령·parameter·snapshot·events가 날짜보다 우선한다.
Q/R·heading·UDP receipt-time·Local EKF·detector/판단/제어 정책은 변경하지 않았다.

### 장시간 GPS 음영의 실제 정차·재출발

`260919_231906...`의 실제 음영27.75초·정차24.59초를 현재 estimator로 재생했다.
같은 초기화/입력에서 Global 횡p95는 prior OFF16.52m 대 ON8.41cm였다.
마지막 실제 정차 센서를 추가60초 유지한 **합성 모델 진단**의 누적 Global 변화는
ON0.021cm 대 OFF144.62m였다. 합성 구간에 GT 위치를 만들어 넣지 않았다.
최신 pair에 합성60초 정차를 삽입하고 원래 입력을 재개한 clean replay도
Global/Smooth 정차 변위0.062/0.589cm,50Hz였다. 재획득1–5초 후 unpaused estimator와의
XY 차이p95는0.81/0.70cm였으며 이 차이는 GT accuracy가 아니다.

실제 `loc_final_shadow_hold_261015_062342`는 기존 PP/PI 주행 중 음영→controller 종료→
stock brake/P·실제 정지 확인→30초 Park→같은 PP/PI 재개다. 별도 직진·정속 제어기가 아니다.

| 실제 hold·재출발 지표 | 결과 |
|---|---:|
| 총 GPS invalid / Park hold | 50.246 / 30.068초 |
| hold raw GPS / 유효 GNSS odometry | 815개 전부zero(status1) / 0개 |
| Global/Smooth hold rate / 최대 header gap | 약50Hz / 25.57ms |
| hold 횡 / XY 오차 p95 | 9.80 / 26.66cm |
| hold 시작·끝1초 median 오차 변화 | XY0.00119cm, 횡0.00115cm |
| hold 종편차 | 약−24.79cm: 기존 offset은 남지만 발산하지 않음 |
| 재개 후 GPS invalid 유지 / Smooth 횡p95 | 8.640초 / 10.57cm |
| 재획득 첫1초 / 이후1–5초 Smooth 횡p95 | 13.14 / 4.16cm |
| 재개 후 TF closure p95 | 0.0218cm |
| 목적지·충돌·안전 종료 | SUCCESS, 충돌0, 실제 P VERIFIED |

nominal 정차에서는 fresh vx=0 측정과 Global soft vy prior가 충분히 제한한다.
**새 ZUPT/Local prior는 구현하지 않았다.** 큰 Local vy covariance를 실제 mean 발산과
동일시하거나 작은 값으로 clamp하지 않았다. 초기 private replay harness의 hot-edit로
Bash file-offset 오류가 난 실행은 보존·제외하고 clean frozen-runner exit0와 구분했다.

### 체크포인트: 유효 GNSS 복귀와 로컬 무GNSS 예외 실험의 차이

번호 주의: 이 절의 과거 CP13/CP15는 구표 번호이며 수정안에서는 CP12/CP14다.
후속 재배포 수정안과 Spawn 프로그램의 일치는 아래 실제 Spawn 시험 절에 반영했다.
원본 실행명·snapshot·측정값은 당시 provenance 그대로 보존한다.

**대회 조건 후속 정정 (2026-10-15): ‘체크포인트는 음영 밖’은 사용자 지도 그림 판단이었다.**
대회측 유효 GNSS 보장은 아니며, 이후 제공된 실제 Spawn 툴14(수정안CP14, 구그림CP15)에서도
로컬 raw NMEA zero를 재현했다. 여기서 CP15는 구번호(수정안14)다.
아래 SceneLoad 기록은 보존하고 마지막 후속 실측 절과 구분한다.
[공식 좌표 안내 백업](../competition/261015_공식_체크포인트_안내_보관.md)에 원래 좌표·yaw를
보존했다. 아래 CP15 개발 장면에서 GPSzero가 관측된 사실은 유지하지만 공식 CP15가 음영에
있다는 이전 해석은 철회한다. 로컬 장면의 zero 원인은 별도 미확인이다. 본 절의 실험 수치와
원본 bag/snapshot은 수정하지 않으며, 무GNSS reset을 대회 복귀의 필수 조건으로 세지 않는다.

기존 좌표 config의 CP13으로 정차/P에서 약114m SceneLoad 순간이동을 에뮬레이션했다.
원래 목적지로 fresh route를 확인하고 stock PP/PI를 재개했다.

| CP13 개발 emulation | Guard OFF | Guard active ON |
|---|---:|---:|
| Global XY≤25cm를0.5초 유지 | 3.369초 | 3.262초 |
| Smooth 동일 gate | 4.009초 | 3.942초 |
| 실제 relocation reset / ACK | 0 / 0회 | 0 / 0회 |
| 재개 후 Smooth 횡p95 | 8.39cm | 8.61cm |
| 목적지/충돌/최종 P | PASS/0/VERIFIED | PASS/0/VERIFIED |

GNSS update가 innovation114m를 먼저 줄여 정차 gate가 준비될 때≤1.362m였다.
**ON이 더 빠르게 reset했다는 증거가 아니다.** Local XY step은약5.8cm 이하였지만
SceneLoad의 absolute heading으로 Local yaw는약46° 변했다. Local 전체 pose의 연속성과
TF closure/독립 GT 정확도를 혼용하지 않는다.

가드는 opt-in 안전 계약만 강화했다: 정상 frame·finite/unit quaternion/covariance·
co-time state/orientation, vx≤0.05m/s1초, GNSS span≤0.3m1초, gap/invalid/duplicate
count 초기화·재획득1초 grace, 단일 pending reset과 새 Global2개 ACK다.
ACK2초 timeout/발행 실패는 blocked latch이며 restart 전 재시도하지 않는다.
미래 stamp로 cache가 고정되거나 잘못 reset되던 결함도 mock으로 재현·수정했다.
age≤0.5초/미래≤0.05초 검사는 **guard만의 발행 안전 조건**이며 core 시간 정책은 그대로다.
`061917`은 마지막 absolute-age 강화 전 snapshot이므로36unit/격리 transport와 구분한다.

**로컬 CP15 ScenarioLoad에서 raw GPS=0이 관측됐다.** 공식 체크포인트의 음영 여부를
증명한 것이 아니며, 공식 운영 이벤트가 아니라 개발 장면의 예외 조건 대조다.

| CP15 emulation | 새 위치 정보 없음 `064054` | 명시적 known prior `064654` |
|---|---:|---:|
| 순간이동 뒤 위치 차이 | 약138.65m 유지 | reset 후약1.94mm |
| 음영 중 GNSS odometry | 0개 | 0개 |
| 추가initialpose | 0회 | lab prior1회(총2회) |
| 재출발 | GPS safety gate 차단 | stock PP/PI 재개·PASS |

known prior는 요청한 SceneLoad checkpoint 좌표를 map TF로 변환하고 fresh canonical
Competition heading을 복사해 기존 `/localization/initialpose`에 한 번 전달했다.
Ego XY는 runtime prior가 아니라 안전·독립 오차 확인용이다. guard 자체 reset/ACK는0회,
fake GNSS도0개였다. Global-only reset에서 Local XY는 이동하지 않았다.
실제 reset receipt 후약18ms에25cm 안으로 회복, 약518ms에0.5초 유지 gate를 충족했다.
GPS는 stock 제어 재개1.682초 뒤에야 돌아왔다. 당시 prior header/receipt 차이가302ms라
후속 helper는 TF/subscriber 준비 뒤 fresh P·heading을 다시 확인하고 발행 직전에
새 event stamp를 설정했다. 최신 `065447` 반복도 같은 stock 경로로PASS했다.

**알려진 반환 위치가 있으면 음영에서도 재초기화 가능**하지만 GNSS·새 위치 정보가
모두 없으면 삼각함수/ZUPT/Kalman 튜닝으로138m 이동을 알아낼 수 없다.
유효 GNSS로 재정합하는 복귀를 우선 검증하되, 실대회 유효 GNSS는 아직 보장되지 않았다.
명시적 좌표 prior를 사용할 경우에는 운영측이 제공/허용하는 복귀
이벤트·번호·배치 정확도 계약이 추가로 필요하다. 임의 위치 추측/Ego GT 재활성화를
해결책으로 사용하지 않는다. z는 마지막 GNSS를
유지하며 known XY prior가 높이 측정을 대체하지 않는다. 옛 metadata의 GNSS-only
문자열보다 실제 known receipt/bag가 우선이고 새 helper는 metadata도 두 모드로 구분한다.

### 종방향 원인과20ms 후보의 이득/악화

raw GPS 직접 UTM 변환과 recorded GNSS odometry는 일치했다. signed 종오차는 속도에
비례했고 robust 회귀/독립 Ego lag sweep에서 최신 full은약19–20ms,E24는24ms,
pair는16ms,9/19는거의0ms의 apparent lag였다. 정지 때는약5cm cell bias뿐이고 vx 차이도
작아−26cm 고정 lever-arm/vx scale/Smooth 단독 지연으로 설명하지 않는다.
이는 상대 observation epoch 진단이지 실제 네트워크 지연20ms의 확정이 아니다.

후보는 GPS map XY에 `dt * vx * [cos(corrected_map_yaw), sin(corrected_map_yaw)]`를 더한다.
header/UDP 시간/z/cell/Q/R/heading은 유지한다. canonical twist·orientation의 같은 stamp,
base_link header, 정상 quaternion·≤0.1초 source age에서만 적용한다. missing/stale/future/
malformed pair는 원래 유효 GPS를 발행한다. map yaw를 또 회전하지 않고 Ego 위치나
미제공 Competition vy를 사용하지 않는다. default0은 aux 구독도 없다.
motion 상관/모델 불확실성의 별도 covariance 보상은 하지 않은 단일변수 후보다.

| 동일입력 Smooth moving 종오차(cm), mean/p50/p95 | 0ms | 20ms |
|---|---:|---:|
| 최신 full `044421` | 27.21 / 25.97 / 57.92 | 2.99 / 2.24 / 7.64 |
| historical9/19 heldout | 2.46 / 1.83 / 6.59 | 21.86 / 23.22 / 36.54 |

최신 정상선회 종p95는31.18→7.72cm,고속20m/s+는72.80→7.20cm였다.
반면 Global 정상선회 횡p95는4.11→4.74cm로 소폭 악화했다. moving fallback은 최신
0.37%,과거6.71%다. fallback이 항상 성능상 무해하다고 보장하지 않는다.
raw byte/header/count·static TF·초기화/SHA·약50Hz를 감사했다. 최신은 원본 post-load
pose/covariance까지 일치했다. 과거 원본에initialpose가 없어 두조건 동일first-raw 초기화를
썼으며 당시live 완전 재현으로 주장하지 않는다. Ego 없는 GPS 변위/Competition 적분은
trapezoid에서최신30/과거8–10ms,left-hold12/0ms,right-hold40/20ms로 바뀌었다.
상대 변화의 단서는 있지만 절대 보정값을 자동 선택하는 근거로 쓰지 않는다.

실제 짧은020 주행`062131`은PASS했다. 같은scene Smooth 음영 종p95는 기본25.10cm
대 후보6.52cm,횡p95는8.56 대4.64cm였다. 다른 실제 주행이며 인과 replay 표와 구분한다.
기본 전체 코스`062655`는PASS했으나020 전체`063115`는고주로 MPT infeasible→비상제동→
out-of-route→먼 목적지2점zero-speed fallback으로 중단했다. 첫warning에서 독립 GT 대비
Global/Smooth XY는7.17/6.09cm,횡−2.15/−1.35cm,GPS invalid0개였다. GPS 발산/프로세스
crash는 아니며 기존 baseline의 같은 장소/route 실패 이력도 있다. 다만 후보와의
상호작용을 기각하지 않았으므로 **전체 코스 승격 gate는 실패**다. 정책을 바꿔 통과시키지 않았다.
ERROR/MANUAL_ABORT·안전 P 확인을 보존하고 partial을 완주와 비교하지 않는다.

### 재현·검증·BEVFusion 인계

새10개 실행은 [bag catalog](260924_LOCALIZATION_EVALUATION_BAG_CATALOG.md)의 정확한
execution명을 따른다. 상세JSON/그림/생성script는 Git제외
`planner_logs/analysis/261015_localization_final_hardening/`의
`stationary/`, `checkpoint/`, `longitudinal/`, `collapse/`에 있다.
localization_manager/bringup/validator만 증분 설치했다. 순수unit202개(측위124,validator70,
helper8), 가드36unit 포함 및 격리ROS transport를 통과했다. ROS transport 파일을 일반
unittest discovery로 실행한 초기 실패는 runner 오류였고 격리rostest와 구분했다.
source/old install 혼재 시험은 gate에서 중단한 뒤 새 실행을 사용했다.

helper는 **개발용 정차·SceneLoad 개입**이며 production launcher가 아니다.
stock stack/PP·PI/UDP를 사용하고 `--execute` 없으면 check만 한다. 다른 주행기와 중복
실행하지 않는다. 본래 `planning_validator.py`에는 여전히 `--run`을 붙이지 않는다.

```bash
source scripts/workspace/source_aw149.sh
# 공유 baseline YAML 전체를 개인workspace에 복사해 한scene·safety/bagON으로 실행.
# intervention시간도포함하도록timeout설정(이번180초). 조건변경강제resume금지.
/usr/bin/python3 scripts/evaluation/localization/run_validator_intervention.py \
  <개인 캠페인 YAML> --mode shadow_hold --hold-sec 30 --execute

/usr/bin/python3 scripts/evaluation/localization/run_validator_intervention.py \
  <개인 캠페인 YAML> --mode checkpoint --checkpoint 13 --execute

# lab에서확정한반환정보대조. 공식이벤트없는자동대회명령이아님.
/usr/bin/python3 scripts/evaluation/localization/run_validator_intervention.py \
  <개인 캠페인 YAML> --mode checkpoint --checkpoint 15 --known-checkpoint-prior --execute

# 환경을검증하는별도후보. 공통실대회기본변경아님.
localization gnss_motion_compensation_sec:=0.02
```

BEVFusion 재개 시 기존 alias/토픽/Heading/UDP/soft/Smooth 계약은 그대로다.
새 기능을 함께 켜야 하는 의존성도 없고 가중치/perception/controller 정책을 변경하지 않았다.
**정상 GNSS 체크포인트 복귀의 위치 회복 대기·새 route·제어 재개**와 LAN gate를
판단팀/운영측과 확인해야 한다. 명시적 무GNSS 복귀 prior는 별도 개발 예외 실험이다.
20ms의환경검증·route interaction은 후속연구이며 기본0을 유지한다.
복귀 실험은 무객체·짧은 원래목적지이며 공식 remaining-checkpoint 미션 복구가 아니다.
실제 배우/BEVFusion이 있는 복귀에는 tracker/prediction history와 오래된 trajectory/command
무효화·공식 route 진행 상태의 별도 통합 확인이 필요하다.

### 9월 19일 무카메라·무LiDAR 조건 복원과 실제 풀주행

사용자가 9/19 경량 센서의 실제 배치 OFF 조건을 재확인했다. 과거 비교 입력은 현장
LAN bag이 아니라 `260919_231906_q0005_r010_gt_object_lightweight_no_traffic.bag`이다.
`configs/sensors/Sensor_localization_gt_lightweight.json`은 생성 commit `37b209d4`부터
bytes가 같고 SHA256은 `4a7c8eb9f9f5a5c5a99a313472ffc8d6b71df20f1ed26a7f05c4fef9b5576c74`다.
cameraList/Lidar3DList/Lidar2DList가 모두 비어 있으며 GPSList/IMUList는 현재
`Sensor_competition_local.json`과 구조적으로 완전히 같다. 센서 pose·rate·noise 변경 없이
카메라·LiDAR 생성 부하를 분리할 수 있다.

새 개인 입력은 `full_course/workspace/loc_fullcourse_true_lightweight.yaml`이다.
기존 soft_production의 무객체 코스·목적지·최신 soft/Smooth·실제 planning/PP·PI·od_gt·
신호등OFF·RViz/영상10fps split·측위bag/safety를 유지하고 sensor 경로만 바꿨다.
종방향 보상은 기존 기본값인0초를 명시했다. 설정 검사와 dry-run은1장면×1회,
기존600초 주행 상한으로 통과했고 source/installed runner·UDP launch·GNSS converter·
Smooth publisher도 일치했다. 과거9/19의 필터/제어기 구현을 되돌리는 시험이 아니다.

Validator YAML은 ROS 수신/TF 구성을 바꾸지만 MORAI Sensor 배치를 자동 Load하지 않는다.
사용자가 MR에서 경량 Sensor Load 완료를 확인한 뒤 새 실행
`loc_fullcourse_true_lightweight_261015_112304`로 실제 풀주행했다. 기존 master만 남은
graph에서 시작했고 live XMLRPC에서도 카메라4개/LiDAR/Velodyne 발행자가 없었다.
`/ctrl_cmd`는 stock longi_pi 하나, planning state는 GNSS EKF publisher,
controller state는 Smooth publisher 하나였다. Ego GT pose provider로 대체하지 않았다.
센서 snapshot과 실제 명령의 `enable_lidar=false`, camera/lidar TF=false를 함께 대조했다.
`include_sensors=false`만으로 센서 생성OFF를 선언한 것이 아니다.

| 실제 폐루프 확인 | 결과 |
|---|---:|
| validator 주행 / controller header 평가창 | 153.71 / 154.557초 |
| 주행·충돌·종료 | PASS/SUCCESS, 충돌0, 실제 정지/P VERIFIED |
| 실제 GPS invalid / 정상 재획득 | 7.276초 / 복귀 확인 |
| GPS / IMU / Competition·Ego | 29.99 / 49.95 / 49.98Hz |
| Global·Smooth / 최대 header gap | 50.00Hz / 24.45ms |
| trajectory / ctrl_cmd | 10.01 / 50.00Hz |
| 독립 입력·중간 state·후단·제어 녹화 | localization replay/assessment audit 모두 true |

모두 **보정하지 않은 원래 header/pose**로 평가했다. 아래는 정상 GNSS 이동
(`|speed|>0.5m/s`, 음영과 재획득 첫1초 제외)의 절대 종오차다. Global/Smooth는 각
실제 주행에서 기록한 출력이며 과거 same-input20ms 후보 표와 모집단을 혼용하지 않는다.

| 상태 | 기존 로컬 `044421` mean/p50/p95(cm) | 새 무센서배치 `112304` mean/p50/p95(cm) |
|---|---:|---:|
| raw GPS | 28.70 / 26.51 / 60.26 | 4.49 / 4.23 / 9.19 |
| Global | 27.70 / 26.51 / 60.14 | 1.46 / 1.09 / 3.71 |
| Smooth | 27.33 / 26.08 / 59.60 | 1.46 / 1.10 / 3.67 |
| raw GPS 상대epoch 최적lag | 20ms | 0ms (2ms 검색격자) |

기존 로컬 반복 `045214`도 raw GPS lag18ms였으며, 9/19 진짜 경량 raw는0ms,
종mean/p50/p95=4.91/4.53/10.54cm였다. 후자는 구형 software와 전체bag 기반 정상 이동
창이므로 현재 controller 창과 완전히 같은 주행으로 취급하지 않는다.
새 입력 raw→GNSS converter 동시각 좌표 차이는5,244개에서0이었다.
종오차의 속도 회귀계수는 새 raw−0.0000393초 대 기존 raw−0.019595초였다.

새 전체 이동 구간(음영·재획득 포함)의 Smooth 결과는 다음과 같다.

| 지표 | mean / p50 / p95(cm) |
|---|---:|
| XY | 2.18 / 1.75 / 4.87 |
| 종 | 1.47 / 1.10 / 3.71 |
| 횡 | 1.24 / 0.78 / 4.01 |

새 실제 최고속도34.714m/s,25m/s 초과 Smooth 종mean/p95=2.39/7.13cm였다.
단순히 저속 주행만 했기 때문에 오차가 작아졌다고 설명할 수 없다. 실제 음영 중
Smooth 횡p95=4.42cm, 재획득 첫1초5.16cm였고 state 중단 없이 복귀했다.
정상/음영 phase 전체 표본 표와 실제 Ego 경로 추종 오차는 별도 분석에 보존했다.

**판단:** 센서 배치·생성 부하와 GPS↔Ego 상대epoch의 관계를 강하게 지지한다.
EKF나 Smooth를 바꾸지 않아도 최신 로컬의 큰 종편차가 사라졌다. 다만 서로 다른
시점 주행이므로 camera/LiDAR 중 어느 구성·부하·스케줄링이 원인인지는 확정하지 않았다.
동일 코드/코스/RViz·영상 조건에서 실제 Sensor ON 재주행이 다음 인과 대조다.
raw 상대lag를 물리적 UDP 전송지연·RTF로 단정하지 않는다. 주행153.71초와 전체
case lifecycle203.25초의 비율도 RTF가 아니다. 대회4090/LAN 기본값을 이 결과로 변경하지 않는다.
**20ms 보상은 계속0초, Q/R·soft/Smooth·heading·UDP 정책과 planning/control 정책도 그대로다.**

원본은 `planner_logs/validation/loc_fullcourse_true_lightweight_261015_112304/success/`의
case bag/snapshot/events/receipt다. 분석과 새 wrapper는 Git 제외
`planner_logs/analysis/261015_true_lightweight_comparison/run112304/`의
`comparison.json`, `README.md`, `phase_analysis.json`, `PHASE_REPORT.md`, `timeline.png`에 있다.
완료 후 validator 소유 노드는 모두 종료되어 원래 master와 rosout만 남겼다.

### 9월 15일 LAN 현장 bag 상대 epoch 역산 — 직접GPS 미녹화의 한계 보완

사용자 요청으로 현장 localization1/2/3과 직접GT 대조의 원본4개 bag를 전수 추출했다.
모든 bag은33토픽이며 다른 이름/type의GPS·IMU·Competition·GNSSodom·내부EKF도 없었다.
독립 bag TF의 map→morai_map은 translation(131.55913022,545.911338186),yaw0이었다.
fusion oracle heading−1.347°,직접GT0°를 분리했다. 이분석은 offline이며 주행·source/
config·시간 정책·원본bag를 변경하지 않았다.

| 현장bag, 실제 receipt 평균Hz | Ego | 전방camera | LiDAR | 최종state | tracking | trajectory | ctrl_cmd |
|---|---:|---:|---:|---:|---:|---:|---:|
| localization1 | 48.415 | 29.981 | 12.811 | 50.000 | 29.842 | 10.006 | 50.000 |
| localization2 | 47.992 | 29.976 | 12.592 | 50.005 | 29.807 | 9.996 | 50.000 |
| localization3 | 47.909 | 29.939 | 12.592 | 50.000 | 29.798 | 9.995 | 50.000 |

平均Hz는count/span이다. 잘 들어오는Hz만으로GPSepoch가0이라고 판단하지 않는다.
최종state를 독립Ego와 비교해 종오차와 속도/방향/가감속/회전의 관계를 역산했다.
positive lag는state(t)가Ego(t−lag)에 더 가깝다는 뜻이다. speed>2m/s,유효oracle,
같은 lag비교표본,2ms검색격자와5/10초블록bootstrap각300회를 사용했다.

| 최종state header기준 | MAE최적lag | 공간·가감속 nuisance 포함 robust회귀lag | 10초block 최적lag 2.5–97.5% |
|---|---:|---:|---:|
| 현장1 | 4ms | 5.66ms | 4–6ms |
| 현장2 | 4ms | 5.79ms | 4–6ms |
| 현장3 | 4ms | 5.40ms | 4–6ms |
| 현장 직접GT | 0ms | 0ms | 0ms |

이구간은 표본재추출 정밀도이며 미관측입력·모델체계오차의 신뢰구간이 아니다.
방향별best는2–6ms였다. 정상covariance trace<0.02m² proxy와 직선 대조도 대체로4–6ms를
지지했다. 제한된 정속subset 회귀는 조건수가 나빠 넓은구간을 보여 단일숫자를 강조하지 않는다.
현장3의15–20m/s 구간은 평균16.71m/s에서 median종오차 약−6.9cm였고,
20ms 지속지연이면 약−33.4cm가 예상돼 기록과 맞지 않았다.
0ms/최적4ms/20ms 가정의 현장3 종오차MAE는5.52/3.72/16.53cm였다.
이는 oracle시각이동 진단이며 runtimeGPS보상실험이 아니다.

방법은rawGPS가 있는5개 bag로 교차검증했다. raw→최종Global lag가
일반센서20→20,18→18,새경량0→0,9/19경량0→0,현장구형sigma.05/Qxy.05계열0→0ms로
일치했다. 필터출력이 숨은GPSepoch의 유용한proxy라는 근거이지만 raw없는 현장의
GPS 지연을 직접 측정한 것은 아니다. 당시predictcurrenttrue 소스와 일치하며 그옵션이
receipt stamp에 숨은미지의 관측지연까지 자동 제거하지는 않는다.

추가로 최종XYcovariance 정상영역 수축 빈도는현장1/2/3에서28.79/29.11/29.22회/s였다.
약30Hz 절대위치갱신 패턴과 정합적이며 정확한GPSHz는 아니다. 현장3에서120ms보다긴
보정빈구간은7.58초 한 번으로 covariance증가 구간과 겹쳤다. 음영외장기보정단절은
이proxy에서 관측되지 않았다. 전체15-state cross-cov/update순서가 없어GPS값을정확복원하지 않는다.

최종state header→bag age p50/p95는현장1 .886/1.132ms,현장2 4.038/28.214ms,
현장3 .965/1.206ms였다. 2번은receipt기준lag약10ms로 늘어GPS와별개인하류지연이있다.
직접GT조차header기준0ms지만receipt기준최적6ms가된다. 작은headerage는stamp전부하/
물리센서지연을 제한하지 않는다. LiDAR age약90ms는scan누적을포함해GPS의대용지표가아니다.

**판단:** 현장은로컬일반센서18–20ms보다작은수ms상대차이와정합적이다.
"정확히0ms"라기보다"큰고정20ms급지연은지지되지않음,추가보상0초유지"가맞다.
MORAI PC분리가유리했다는가설을지지하나CPU사용률·stamp전queue가없어원인을확정하지않는다.
당시source기본과실제runtime를구분하고현재production의LAN검증완료로세지않는다.

세부보고서·숫자cache·재현script·원시입력5개calibration·독립검토는Git제외
`planner_logs/analysis/261015_field_gps_epoch_inverse/`에있다.
기존필드bag등급은FIELD_RESULT_ONLY를유지한다. 새로운추정으로원시입력이나현재필터
재실행권한을복원한것은아니다.

### 제공 Spawn 프로그램 실제 시험 — 번호·원시 패킷·측위 복귀

2026-10-15 사용자가 제공한 `MORAI_Spawn_Linux`를 Music에서 Git 밖
`~/Applications/MORAI_Spawn_Linux`로 이동했다. 프로그램 SHA256은
`5ecc827f5fcbd1cbe00aabb5b01aa01e6d1f87293345d61c20ba5d139210a2f6`이다.
Python3.8 PyInstaller 단일 실행 파일이며 `숫자+Enter`/`q+Enter`를 받는다.
`127.0.0.1:7789` gRPC의 Ego SetTransform을 호출하고 주소 CLI 옵션은 없다.
시작만 해서는 차량을 옮기지 않으며, CLI의 Success 문자열은 측위 복구 증거가 아니다.

**수정안 확인 후 번호 계약:** 0=START, 1–14=현행 CP1–14이며 좌표·yaw가 모두 일치한다.
사용자가 후속 제공한 그림은 `image-20261007-064744.png`다. 최초 비교에 사용한 구표·구그림은
START+1–15였으므로 차이를 프로그램 불일치로 본 판단은 철회하고, 번호 관련 문의는 삭제한다.
현행11은 수정된 좌표이며 현행12/13/14=구CP13/14/15다.
[수정안 CURRENT와 구표 HISTORICAL](../competition/261015_공식_체크포인트_안내_보관.md)을 따른다.
원본 bag·JSON·실행명의 CP13/15 및 UNMATCHED 라벨은 당시 구표 기준이며 고치지 않는다.
아래 본문은 현행 번호를 우선하고 구번호는 명시한다. 프로그램/좌표를 바꿔 재시험한 것이 아니라
같은 실측 좌표의 공식 번호가 확정된 것이다. runtime rule monitor의 구15점 설정은 별도 동기화 대기다.
‘체크포인트는 음영 밖’은 그림을 보고 한 시각적 판단이었다고 사용자가 후속 확인했다.
복귀 직후 유효 GNSS가 제공된다는 대회측 보장으로 사용하지 않는다.

#### 시험 조건과 안전

현재 production soft/Smooth·GNSS시간보상0·heading−1.347°·Q/R·receipt-time을 고정했다.
실제 MORAI Sensor 배치는 카메라/LiDAR 없는 경량 GPS/IMU이며 무객체 scene,
od_gt→tracking/Map-CV→ROS1 planning→기존 PP/PI·UDP를 사용했다.
RViz/영상10fps split·localization_analysis bag·safety ON,신호등OFF다.
초기 장면 준비에만 stock ScenarioLoad를 사용하고 **각 checkpoint 이동은 전달받은
원본 실행 파일**로 수행했다. 추가 initialpose 주입은0회, relocation guard는OFF다.

스폰 전 제어기를 중단하고 brake=1/accel=0·P 송신을 유지했다. 원시 Ego로 새 위치와
header를 확인하고, 스폰 후 3D실속도≤0.05m/s/P/ExternalCtrl의1초 안정화를 요구했다.
P·fresh 입력·GPS·Global/Smooth XY≤25cm의0.5초 유지가 확인되기 전 다음 단계로 가지 않았다.
초기 시도의 즉시 P 검사 실패와 후속 정상 시험은 새 execution으로 분리했다.

**실제 관측:** SetTransform 직후 raw gear가D로14.56–49.45ms 바뀌었다가 P로 돌아온다.
높이의 접지 settling도 있어 최초 작은0.42m 스폰의3D속도최대0.0927m/s,
XY속도최대0.0292m/s였다. CLI가 brake·gear·velocity reset을 명시적으로 보내지 않는 것과
MORAI 내부에서 D/물리 상태가 잠깐 변하는 것을 구분한다. 통신·실제 정지를 확인 못하면
다음 자동 이동/주행은 차단하며, 이 transient를 무시해도 된다는 production 변경은 하지 않았다.

#### 0–14 정차 시험

실행 `loc_official_spawn_probe_261015_131944`는 각 지점에서 정차·회복을 관측했다.
원본 bag의 static TF를 읽고 실제 Ego teleport15개에서 보간을 분할했다.
아래 회복 시간은 실제 Ego 위치 변화부터 측정하며 정상 주행 오차와 구분한다.

| 항목 | 실측 결과 |
|---|---|
| GNSS 유효·정차 자연 복구 | 툴0–13의14지점 |
| Smooth XY≤25cm 최초 진입 / 0.5초 유지 확인 | 2.28–6.53초 / 2.78–7.03초 |
| 정상14지점의 마지막2초 Smooth XY mean | 2.48–12.05cm |
| Global/Smooth 발행 | 약50Hz, 주요9개 stamped topic header역행0 |
| Local XY 최대 frame step | 0.600mm, map 절대위치 오차와 구분 |
| 스폰 후 추가 initialpose | 0개; GNSS 자연 보정과 Smooth 정렬 반영 |
| 툴14/수정안CP14(구CP15) | GPS340개 모두zero, 새GNSS odometry0개, 위치 미복구 |
| 툴14의 Global/Smooth XY | 이전 툴13/현행CP13(구CP14)에 남아 약136.77m 오차, heading은 새 방향 반영 |
| 미복구 후 종료 | 자동 재출발 금지, 실제 정지/P VERIFIED |

GPS가 없어도 현재 state는50Hz로 계속 나오므로 state freshness만으로 절대위치 복구를
인증하면 안 된다. soft vy/ZUPT/더 큰 Q가 관측되지 않은136m 순간이동을 알아낼 수는 없다.
기존 SceneLoad 구CP15(현행14)의zero가 툴 차이만으로 사라지는 현상은 아니었다.

#### 원시 패킷 대조 — 0→14→0

별도 **정차 원시 패킷 진단** `packet_probe_20261015_132719_550832`는 scene load·
localization·planning·주행 controller 없이, stock brake/P sender와 소유 receiver만 사용했다.
GPSzero인 툴14에서 시작해0→14→0을 옮겨 마지막은 정상GPS·정지/P로 종료했다.
tcpdump 권한이 없어 소유 receiver의 recvfrom만strace로 관찰했다. 관찰 부하가 있으므로
기존 수ms latency 평가와 혼용하지 않는다. 첫recvmsg 계측 시작 오류는 별도 원본 보존했다.

- GPS GGA1125개 중470개가 원본 bytes부터lat/lon0이며 fix quality는모두1이었다.
  RMC도동일470개가zero인데 validity는A였다. **status만 보지 않고 좌표zero를 거부하는
  기존 adapter 계약을 유지**한다. bridge/ROS가 정상GPS를0으로 만든 것이 아니다.
- 초기14zero→0valid→14zero→0valid를 반복했다. 실제 지역/scene 조건에 묶인 현상이다.
- Ego/Competition/IMU native sec/nsec와 GPS UTC는 역행/리셋하지 않았다.
  기존 ROS receipt-time 정책·source/설치본/프로그램hash도 변하지 않았다.
- 평균Hz는 Ego49.97,Competition50.00,IMU49.95,GPS29.96이었다.
  최대수신gap은 각각59.66/46.06/51.68/80.20ms로100ms이상 단절은없었다.
- Competition 원본position은항상0이고heading은새방향으로바뀐다.
  rawgear의짧은D도원본에있다. 프로그램이별도ROS reset이벤트를발행하지는않는다.

현재 로컬 scene의 shadedArea는 중심(-23.668,-519.240),긴폭117.959/짧은폭7.419m다.
기존 dataset 검증기의yaw−90° 축 가정으로 현행CP14(구15) 중심은측면경계밖약1.09m다.
차량collider·실제volume좌표축·경계판정에민감한위치라는단서이지, 원인이확정된것은아니다.
그림의파란영역만보고 유효GNSS를보장하거나 이를실대회volume의직접측정으로쓰지않는다.

#### 판단과 산출물

정상GNSS복귀도 즉시정확해지는것은아니며, 수초동안큰중간오차가남는다.
따라서 **정지유지→위치/yaw복구확인→새route/trajectory→제어재개** 계약이필요하다.
zero-GPS 복귀에는 신뢰할 수 있는 복귀번호/좌표 등의새절대위치입력이필요하지만,
이번에 자동초기화·가드기본값·판단/제어정책을변경하지않았다.
배우없는정차실험으로tracker history무효화나충돌후공식미션상태복구를검증한것도아니다.

Git 제외 `planner_logs/analysis/261015_official_spawn_probe/`에
`batch_analysis.json`, `compact_summary.json/md`, 첫중단분석, private실행/평가기와
raw probe의`raw_summary_v2.json`·원시trace·bag·metadata를모았다.
`raw_summary.json`의invalid좌표거리계산오류는v2에서결측전이로수정했고원본은보존했다.
재출발 actual drive의 별도 결과는 후속 절과 원본execution을 따른다.

#### 실제 유효GNSS checkpoint 재출발 — stock planning/PP·PI PASS

별도 실행 `loc_official_spawn_remaining_route_261015_132923`는 툴12(수정안CP12, 구CP13)로
약211m 이동하고, 원래START로돌아가지않은채 그지점에서 재출발했다.
브레이크/P에서 Global/Smooth XY≤25cm 및 보정된Competition대비yaw≤1°를0.5초
확인한뒤, 기존 전체코스의후반2waypoint와목적지로 fresh route를요청했다.
추가initialpose/GT위치fusion없이 기존PP/PI→MORAI UDP로실제이동했다.
분석시구간은 `OFFICIAL_SPAWN_STOCK_CONTROLLER_RESUMED`부터`RUN_FINISHED`다.
전체monitor변위에teleport가포함되므로이를주행거리로사용하지않았다.

| 실제 재출발 구간 | 결과 |
|---|---:|
| 시간 / rawEgo 이동거리 / vx적분 | 34.115초 / 416.044m / 416.007m |
| 실제 최고속도 | 16.366m/s |
| 실제Ego 최종목적지XY거리 | 0.159m |
| 실제주행중 GPSzero | 7.267초,정상재획득 |
| 이동>1m/s Global XY mean/p50/p95 | 2.91/1.99/6.42cm |
| 같은창 Smooth XY mean/p50/p95 | 3.01/2.08/6.84cm |
| 실제주행 음영중 Smooth XY mean/p95 | 5.97/7.39cm |
| 충돌 / post-spawn initialpose | 0 / 0 |
| recording replay/assessment audit | 둘다true |
| 최종 raw gear/mode/정지 | P(1),ExternalCtrl(2),VERIFIED |

최종Ego yaw는설정goal yaw와약6.87°다르지만Global/Smooth와Ego의yaw는일치한다.
이를측위yaw오차로세지않는다. TF closure XY p95약0.233mm도내부정합지표이지GT정확도가아니다.

이결과는 **유효GNSS에서스폰→명시적정합대기→새경로→실제재출발→연속주행음영통과**의
성공이다. **GPSinvalid 상태로 현행CP14(구CP15)에 직접 스폰**한 문제를 해결한 것은 아니다.
무객체·현재경량로컬·개발용remaining경로이며공식미션진행상태/배우history초기화는남는다.
`planner_logs/analysis/261015_official_spawn_probe/resume_analysis.json`과
`resume_summary.md`, 원본execution의frozen helper/계약/receipt를따른다.
종료후실제graph는`/rosout`만남았으며,사용자roscore/MORAI/idleGUI는보존했다.

#### 운영측 후속 답변 — CP14 유지, GPS 음영 범위 조정 예정

2026-10-15 사용자가 전달한 MORAI 박영후 담당자 답변(14:33 후속 설명)에 따르면,
**CP14 좌표와 차량 소환 위치를 그대로 유지하고 GPS 음영구역 범위를 조정**해 여유거리를 둔다.
spawn을 전진시키는 방안이 아니며, 앞선5m 이동 제안은 운영측 방안으로 대체됐다.
규정집 v1.2 링크는 이미 갱신됐으나 정보창고 채널 링크는 미갱신이었다는 설명도 받았다.
현재 공식 링크·수정 좌표표·구번호 대응과 답변은
[체크포인트 안내 보관](../competition/261015_공식_체크포인트_안내_보관.md)에 모았다.

상태는 **운영측 반영 예정 / 새 환경 재검증 대기**다. 위 raw GPSzero·136.77m 미복구와
유효GNSS 재출발 PASS는 모두 변경 전 로컬 환경의 EVIDENCE이며 원본을 보존한다.
이번 답변으로 production 측위·좌표·guard를 변경하지 않는다. 새 환경 적용 후 별도 실행에서
툴14 스폰→유효 raw GPS→Global/Smooth 위치·yaw 정합→새 경로→기존 제어 재개를 확인해야 한다.
아직 해당 변경본으로 실행하지 않았으므로 CP14 문제를 검증 완료로 표시하지 않는다.
