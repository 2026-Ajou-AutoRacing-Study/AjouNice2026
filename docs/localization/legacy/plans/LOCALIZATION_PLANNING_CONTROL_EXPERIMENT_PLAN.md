# 측위–판단–제어 공동 고도화 실험 계획

상태: **PLAN**. Production 설정이나 판단·제어 동작을 변경했다는 뜻이 아니다.
이 문서에 후보, 실험 조건, 결과, 채택·기각 사유를 누적한다. 실험마다 별도
문서를 만들지 않는다.

## 목적과 기준선

GNSS 양자화로 인한 위치 보정이 경로·제어에 전달되는 방식과, 곡선 구간의
추종 오차를 분리해 개선한다. 먼저 측위 입력과 좌표계 구조를 검증하고,
그다음 판단·제어팀과 경로 추종 정책을 함께 실험한다.

현재 기준선은 MR `localization`의 `/Competition_topic`, heading `-1.347°`,
GNSS sigma `0.10 m`, Global EKF Qxy `0.0005`, Planning/TF용
`/localization/kinematic_state`, Controller용 Smooth
`/localization/control_kinematic_state`(translation/yaw tau `0.5/0.5 s`)다.
MRP와 GT oracle의 heading 계약은 [현재 상태](COMPETITION_LOCALIZATION_CURRENT_STATE.md)를
따른다. 실험 중에도 공식 UDP receipt-time, GPS `(0,0)` 거부, 음영 중 state 발행,
체크포인트 안전 계약은 임의로 바꾸지 않는다.

최근 전체 경로 bag에서 Pure Pursuit 경로 대비 횡오차 p95는 약 `0.40 m`였다.
고속 직선보다 회전·회전 직후 구간에 큰 오차가 집중됐고, 동일 경로를 GT Ego로
평가해도 p95 약 `0.38 m`였다. 따라서 이 수치를 전부 측위 오차로 해석해서는
안 된다. 명령 조향 대비 실제 yaw-rate의 비율 약 `0.74`, 약 `0.1 s`의 응답 지연은
추가 검증이 필요한 *진단 단서*이지, 즉시 조향 명령에 `1/0.74`를 곱할 근거는
아니다. Smooth와 Quantized **제어 출력** 후보의 폐루프 비교에서는 후자가
Smooth보다 낫지 않아 보류했다.

## 실험 순서

### 1. Quantization-aware GNSS **입력** adapter

기존 Quantized 제어 **출력** 후보와 다르다. 위경도 양자화 셀을 UTM의 한 점으로
확정해 EKF에 넣는 현재 관측 계약을 재검토한다. 셀 경계·폭과 현재 상태의 관계를
이용한 관측값/불확실도 후보를 만들되, EKF 예측치를 독립된 새 GNSS 측정처럼
되먹이거나 임의로 covariance만 낮춰 자신감을 부풀리지 않는다.

- 먼저 실제 GPS lattice, 좌표계, timestamp, 공분산 단위를 재감사한다.
- 현재 point observation과 셀-aware 후보를 동일 bag 재생에서 A/B한다.
- XY·횡/종 p50/p95, frame jump, yaw, 음영·재획득, TF/상태 발행률을 본다.
- offline에서 이득이 있더라도 경량 `od_gt` 폐루프에서 조향 안정성과 경로
  추종을 재확인한다. 특정 bag의 격자 위상에만 맞춘 보정은 채택하지 않는다.
- 의미 있는 폐루프 이득이 없거나 정확도·음영 안전성을 해치면 기각하고
  기존 Q/R과 Smooth를 유지한다. 기본값은 실험 전후 계속 OFF다.

#### 1차 결과: 동일 셀 반복 관측 억제 — 기각

`quantization_input_mode=cell_transition`을 실험 플래그로 구현했다. GPS의
`1/600000°` latitude/longitude 격자가 바뀔 때만 기존 GNSS Odometry를 발행하며,
기존 관측값·공분산·EKF Q/R은 그대로다. Production 기본값 `off`는 유지한다.
이는 셀 내부의 확률분포를 직접 모델링하는 완전한 interval filter가 아니라,
반복 관측의 독립성 가정을 검증하는 첫 ablation이다.

2026-09-26에 `260822_005713_competition_full_route_raw_hz_run01.bag`의 99.44초를
Competition/MR heading `-1.347°`, 1×로 두 번 재생했다. 원본 GPS 2,636건 중
동일 셀 연속값은 1,325건(50.3%)이고 `(0,0)` 음영값은 0건이다.

| 지표 | 기존 `off` | `cell_transition` |
|---|---:|---:|
| GNSS Odometry 발행률 | 26.51 Hz | 13.18 Hz |
| Global state / Smooth state | 50.00 / 50.00 Hz | 50.00 / 50.00 Hz |
| Global 횡오차 p50 / p95 | 1.84 / 6.95 cm | 1.88 / 7.02 cm |
| Global 종오차 p50 / p95 | 9.20 / 20.50 cm | 9.20 / 20.55 cm |
| Global 횡 frame jump p95 / max | 1.45 / 2.91 cm | 1.54 / 17.86 cm |
| Smooth 횡오차 p50 / p95 | 1.34 / 7.59 cm | 1.40 / 7.66 cm |
| Smooth 횡 frame jump p95 | 0.356 cm | 0.358 cm |

입력 발행이 최대 약 6.7초 멈춘 구간이 있었다. 평가기가 이를 GPS outage로
표기하지만 **원본 GPS는 계속 들어왔으므로 실제 음영이 아니다.** 실험 모드가
만든 인위적 관측 공백이다. 반복 측정을 모두 버리면 현재 필터의 위치 구속이
약해질 뿐이며, 동일 bag 오차·연속성 모두 개선되지 않았다. 이 모드는 production
승격·폐루프 시험 대상으로 삼지 않는다. 안전한 다음 입력 후보를 연구한다면
셀 interval의 likelihood와 GNSS 수신 fresh 상태를 분리해야 하고,
`robot_localization`의 Gaussian measurement 계약과의 정합부터 설계해야 한다.

재현 runner: `scripts/evaluation/localization/run_gnss_quantization_input_ab.sh`.
결과: `/home/luv/localization_experiments/gnss_input_ab_260926_004230/`(Git 외부).
실험 당시 branch `localization/competition-stability-v3`, 코드 변경 미커밋,
MR/Competition, receipt-time이 기록된 bag header로 1× replay,
evaluator heading `-1.347°`; Planning/Controller는 실행하지 않았다.

### 2. Odom-frame 경로·제어기 구조

연속성이 좋은 local odom pose를 controller에 제공하되, **차량만** odom-frame으로
바꾸지 않는다. Planning의 map-frame trajectory를 같은 시각의 `map→odom`으로
변환해 차량 상태와 경로가 같은 control-local frame에 있어야 한다.
Global pose와 map TF는 Planning/시각화용으로 유지한다. 이 단계는 측위 패키지의
단순 토픽 remap이 아니며 경로 adapter 또는 controller 경계 변경이 필요하므로
판단·제어팀과 인터페이스를 확인한 뒤 별도 실험으로 진행한다.

- 경로·차량 pose의 frame_id, timestamp, 보간/TF 가능 시간, reset 시 동작을
  먼저 명세한다. map 보정이 변환된 경로의 jump로 다시 나타나면 실패다.
- 동일 기록에서 경로 step, 차량–경로 상대 오차, 지연을 재생 비교한다.
- GPS 음영·재획득·체크포인트 순간이동과 전체 경로 폐루프를 통과해야 한다.
- 현재 map-frame Smooth와 동일 Planning/Controller revision으로 비교한다.
  후자가 이미 충분히 안정적이라면 구조 변경의 위험 대비 이득을 판단한다.

#### 2026-09-26 1차 offline 계약 A/B — 단순 변환판 기각

상태: **EVIDENCE / 단순 8번 기각, 구조적 8번 미시험**. Production과 판단·제어
코드는 변경하지 않았다. 재현 스크립트:
`scripts/evaluation/localization/analyze_odom_frame_control_contract_bag.py`.

- branch/commit: `localization/competition-stability-v3` / `2e142ecfe02e5507232fe10ca781c71d8d56da62`
  (실험 시작 시 기존 GNSS-input 작업이 dirty였으나 이 실험은 기록된 state·경로만 읽었다).
- 입력: `rosbags/eval/localization/local_mr_stability/`의
  `260925_194856_smooth_tau0p5_full_route_no_actors.bag`,
  `260925_200740_global_full_route_no_actors.bag`.
- acquisition: MR 폐루프 기록을 **offline 분석**. `gt_source=competition`, estimator
  heading `-1.347°`, evaluator는 GT pose를 사용하지 않으므로 heading offset 미적용.
  UDP receipt-time으로 녹화된 bag stamp와 trajectory header stamp를 구분했다.
  원본 planning/controller commit은 두 bag metadata의 `8889293f...`이며,
  실험에서 controller를 재실행하거나 real-time factor를 가정하지 않았다.
- 공통: 같은 bag의 map-frame `/planning/trajectory` 약 10 Hz, Global·Local
  odometry 약 50 Hz, Local을 Global stamp에 외삽. 측정값은 controller 차량 pose와
  기록 trajectory의 가장 가까운 선분 간 **절대 횡거리**다. Pure Pursuit 출력이나
  실제 폐루프 결과가 아니다.
- A `odom_latest`: 경로와 Local 차량을 동일한 최신 `map→odom`으로 함께 변환.
  SE(2) 좌표변환 불변성 때문에 Global-map 기준 상대 기하와 정확히 같다.
- B `odom_held_receipt`: 새 trajectory 수신 시 정합을 고정하고 다음 trajectory까지
  Local 차량 운동만 진행. `odom_held_source`는 trajectory 원본 stamp에서 고정한
  이론적 변형이다. 소스 stamp의 과거 TF 조회/지연 처리 구현은 아직 없다.

| 같은 bag 내 횡거리 p50 / p95 | 19:48 Smooth 주행 | 20:07 Global 주행 |
|---|---:|---:|
| Global map 기준 | 5.94 / 41.18 cm | 5.18 / 42.10 cm |
| Smooth map 기준 | **5.00 / 39.31 cm** | **4.19 / 40.28 cm** (offline 재구성) |
| `odom_latest` | 5.94 / 41.18 cm | 5.18 / 42.10 cm |
| `odom_held_receipt` | 5.72 / 41.21 cm | 5.18 / 41.53 cm |
| `odom_held_source` | 5.57 / 40.83 cm | 5.00 / 41.49 cm |

두 bag의 일치 표본은 각각 7,855 / 7,768개였다. 첫 bag의 offline Smooth 계산은
기록된 Smooth XY와 p50/p95 **0.0000 cm**로 일치해 구현을 교차 검증했다.
`odom_latest`와 Global의 상대 횡거리 차이는 전체 표본에서 수치상 0이었다.
Trajectory의 source→receipt 지연 p50/p95는 약 `85/150 ms`, receipt→현재 state
나이는 약 `49/95 ms`였다. `odom_held_receipt`는 경로가 약 10 Hz로 갱신될 때
정합을 다시 받아 차량 근처 경로 좌표가 한 번에 바뀔 수 있다. 두 bag에서 그
정합 변화의 p95는 약 `3 cm`, 최대는 `44~46 cm`였다. 두 bag에서 최대 step은
각각 약 `7.19/7.23초` GPS 음영의 재획득 후 `0.03/0.09초`에 발생했다. 경로를
발행 시점에 odom으로 고정하더라도 재발행 때 이 correction이 다시 들어온다.

재획득 첫 1초의 절대 횡거리 p95는 Smooth `37.36/37.71 cm`, Global
`56.24/62.70 cm`, `odom_held_receipt` `55.81/60.85 cm`였다(각 bag 50 sample).
이는 동일 기록 경로에 대한 기하 평가이며, 새 구조의 폐루프 제어 성능을
직접 측정한 값은 아니다.

**판정:** 단순 좌표변환판은 Global 입력을 재표현할 뿐이며, 경로마다 변환을
고정하는 판도 두 독립 주행에서 Smooth를 넘지 못했다. 현재 구조를 크게 바꿔
동일한 결과를 재현할 이유가 없으므로 이 두 후보는 **REJECT**한다. Local 경로를
재계획 사이에도 연속적으로 유지하고 Global correction·checkpoint와 정합시키는
본격적 odom-frame planner는 다른 설계이며 여기서 검증하거나 기각하지 않았다.
그 설계의 비용 대비 이득은 planning/control팀 인터페이스와 실제 폐루프가
확보될 때만 재평가한다. 당장은 Smooth production을 유지하고 곡선 추종·조향
응답 실험으로 우선순위를 옮긴다.

### 3. 판단·제어 공동 곡선 추종 실험

1·2번 이후에도 남는 오차를 대상으로 한다. 동일 구간과 속도 조건에서 경로
생성 주기/age, 조향 명령–실제 yaw-rate 응답, Pure Pursuit lookahead 제한,
차량 모델의 effective wheelbase/steering gain을 각각 분리해 A/B한다. 좌·우 회전,
저·중·고속, GPS 음영 전후를 나눠 보고 한 번에 한 변수만 바꾼다. 경로 대비
횡오차뿐 아니라 차선 이탈, 조향 진동, 완주 시간과 안전 정지도 함께 본다.
판단·제어팀의 의도를 확인하기 전에는 production 파라미터를 변경하지 않는다.

#### 2026-09-26 곡선 오차 위치·조향 응답 1차 감사

상태: **EVIDENCE / 원인 후보 확인, 제어기 변경 없음**. 현재 branch
`localization/competition-stability-v3`, HEAD `2e142ecfe02e5507232fe10ca781c71d8d56da62`에서
기록된 두 MR 폐루프 bag을 읽기 전용으로 분석했다. 평가 대상은 위 Smooth/Global
actor-free 전체 경로 bag이며 acquisition은 MR, `gt_source=competition`, estimator
heading `-1.347°`, UDP receipt-time, `od_gt`, 동일 planning/controller revision
`8889293f...`이다. 평가에는 GT 절대 heading을 사용하지 않고 기록된 controller
경로 대비 횡오차, MORAI `wheel_angle`, `/ctrl_cmd`, `/sensor/imu` yaw-rate를 사용했다.
분석 중 주행이나 controller 재실행은 하지 않았으므로 real-time factor와 새로운
폐루프 성능을 주장하지 않는다.

- `|path_curvature| > 0.025 1/m`, 속도 `> 5 m/s`인 표본의 횡오차 p95는
  Smooth `64.4 cm`(861개), Global `65.6 cm`(864개)였다. 각각 `88.7%`, `90.6%`의
  표본에서 오차 부호가 회전 방향의 반대였다. 즉 커브 안쪽을 과하게 자르기보다
  **바깥쪽으로 밀리는** 패턴이다. 이 표본의 lookahead 중앙값은 `5.5 m`로 현행
  최솟값에 붙어 있었다.
- Smooth bag의 대표 6개 커브에서 진입 오차는 `-0.17~+0.06 m`이고, 커브 끝 또는
  직후 peak는 `0.45~0.75 m`였다. 좌회전에서는 음수, 우회전에서는 양수의 바깥쪽
  오차가 누적됐다. 최대치 `0.75 m`는 9.1 m/s의 우회전 종료 약 0.2초 뒤였다.
- 위 여섯 peak에서 ego→최종 trajectory 최근접 선분 거리는 `0.44~0.75 m`였다.
  동일 시점 ego→behavior path 거리는 `0.28~0.63 m`; 두 경로 최근접점의 차이는
  `0.07~0.25 m`였다. bag 당시 launch가 지정한 v85 OSM의 해당 lane ID 중심선과
  교차 검증하면 ego→차선 중심선은 `0.28~0.63 m`로 behavior path와 `2 mm` 이내
  일치했다. 최종 optimizer trajectory는 중심선에서 `0.06~0.24 m` 벗어나 있고
  이 peak들에서는 차량보다 회전 안쪽에 있다. 따라서 최종 경로의 offset이
  controller 횡오차를 일부 키우지만, **차량 자체의 중심선 이탈도 남는다.**
  v85 OSM은 Lanelet2 `loadRobust`로 규제 요소 오류 19건을 건너뛰어 lane geometry를
  읽었다. 해당 centerline과 기록 behavior path 좌표가 일치하는 범위만 사용했다.
- 명령 조향 `/ctrl_cmd.steering`을 degrees로 바꾸어 `/Ego_topic.wheel_angle`과
  비교하면, 주행·회전 표본의 절대 차이 p50/p95는 Smooth `0.004/0.418°`, Global
  `0.007/0.461°`, 상관계수는 각각 `0.99969/0.99963`이었다. **명령이 wheel_angle에
  전달되지 않는 문제는 아니다.**
- `v/3.0 * tan(steering)`을 기대 yaw-rate로, IMU z를 실제 yaw-rate로 둔
  원점 통과 회귀는 약 `0.1 s` lag에서 Smooth `0.720`, Global `0.719`였다. 실제
  wheel_angle로 바꾸어도 `0.719/0.718`로 거의 같다. 좌·우, `5~10 m/s`와
  `>10 m/s`에서도 대략 `0.71~0.73`으로 재현됐다. `0.74`는 길이(cm)가 아니라
  개별 표본 yaw-rate 비율의 중앙값에 가까운 **무차원 진단값**이다.
- 높은 곡률 표본에서 목표 곡률 중앙값 `0.080 1/m`가 경로 곡률 중앙값
  `0.058 1/m`보다 컸다. 약한 회전 응답을 횡오차가 누적된 뒤 controller가 더 큰
  목표 곡률로 보상하는 양상과 일치하지만, 이 관찰만으로 차량 동역학·조향
  calibration·경로 lookahead의 인과 기여도를 확정하지 않는다.

**다음 gate:** MORAI의 `wheel_angle` 단위·기준과 조향 명령 계약을 실측/문서로
교차 확인하고, 동일 커브·속도에서 effective wheelbase/steering gain, 경로 곡률
feed-forward, lookahead를 각각 *별도의* 실험 변수로 A/B한다. `1/0.72`를 조향
명령에 일괄 곱하거나 production 속도를 먼저 올리지 않는다. 각 후보는 바깥쪽
오차 p95/p99, 좌우 대칭, overshoot/채터링, 차선 경계 여유와 완주를 폐루프로
확인해야 한다.

## Competition UDP 추가 입력 판단

**추가 입력을 영구히 배제한 결정은 아니다.** 현재 EKF는 Competition의 heading과
`velocity.x`, IMU의 yaw-rate와 종가속도, GNSS의 x/y를 사용한다. 패킷에
`angular.z`가 있어 `/competition/raw_twist`로 보존하지만, 단위·부호 변환 후
IMU yaw-rate와 거의 같은 신호이고 대체 A/B에 우위가 없었다. 따라서 현재는
IMU를 유지하며 둘을 독립 관측 두 개처럼 중복 fusion하지 않는다.

패킷 구조에 `velocity.y/z`, acceleration 등의 필드가 있더라도, 확인한 주행
bag에서는 0으로 기록됐고 대회 제공 계약상 유효한 연속 측정으로 기대하지
않는다. `steer`·gear·brake 등은 차량 응답 분석에 활용할 수 있으나 그 자체가
독립 위치 관측은 아니다. 공식 제공값/실제 packet이 달라지면 유효성·단위·시간·
상관관계를 먼저 다시 감사하고 별도 A/B로 채택한다.

## 공통 검증·기록 계약

각 실험에 branch/commit, MR·MRP, `gt_source`, estimator와 evaluator의 heading
offset, UDP receipt-time 정책, 시나리오/bag, Planning·Controller revision을
기록한다. 측위 단독 replay → 경량 `od_gt` 폐루프 → GPS 음영/재획득 → 가능한
경우 LAN-to-LAN 순으로 검증한다. localization p50 개선만으로 채택하지 않고
p95·jump·경로 추종·제어 안정성과 안전 실패를 함께 비교한다. Production 승격은
실험 결과와 팀 인터페이스 합의 이후 별도 결정한다.

관련 근거: [Smooth 폐루프](260925_SMOOTH_CONTROL_STATE_CLOSED_LOOP_EVIDENCE.md),
[Quantized 출력 후보](260925_QUANTIZED_GNSS_CONTROL_STATE_EVIDENCE.md),
[GNSS lattice](260925_GNSS_QUANTIZATION_LATTICE_AUDIT.md),
[Competition yaw-rate A/B](260925_COMPETITION_YAW_RATE_AND_YAW_SMOOTHING_EVIDENCE.md).
