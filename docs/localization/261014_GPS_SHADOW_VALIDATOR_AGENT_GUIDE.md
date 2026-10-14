# GPS 음영 횡방향 드리프트 실험 — Validator 에이전트 가이드

상태: **LOCAL MR FUNCTIONAL CHECK PASSED / 측위 후보 평가는 별도 EVIDENCE**. 기준: 2026-10-14, Asia/Seoul.
현재 validator의 CLI·후보 전달·입력 사본·측위 녹화·안전 gate와 대조한 작업 안내다.
아래 구현 지원과 실제 주행 성공·측위 개선을 구별한다. 사용자는 필요한 validator 구현과
연결된 MR 시험을 허용했다. Production localization Q/R·heading·시간축·제어 입력 기본값을
바꾸거나 후보를 자동 승격하는 권한은 아니다.

목표는 음영 진입·유지·복귀의 **측위 횡오차**와 **실제 Ego의 횡방향 움직임**을 분리하고
같은 장면의 localization 후보를 비교하는 것이다. Base5 본수집·학습은 범위 밖이다.

2026-10-15 후속 실제 폐루프와 Global-only soft 횡속도 제약의 검증·악화·한계는
[`261015_GPS_SHADOW_CLOSED_LOOP_IMPROVEMENT.md`](261015_GPS_SHADOW_CLOSED_LOOP_IMPROVEMENT.md)에 기록했다.
이 가이드의 기존 기능 시험 표를 후보 성능 표로 재사용하지 않는다. 이후 사용자 승인으로
모델 prior는 기본 ON이며 개선 Smooth도 반영됐다. 현재 값과 미완료 gate는 CURRENT와
위 EVIDENCE의 마지막 절을 따른다. 과거 기준선 재현에는 두 OFF 인자를 명시한다.

## 1. 읽을 문서와 작업 경계

먼저 [README](README.md), [측위 CURRENT](COMPETITION_LOCALIZATION_CURRENT_STATE.md),
[bag 목록](260924_LOCALIZATION_EVALUATION_BAG_CATALOG.md), [실험 기록 형식](EXPERIMENT_TEMPLATE.md)을 읽는다.
과거에 기각한 Q/R·속도 scale·시간축 후보를 근거 없이 production에 다시 적용하지 않는다.

Validator는 [음영 미션 README](../../base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/base/README.md),
같은 폴더의 example.yaml·manifest.json·대표 JSON,
[제작 가이드](../planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md),
[실행 런북](../planning/SCENARIO_VALIDATOR_RUNBOOK.md)을 참고한다.
독립 GT는 [평가 좌표 계약](../bevfusion/GT_EVALUATION_COORDINATE_CONTRACT.md)을 따른다.

- 개인 JSON/YAML·후보 config는 gps_shadow/workspace/ 아래에 둔다. 공유 base/·production config는 덮어쓰지 않는다.
- JSON 배우 배치와 드리프트 분석은 localization 에이전트 업무다. Validator는 객체 배치나 측위 개선을 자동 설계/판정하지 않는다.
- MORAI 통제권은 한 작업자만 가진다. Collector·다른 validator·수동 제어기를 함께 켜지 않는다.
- 사용자 master와 다른 작업 프로세스를 일괄 종료하지 않는다.
- 문서보다 source·설치 overlay·실제 명령·resolved ROS parameter를 우선한다. 로컬 기능 시험을 대회 LAN 검증으로 확대하지 않는다.

## 2. MR 연결과 제어 소유권

실시간 폐루프 주행이다. MORAI는 대회 맵·Sunny13·주행 가능한 상태를 유지하고 pause/Capture freeze를 쓰지 않는다.
연결 변경이 필요하면 사용자가 GUI에서 적용한다. A/B 중 센서·네트워크·영상 부하를 몰래 바꾸지 않는다.

```text
Sensor: configs/sensors/Sensor_competition_local.json
Ego Network: base_ws/src/planning_addons/planning_scenario_validator/config/internal/generated/ego_network_validator_local.json
ScenarioLoad: UDP 127.0.0.1:9095
```

ego_network_mr_base5_savesensordata_collision_local.json은 이번 주행 입력이 아니다.
Sensor_localization_gt_lightweight.json을 쓰려면 실제 MR Sensor GUI와 YAML을 함께 바꾸고 별도 조건으로 기록한다.

Validator는 빈 ROS graph에서 센서 수신·sensing·측위·후단·planning·제어를 소유한다.
기존 rc master만 두는 것은 가능하지만 udp·localization·detector·제어기 등을 따로 켜 두면
중복 검사에서 거부될 수 있다. GUI 네트워크 연결과 ROS 노드 실행을 혼동하지 않는다.

일반 준비 단계에서는 D 제어 송신기가 OFF다. 이번 opt-in 안전 실험은 준비 전용 P 송신기를
쓰고, ScenarioLoad 뒤 새 gear P·ExternalCtrl·raw Ego 정지를 확인한다. Ego 속도는 로드용
사본에서 0으로 준비한다. 원본 JSON은 그대로 보존한다. Route와 첫 trajectory 준비 뒤
P 송신기를 끄고 D 송신기, 마지막으로 제어기를 시작한다.
출발 GNSS는 유효해야 하므로 음영 안에서 시작하는 JSON은 이 기준선용 입력이 아니다.

## 3. 개인 입력 준비와 CLI

저장소 경로는 /home/luv로 고정하지 않는다. 아래 예시는 ~/AjouNice2026 설치 기준이다.
CLI/YAML의 상대경로는 **저장소 루트 기준**이며 YAML 옆 기준이 아니다.

```text
base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/workspace/
  loc_shadow_baseline.yaml
  loc_shadow_gt_pose.yaml
  loc_shadow_obstacle.yaml
  loc_shadow_candidate_tau.yaml
  candidates/                 # Q/R 개인 config가 필요할 때
  scenarios/                  # 사용할 JSON 복사본
```

GUI의 베이스 → 내 작업 폴더 복사 또는 다음처럼 완성된 예제를 복사한다. 빈 YAML을 만들지 않는다.

```bash
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
P=base_ws/src/planning_addons/planning_scenario_validator
mkdir -p "$P/config/missions/gps_shadow/workspace/scenarios"
cp "$P/config/missions/gps_shadow/base/localization_baseline.yaml" \
   "$P/config/missions/gps_shadow/workspace/loc_shadow_baseline.yaml"
cp "$P/config/missions/gps_shadow/base/scenarios/hid_soa_baseline_00.json" \
   "$P/config/missions/gps_shadow/workspace/scenarios/"
```

완성된 `localization_baseline.yaml`은 무장애물 1장면, od_gt, 측위 bag, 준비/종료 안전 확인을
이미 설정했다. 아래는 workspace JSON을 쓰도록 경로를 바꾸는 예시다. 기존 키를 중복 추가하지 않는다.
`localization_gt_pose.yaml`과 `localization_candidate_tau.yaml`도 같은 폴더에 있다.
일반 example.yaml은 기본 24장 미션이며 static_obstacle_gt.yaml은 센서·시나리오 경로가
다른 별도 예제다. 이름에 GT가 있다고 동일 장면 A/B로 사용하지 않는다.

```yaml
campaign_name: loc_shadow_baseline
scenarios:
  directory: base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/workspace/scenarios
  glob: hid_soa_baseline_00.json
  include: [hid_soa_baseline_00.json]
  repeat: 1
stack:
  pose_source: localization
  detector: od_gt
  traffic_light: false
  planviz: true
  localization_args: {}
recording:
  bag_enabled: true
  analysis_mode: false
  profile_preset: localization_analysis
  topic_profile: base_ws/src/planning_addons/planning_scenario_validator/config/internal/topic_profiles/localization_analysis.json
  include_sensors: false
  video:
    enabled: true
    display: ':0'
    fps: 10
    focus_planviz: false
    window_layout: split
    bottom_bar_px: auto
```

2026-10-15 사용자 관찰용 기본값은 RViz/영상 ON, split(RViz 왼쪽·MORAI 오른쪽,
GUI 하단)이다. `gps_shadow/base/`의 예제 6개와 현재 개인 `workspace/`의 캠페인
12개에 동일 표시 옵션을 적용했다. 현재 runner는 영상 ON + RViz ON일 때 split
배치를 적용하므로 layout만 지정하고 video를 OFF하면 자동 배치는 실행되지 않는다.
과거 영상/RViz OFF 실험 결과와 새 ON 주행은 부하 조건이 다르며, A/B에서는 이 값을
동일하게 유지한다. GUI에 이미 열어 둔 YAML은 **다시 불러오기**하고 새 실행을 시작한다.
과거 결과의 snapshot/resume로는 바뀐 원본 옵션이 반영되지 않는다.

예제 validation: 안에 다음을 추가한다. **안전 gate는 opt-in이며 기존 모든 캠페인에
자동 적용되는 기본값이 아니다.** 이번 실험에서는 켜고 A/B 내내 유지한다.

```yaml
  localization_safety:
    enabled: true
    startup_timeout_sec: 20.0
    stop_timeout_sec: 10.0
    freshness_sec: 1.0
    stationary_hold_sec: 1.0
    speed_max_mps: 0.05
    require_valid_gps: true
    park_after_run: true
    stop_on_collision: true
```

검사는 ROS·MORAI 주행을 시작하지 않는다. 설치 경로·네트워크·후보 경로 등은 검사한다.

```bash
GUIDE_CASE="$P/config/missions/gps_shadow/workspace/loc_shadow_baseline.yaml"
/usr/bin/python3 "$P/scripts/planning_validator.py" "$GUIDE_CASE" --check
/usr/bin/python3 "$P/scripts/planning_validator.py" "$GUIDE_CASE" --dry-run
```

JSON 1개 × 반복 1회 = 총 1회와 목적지·제한시간을 확인한다. 연결·안전·통제권 확인 후에만
--check/--dry-run을 빼고 실행한다. --run 옵션은 없다. Source CLI와 roslaunch가 서로
다른 설치본을 소비하지 않는지 실제 ROS overlay도 확인한다.

### 입력 사본과 resume

- 원본 YAML 실행은 timestamp 결과 폴더와 immutable 사본을 만든다. 원본 파일명을 바꾸지 않는다.
- 사본은 실행 JSON·Sensor/Ego Network·토픽 목록·model/측위 계약·측위 config를 보존한다.
- Fusion은 기본/후보 6개 config를 복사한다. GT pose는 manager config와 heading 인자를 보존한다.
- --execution 결과폴더는 당시 config_snapshot.yaml과 파일 hash를 사용한다. 원본 YAML/config를 고쳐도 기존 재개에 적용되지 않는다.
- snapshot 변경·누락, source/설치본·측위/제어 config·heading·안전 계약 변화는 같은 결과 resume를 거부한다. 후보마다 새 실행을 시작한다.
- 성공 회차는 건너뛰며 실패를 다시 돌리려면 --rerun-failed를 명시한다. 중단 회차는 새 attempt가 된다.
- --resume true의 예전 고정 이름 결과 호환 경로는 새 실험에 쓰지 않는다. 과거 기록을 고쳐 계약을 맞추지 않는다.

```bash
/usr/bin/python3 "$P/scripts/planning_validator.py" \
  --execution planner_logs/validation/loc_shadow_baseline_YYMMDD_HHMMSS
```

## 4. 실행 순서: 한 장면씩 원인 분리

| 순서 | 조건 | 확인할 사항 |
|---|---|---|
| A | 무장애물 hid_soa_baseline_00.json, localization + od_gt | 유효 GNSS 출발 → 실제 음영 → 재획득이 bag에 들어 있는지 |
| B | A의 같은 JSON/network/goal/제어, 보정된 maptf_morai + od_gt | 지도·route·제어와 localization 원인 분리; 대회 측위 성능 아님 |
| C | 같은 구간에 정적 장애물/정지 차량 한두 개, localization + od_gt | 회피 위치와 음영 체류시간의 차이 |
| D | A/C와 같은 JSON·조건, tau 또는 Q/R 한 변수만 변경 | 새 YAML·새 실행, 횡/종/yaw·제어·연속성·안전 회귀 |
| E | 다른 일반 장면·진입 조건·장애물 위치 | 특정 장면 과적합 여부 |
| F | 채택 후보와 실제 E24 | 인지/GPU 부하를 포함한 별도 폐루프 회귀 |

24장 전체 live 성공을 가정하지 않는다. 반복 수는 첫 결과 변동을 보고 정한다.
같은 JSON 반복도 MORAI physics/traffic까지 seed로 동일하다고 주장하지 않는다.
od_gt도 TF·tracker·GT 박스 변환 오차까지 제거하는 oracle은 아니다.

### B: 보정된 GT pose

A YAML을 개인 loc_shadow_gt_pose.yaml로 복사하고 이름·아래 stack만 바꾼다.
localization_args는 삭제한다. 기본 maptf_morai는 heading correction OFF이므로 MR 대조에서
아래 opt-in을 빠뜨리면 약 1.347°의 yaw 차이가 남을 수 있다.

```yaml
stack:
  pose_source: maptf_morai
  detector: od_gt
  traffic_light: false
  gt_pose_args:
    use_heading_correction: true
    heading_yaw_offset_deg: -1.347
```

GT 조건은 Ego 위치를 직접 소비한다. Fusion/EKF/Smooth state 미발행은 정상이며 controller는
/localization/kinematic_state를 사용한다. GT PASS를 actual localization 개선으로 합산하지 않는다.

### D: 작은 후보 하나부터

A/C YAML을 복사하고 campaign_name과 단일 후보 인자만 바꾼다. 예시는 yaw tau A/B이지 승격값이 아니다.

```yaml
stack:
  pose_source: localization
  detector: od_gt
  traffic_light: false
  localization_args:
    control_yaw_tau_sec: 0.2
```

Q/R은 production config를 개인 workspace/candidates/로 복사하고 한 설정만 바꾼 뒤 역할을 명시한다.

```yaml
  localization_args:
    global_ekf_gt_config: base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/workspace/candidates/global_q_candidate.yaml
```

gt-overlay가 base config에 덮어쓰는 값도 확인한다. 경로만 바뀌고 최종 Q/R이 같으면 유효한 A/B가 아니다.
배우 UID/DataID·지면·도로 link·BBox 간격은 실제 파일로 확인한다. MORAI actor 좌표를 map goal에
그대로 넣지 않는다. 공식 음영 영역·출발점·목적지는 먼저 유지하며 신호등 OFF, 동적 끼어들기는 넣지 않는다.

## 5. 후보 전달 whitelist와 고정 계약

stack.localization_args는 임의 ROS parameter/launch 전달 경로가 아니다.
잘못된 키·타입·범위·없는 config 경로는 검사에서 거부한다.

| 역할 | 허용 키/값 |
|---|---|
| config YAML 경로 | localization_manager_config, measurement_noise_config, local_ekf_base_config, global_ekf_base_config, local_ekf_gt_config, global_ekf_gt_config |
| bool | use_quantized_gnss_control_state, enable_relocation_guard, relocation_guard_monitor_only, use_lateral_velocity_constraint, control_state_use_vehicle_pivot |
| 0 이상 유한 숫자 | control_translation_tau_sec, control_lateral_tau_sec, control_yaw_tau_sec, control_max_lateral_correction_rate_mps, quantized_control_yaw_tau_sec, control_state_reset_translation_error_m, control_state_reset_yaw_error_deg(≤180), relocation_guard_cooldown_sec |
| 양수/양의 정수 | lateral_velocity_stddev_mps, relocation_guard_threshold_m; relocation_guard_consecutive_samples(정수 ≥1) |
| 선택값 | gnss_quantization_input_mode: off / cell_transition; initial_position_source: ego / official_start |

기본 config는 configs/localization/의 localization_manager.yaml,
gnss_imu_gt_noise_sensor6_ideal.yaml, robot_localization_gnss_imu_local.yaml,
robot_localization_gnss_imu_global.yaml, robot_localization_gnss_imu_gt_ctra_local.yaml,
robot_localization_gnss_imu_gt_ctra_global.yaml이다. 파싱 검사는 YAML mapping과 경로까지이며,
EKF 행렬/필드의 모든 의미를 검증하지는 않는다. 실제 ROS parameter·노드 시작 로그를 확인한다.

MR Fusion은 gt_source=competition, heading correction ON/−1.347°, live use_sim_time=false,
publish_sensor_tf=false, publish_control_state=true로 고정한다. Sensor config는 실행 YAML의
실제 Sensor JSON을 전달한다. 이 값은 candidate dictionary로 우회할 수 없다.
현재 기본값은 Global-only soft body vy prior ON(σ=0.20m/s), vehicle-pivot Smooth
종/횡/yaw tau=0.5/0.1/0.5초, 횡 correction cap=0.3m/s다. quantized control OFF,
relocation guard OFF는 유지한다. custom noise/Global YAML보다 prior launch 인자가
우선하며 publisher/mask가 함께 켜지거나 꺼진다. 과거 기준선에는 아래 두 인자를
명시하고 반드시 **새 실행**을 만든다(옛 결과 강제 resume 금지).

```yaml
stack:
  pose_source: localization
  detector: od_gt
  localization_args:
    use_lateral_velocity_constraint: false
    control_state_use_vehicle_pivot: false
```

현재 사용자 승인 승격과 동일입력 Smooth 검증/남은 폐루프·LAN gate는
`261015_GPS_SHADOW_CLOSED_LOOP_IMPROVEMENT.md` 마지막 절을 따른다.

gt_source=competition은 heading/velocity 입력이지 GT 위치 대체가 아니다. 개발 초기 위치만 Ego를
사용한다. 음영 출발점은 공식 START가 아니므로 initial_position_source=official_start를 임의로
선택하지 않는다. Controller는 Fusion에서 /localization/control_kinematic_state, GT에서
/localization/kinematic_state를 쓴다. Local odometry 직접 controller remap은 지원하지 않는다.

## 6. 녹화와 실제 메시지 검사

GUI **측위 분석 (센서 원본 제외)** 또는 localization_analysis preset/profile을 권장한다.
analysis_mode=true는 별개 인지 분석이다. 측위 프리셋은 BEV frame timing·planner 계산시간
자동 발행을 켜지 않는다. 해당 부하 비교는 별도 latency/perception 실험으로 분리한다.

```text
원시 입력/독립 GT:
  /Ego_topic /Competition_topic /competition/raw_twist /sensor/gps /sensor/imu
Fusion 내부 (localization에서만 필수):
  /localization/imu /localization/gt/orientation /localization/gt/twist
  /localization/constraints/lateral_velocity  (soft ON에서 발행·녹화 확인)
  /localization/gnss/odom /localization/odometry/local /localization/odom
  /localization/control_kinematic_state /localization/initialpose
공통 상태/TF/제어:
  /localization/kinematic_state /localization/acceleration /tf /tf_static
  /vehicle/status/gear /vehicle/status/control_mode /CollisionData
  /ctrl_cmd /control/command/steering /control/autoware/controller_valid
  /control/autoware/failure_reason /purepursuit_autoware/debug/lateral_error
후단/경로:
  /Object_topic /perception/object_recognition/detection/objects
  /perception/object_recognition/tracking/measurement_objects
  /perception/object_recognition/tracking/objects /perception/object_recognition/objects
  /planning/mission_planning/route
  /planning/scenario_planning/lane_driving/behavior_planning/path /planning/trajectory
```

/clock·/diagnostics·/rosout·controller debug·/control/command/control_cmd 등도 요청 목록에 있다.
미발행인 topic을 저장됐다고 하지 않는다. /clock은 live wall-time에서 없어도 정상이고,
현재 제어 경로에 없는 command topic은 optional이다. GT pose에서 Fusion 내부 topic은 필수에서 빠진다.
initialpose 같은 latched 시작 메시지도 실제 count를 확인한다.

개인 토픽 JSON이나 selected_topics로 바꾸면 **직접 선택** 모드가 될 수 있다.
이것은 측위 전용 completeness gate를 켜는 것과 다르다. basic/custom만으로 전체 replay 입력이
저장됐다고 가정하지 않는다. Custom이 필요하면 localization_required_topics()와 실제 count를 별도 대조한다.

종료 후 recording_audit.json을 확인한다.

- topics[topic].message_count > 0: 실제 bag 저장 근거. 0은 미발행·미녹화일 수 있다.
- complete_for_localization_replay: 원시 입력·GT·static TF의 count gate.
- complete_for_localization_assessment: 주행 시작과 조건별 state/제어/후단의 count gate.
- outage_phase_coverage_verified=false: 음영 전/중/복귀 커버리지·heading·시간·독립 GT 품질은 자동 인증하지 않았다.

include_sensors=false는 **카메라/LiDAR bag 제외**이지 MORAI 센서 생성 OFF가 아니다.
GNSS/IMU는 측위 프리셋에서 계속 기록한다. 영상·RViz·센서 부하는 A/B 간 동일하게 유지한다.

## 7. 안전 gate와 실제 종료 확인

validation.localization_safety.enabled=true에서 다음 근거를 요구한다.

1. 로드 뒤 새 Ego·gear·control mode·Collision·Competition·GPS·IMU, 속도 ≤0.05m/s가 새 표본으로
   1초 지속, P와 ExternalCtrl, 음영 진입 전 유효 GPS.
2. 준비 중 새 CollisionData 접촉을 거부한다. Route/첫 trajectory 뒤에도 P·정지·freshness를 확인한다.
3. /ctrl_cmd 소유자는 /longi_pi_autoware 하나이며 현재 제어기의 새 명령을 확인한다.
4. 주행 중 Ego·state·gear/mode·collision·control receipt freshness를 감시한다.
   Fusion은 Competition·GPS·IMU·Smooth도 감시한다. 음영의 0 GPS는 허용하지만 무수신/stale와 구별한다.
5. Controller 종료 → brake → raw Ego 실제 정지 확인 → D 송신기 종료 → 전용 P 송신 →
   **새 gear P와 실제 정지 확인** → 송신기 종료 → 녹화 마감/audit.

Case safety_receipt.json, 결과 JSON의 safety·drive_reason·halt_campaign을 본다.
shutdown.status=VERIFIED와 stop.park_verified=true가 관측 근거다. 보낸 brake/P 명령이나
controller process 종료만으로 실제 P·정지를 선언하지 않는다. 확인 실패는
SAFETY_STOP_UNCONFIRMED로 캠페인을 중단한다. 충돌과 safety-enabled 시스템 오류도 다음 장면으로 자동 진행하지 않는다.

일반 주행 충돌은 기존 Ego 근처 충돌/멀리 떨어진 이전 UID 오염 분리 정책을 유지한다.
준비 gate는 새 접촉 보고를 보수적으로 거부한다. Collision 무수신을 무충돌로 해석하지 않는다.
이 기능은 수신/프로세스가 살아 있는 동안의 소프트웨어 gate이지 전원 종료·SIGKILL·MR/네트워크
단절을 견디는 물리 비상정지 장치가 아니다. 정지를 확인 못 하면 다음 주행을 금지하고 사용자가
MR 차량 상태를 확인한다. continue_on_system_error=true로 우회하지 않는다.

## 8. 소스·설치본·실행 파라미터와 공유

결과는 planner_logs/validation/<실험 이름>_<YYMMDD_HHMMSS>/에 남는다.

```text
execution.json / source_config.yaml / config_snapshot.yaml
localization_contract.json / localization_runtime.json
localization_configs/*.yaml / network/*.json / scenarios/*.json / topic_profile.json
stack_logs/commands.json / stack_logs/*.log
각 case:
  input.json / scenario.json / events.jsonl / ros_parameters.yaml
  runtime_receipt.json / ros_parameters_after_control.yaml / safety_receipt.json
  planbag_*.bag / planbag_*.json / recording_audit.json
```

Safety-enabled 실행은 runtime receipt/param dump를 남긴다. 준비 실패에서도
stage=failed_before_control_ready로 수집을 시도하고, 실패하면 status=UNCONFIRMED와 오류를 남긴다.
이 receipt를 제어기 준비 성공으로 해석하지 않는다. 계약은 전달 인자, config/source SHA,
installed launch/script/binary SHA와 controller 소비 state를 보존한다.
missing_installed_files가 남으면 설치 완전성을 가정하지 않는다. Source hash만 같다고 실행 binary가
최신이라고 하지 않는다. ROS overlay·명령·실제 parameter를 대조하고 필요 패키지만 증분 빌드/재시작한다.
전체 param dump에는 이전 노드가 남긴 parameter도 포함될 수 있다. GT 조건의 Smooth parameter가
남아 있다고 Smooth가 실행됐다고 판단하지 말고 receipt의 graph·실제 publisher와 bag count를 함께 본다.
공유 자료에는 `code/source/`, `code/installed/`의 허용된 script/launch/source도 포함한다.
공유 직전 원본 bytes의 계약 hash가 다르면 해당 파일을 복사하지 않고 경고·공유 분석 불가로 표시한다.
Compiled binary는 hash만 보존하며 전체 ROS 의존 패키지 버전까지 고정하는 컨테이너는 아니다.

자동 공유 OFF여도 분석 프리셋은 share_payload를 준비한다. 공유 basic이면 bag이 빠져 사후
분석 불가능하므로 bag/full을 선택한다. 자격증명을 전달하지 않는다.
sharing_audit.json의 READY_FOR_MANUAL_SHARE는 수동 공유용 로컬 준비,
VERIFIED는 실제 remote 파일 검사 완료다.
**주행 PASS / 녹화 가능 / 드리프트 개선 / 공유 완료는 별개의 판정**이다.

## 9. 독립 GT·heading·시간축·분석기 한계

평가기 oracle은 원시 Ego 위치와 보정한 물리 yaw다. 추정 localization/TF를 정답으로 재사용하지 않는다.
map↔morai_map static transform만 좌표 변환에 쓴다. MR −1.347°는 estimator/evaluator에서 각각
한 번 적용하고 이미 보정된 Global/Smooth에는 다시 빼지 않는다. MRP·과거 GT bag에 일괄 적용하지 않는다.

ROS header·bag receipt·monotonic·simulator 시간은 다르다. Live safety는 monotonic receipt freshness,
offline 분석은 header 우선/없는 경우 bag time이다. MR 비실시간율을 실제 velocity scale로 보정하지 않는다.

**원본 재실행의 초기화 gate:** Validator는 ScenarioLoad 전 raw 입력도 녹화할 수 있다.
따라서 bag 전체를 곧바로 현재 localization에 넣으면 로드 전 첫 Ego로 초기화돼,
실제 SCENARIO_READY 이후 시작한 estimator와 다른 초기 조건이 된다. 분석에서 준비
표본을 제외하는 것만으로 이 filter 이력을 없앨 수는 없다. 원본 events/rosout의 실제
초기화 시각·pose를 확인하고, replay도 같은 post-load Ego/Competition 초기화가 되는지
직접 대조한다. 입력을 잘라야 한다면 원시 메시지/header/receipt 시각을 보존하고 원본
static TF도 빠짐없이 유지한다. `rosbag play --start`로 TF까지 누락시키지 않는다.
derived input의 원본/결과 SHA와 제외 규칙을 저장하며, 초기 조건이 다른 whole-bag replay는
production 비교 표가 아니라 별도 진단으로 분류한다.

로드 teleport·긴 gap·stamp 역행을 넘겨 다른 시각 pose를 보간하지 않는다.

```bash
/usr/bin/python3 scripts/evaluation/localization/analyze_localization_stability_bag.py \
  <case의 실제 bag 경로> --heading-offset-deg -1.347 \
  --output planner_logs/analysis/<run_id>/localization_stability.json
```

기존 분석기는 GNSS odom gap을 이용한 일부 음영/복구 분류, Global/Smooth 오차·jump·steering·
TF closure 등을 계산한다. 원시 GPS status/0 좌표의 완전한 phase 분류, 모든 긴 Ego gap/teleport의
안전한 보간 분절, actual Ego route 횡오차·p90·회복 시간의 요청된 전체 통계는 별도 분석이 필요하다.
기존 자동 보고서 숫자만으로 음영 드리프트 해결을 인증하지 않는다.

다음을 음영 전·중·재획득 첫 1초·후속 회복으로 분절해 확인한다.

- Global/Smooth의 독립 GT 횡·종·yaw 오차, 평균·중앙값·p90·p95·최대·표본 수.
- GPS invalid/stale/체류시간, rate/gap·중복·역행·NaN, TF co-time closure.
- 실제 Ego의 route/trajectory 횡오차와 controller debug. 추정 state debug만으로 실제 횡오차를 인증하지 않는다.
- 정상 회피 변위를 drift로 세지 않는다. Steering-rate·sign flip·제어 출력·정차·완주도 함께 본다.
- 재획득 correction/reset·회복 시간, heading/지연 artifact, Ego pose frame-time 불균일.

MR/MRP·gt_source·estimator/evaluator offset·time policy·branch/source/install·고정/변경 변수는
EXPERIMENT_TEMPLATE.md에 기록한다. 일회성 분석은 planner_logs/analysis/<run_id>/,
재사용 가능한 계약과 결론만 docs에 남긴다.

## 10. 구현 지원과 남은 작업

| 기능 | 현재 코드 위치 |
|---|---|
| CLI·새 실행·사본/재개 | runner.py::main/parse_args, executions.py::prepare_execution/load_execution |
| 후보 whitelist·config 전달 | localization_contract.py::normalize_localization_launch_args/effective_localization_args, runner.py::_start_case_stack |
| 보정된 GT opt-in | localization_contract.py::normalize_gt_pose_launch_args/effective_gt_pose_args |
| source/install/config 지문 | localization_contract.py::resolve_localization_contract |
| 측위 녹화·count gate | recording_options.py::LOCALIZATION, recording_audit.py::localization_required_topics/audit_bag, config/internal/topic_profiles/localization_analysis.json |
| 준비/통신/실제 stop/P | runtime.py::LiveMonitor.wait_stationary/localization_watchdog/verify_stopped, runner.py::_stop_and_verify, launch/parking_hold.launch |
| 실제 명령·제어 후 param | runner.py::_capture_runtime_receipt, runtime.py::Processes.start |

추가 기능과 운영 검증은 아래로 구분한다. 구현 지원은 실험 성공이나 production 승격을 뜻하지 않는다.

- **Phase별 평가:** analyze_localization_stability_bag.py 또는 별도 분석기에서 GPS invalid/stale,
  teleport/gap 분절, 전/중/복귀 p90·회복 시간·actual Ego route 횡오차를 구현/회귀 검증한다.
- **시간/GT 품질:** count audit 뒤 content·header/receipt·TF co-time·oracle 불균일을 검사한다.
  complete_for_localization_assessment는 표본 availability이지 정확도 인증이 아니다.
- **후보 의미:** YAML mapping 검사 외 EKF 행렬·remap·noise의 실제 의미를 log/param으로 확인한다.
- **환경 확장:** MR 로컬 뒤 대회 LAN·설치 overlay·팀 PC와 정상/음영/복귀·통신 실패를 검증한다.
- **승격:** ADOPT/REJECT/HOLD와 회귀 근거를 남기고 운영 변경은 별도 승인으로 진행한다.

## 11. 확인된 로컬 기능 시험 범위

2026-10-14 연결된 MR에서 같은 무장애물 JSON으로 기본 Fusion, 보정된 GT pose,
translation tau 0.25초 후보를 각각 1회 실행했다. 세 조건 모두 주행 PASS, 핵심 bag count audit OK,
종료 후 새 Ego 표본과 gear P 정지 확인을 통과했다. 세 bag에서 유효 GNSS → 좌표 0 음영 → 복귀도 관측했다.
후보 tau의 실제 ROS parameter와 별도 실행 사본, 완료된 실행의 재개 시 재주행 생략도 확인했다.
도착 직전 freshness와 공유 hash 검사 보강 후 기본 Fusion을 한 번 더 실행해 같은 gate를 통과했다.
최종 Validator 회귀 테스트 310개를 통과했고 source/설치본의 새 모듈·launch를 일치시켰다.

상세 재현 정보와 경로는 로컬
`planner_logs/analysis/261014_localization_validator_enablement/EVIDENCE.md`에 있다.
자동 Drive 업로드는 OFF로 시험했으며 `READY_FOR_MANUAL_SHARE`는 업로드 완료가 아니다.
이 확인은 **실험 수행 기반이 동작한다**는 의미다. tau 후보 개선, 장애물 회피 조건,
실제 통신 단절·SIGKILL 안전성, 대회 LAN/다른 PC 재현은 인증하지 않았다.
특히 이 MR의 GPS status는 음영에서도 1이므로 status만으로 유효 GNSS를 분류하지 않는다.
