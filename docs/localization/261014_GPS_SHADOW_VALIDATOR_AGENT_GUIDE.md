# GPS 음영 횡방향 드리프트 실험 에이전트 가이드 초안

문서 상태: AGENT DRAFT. 작성 기준: 2026-10-14, Asia/Seoul.
대상은 localization 실험 에이전트와 이 안내를 검토하는 planning_validator 에이전트다.
validator 검토·교정 전에는 실행 승인된 런북으로 취급하지 않는다. 이 문서 작성에서는
시나리오 로드·주행·코드 수정·운영 설정 변경을 수행하지 않았다.

목표는 GPS 음영의 진입·유지·복귀에서 추정 위치의 횡방향 오차와 실제 주행의 횡방향
흔들림을 분리하고, localization 후보를 동일 조건에서 검증하는 것이다. 객체 배치와
자동 로드·주행·녹화에는 기존 validator를 사용한다. Base5 본수집·학습은 범위 밖이다.

## 1 담당과 변경 권한

- Localization 에이전트는 실험 설계·개인 JSON/YAML·오차 분석을 담당한다. 사용자가 별도로
  승인한 범위에서만 localization 후보를 구현하며 production 기본값을 자동 승격하지 않는다.
- Validator 에이전트는 CLI·YAML 지원 범위·설치·녹화·재개·안전 종료 안내를 검토한다.
  빠진 기능은 지원 여부와 필요한 변경을 보고하며 이 초안 검토만으로 구현 권한을 추론하지 않는다.
- 시나리오 배치는 에이전트가 JSON으로 작성한다. Validator 자체가 적절한 객체 위치를
  자동 설계하거나 localization 드리프트를 자동으로 평가해 주는 구조는 아니다.
- MORAI 통제권은 한 에이전트만 가진다. Base5 collector·다른 validator·수동 제어기와
  동시에 실행하지 않는다. 사용자 master나 다른 작업의 프로세스를 일괄 종료하지 않는다.

## 2 필요한 문서와 소스

먼저 이 디렉터리의 [README](README.md), [측위 CURRENT](COMPETITION_LOCALIZATION_CURRENT_STATE.md),
[평가 bag 목록](260924_LOCALIZATION_EVALUATION_BAG_CATALOG.md), [실험 기록 형식](EXPERIMENT_TEMPLATE.md)을 읽는다.
현재 문제가 과거 음영 연속 발행 문제와 같은지 확인하고, 이미 기각한 Q/R·속도 scale·
시간축 후보를 근거 없이 다시 기본값으로 적용하지 않는다.

Validator 작업은 아래에서 해당 부분을 읽는다.

- [음영 미션 안내](../../base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/base/README.md),
  같은 폴더의 `example.yaml`, `manifest.json`, 대표 baseline/normal JSON.
- [제작 가이드](../planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md),
  [실행 런북](../planning/SCENARIO_VALIDATOR_RUNBOOK.md).
- `base_ws/src/planning_addons/planning_scenario_validator/src/planning_scenario_validator/`의
  `config.py`, `runner.py`, `runtime.py`, 녹화 옵션·입력 snapshot 구현.
- 실제 측위 launch는 `base_ws/src/bringup/launch/localization_gnss_imu_gt_ctra.launch`와
  `base_ws/src/localization/localization_manager/launch/gnss_imu_gt_ctra.launch`다.
- 좌표·독립 GT는 [평가 좌표 계약](../bevfusion/GT_EVALUATION_COORDINATE_CONTRACT.md)을 따른다.

문서 날짜·파일명보다 현재 branch의 source, 설치본, 실제 실행 명령과 resolved parameter를 우선한다.
공유 브랜치와 로컬 작업 브랜치가 다르면 Git revision과 submodule·설치본 차이를 기록한다.

## 3 연결과 사전 안전 확인

이 실험은 SaveSensorData 정지 캡처가 아니라 실시간 제어 주행이다. MORAI는 대회 맵·
Sunny13을 유지하고 pause/Capture freeze가 아닌 주행 가능한 상태여야 한다.
Ego는 도로 위 P·정지에서 시작한다. GUI 연결 변경이 필요하면 사용자에게 요청한다.

현재 미션 예제의 입력은 다음과 같다. 실행 YAML과 실제 UI 설정이 일치해야 한다.

```text
Sensor: configs/sensors/Sensor_competition_local.json
Ego Network: base_ws/src/planning_addons/planning_scenario_validator/config/internal/generated/ego_network_validator_local.json
ScenarioLoad: UDP 127.0.0.1:9095
```

`ego_network_mr_base5_savesensordata_collision_local.json`은 이번 주행 입력이 아니다.
Localization-only 경량 센서를 쓰려면 `Sensor_localization_gt_lightweight.json` 지원과 실제 UI
적용을 확인하고 별도 실험으로 기록한다. A/B 중에 센서나 네트워크를 몰래 바꾸지 않는다.

사용자는 연결과 기존 master 준비를 맡고 validator가 수신·측위·후단·planning·제어기를
소유하는 형태를 우선한다. 중복 provider·TF·detector·UDP control publisher, 디스크,
Competition/GPS/IMU/Ego·충돌 수신과 신선도를 확인한다. GPS는 음영 진입 전 유효해야 한다.
충돌 무수신을 무충돌로 해석하지 않는다. 통신이 불확실하면 주행하지 않는다.

## 4 개인 작업 입력과 CLI

개인 입력은 아래 workspace에 둔다. 공유 `base/`의 JSON·YAML과 원본 샘플을 수정하지 않는다.

```text
base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/workspace/
  loc_shadow_baseline.yaml
  loc_shadow_gt_pose.yaml
  loc_shadow_candidate.yaml
  localization_topics.json
  scenarios/
```

GUI 없이도 예제 전체를 개인 파일로 복사·수정하고 CLI로 실행할 수 있다. 빈 YAML을 만들지 않는다.
`example.yaml`은 현재 24장 기본 미션이고 `static_obstacle_gt.yaml`은 다른 시나리오 폴더·
경량 센서·시간을 사용하는 별도 예제다. 이름의 GT만 보고 같은 장면 A/B로 취급하지 않는다.
동일 예제 복사본에서 `pose_source` 또는 `detector`만 바꿔 비교 조건을 만든다.

CLI의 현재 source 진입점과 설정 검사 예시는 아래다. 검토자가 최신 구현과 설치 환경을 확인한다.

```bash
cd /home/luv/AjouNice2026
source scripts/workspace/source_aw149.sh
GUIDE_CASE="base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/workspace/loc_shadow_baseline.yaml"
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/planning_validator.py "$GUIDE_CASE" --check
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/planning_validator.py "$GUIDE_CASE" --dry-run
```

검사·실행 권한·MORAI 통제권 확인 뒤에만 같은 CLI에서 `--check`/`--dry-run`을 빼고 주행한다.
주행 CLI에는 `--run`을 붙이지 않는다. 새 조건은 새 실행으로 분리한다.
`--execution <기존 결과 폴더>` 재개는 저장된 입력 사본을 사용한다. 원본 YAML만 바꿔
기존 기록을 재개해도 후보가 적용되지 않는다. 코드·센서·실행 조건이 달라지면 강제 resume하지 않는다.

## 5 실험 순서

| 단계 | 장면과 스택 | 확인할 사항 |
|---|---|---|
| A | `hid_soa_baseline_00` 무장애물, localization + od_gt | 실제 GNSS 음영·전후 구간 통과, 측위와 제어의 baseline |
| B | 같은 JSON·route·제어, maptf_morai + od_gt | 지도·route·제어·GT pose 정합의 원인 분리 |
| C | 한두 정적 장애물 또는 정지 차량, localization + od_gt | 회피와 체류시간 증가 때의 드리프트·복구 |
| D | A/C의 같은 입력, localization 후보 하나만 변경 | 횡·종·yaw·조향·발행 연속성의 개선과 부작용 |
| E | 일반 장면·다른 진입 조건과 장애물 위치 | 같은 실패 장면에 맞춘 과적합 여부 |
| F | 채택 가능 후보, 실제 E24 포함 | 인지·부하까지 포함한 폐루프 회귀. 별도 실행 요청과 환경 확인 필요 |

각 단계는 대표 한 장면부터 확인한다. 24장 전체가 live 검증됐다고 가정하지 않는다.
몇 회 반복할지는 첫 결과의 변동을 보고 정하며 MORAI physics까지 seed로 고정됐다고 주장하지 않는다.
`od_gt`는 detector/GPU 원인 분리용이지 모든 인지·TF·tracker 오차가 사라지는 oracle은 아니다.
`maptf_morai` 성공을 실제 localization의 개선으로 보고하지 않는다.

공식 음영 영역, 시작점·목적지·경로와 조건별 객체는 우선 고정한다. 객체 UID·DataID·지면·
도로 링크·BBox 간격을 확인하고 좌표를 임의 추측하지 않는다. MORAI actor 좌표와 planning
map goal 좌표는 다르다. 신호등은 첫 음영 실험에서 OFF를 유지하고 동적 끼어들기는 추가하지 않는다.

## 6 Localization 후보가 실제로 실행되는지 확인

초안 작성 당시 `runner.py::_start_case_stack`은 localization에 `gt_source=competition`,
heading correction ON/−1.347°, control state ON, quantized control OFF, smooth tau 0.5/0.5를
명시적으로 전달한다. Controller는 localization에서 `/localization/control_kinematic_state`,
GT pose 대조에서 `/localization/kinematic_state`를 사용한다.

현재 YAML에 임의 `localization_args` 같은 키를 추가해도 전달되는 기능이라고 가정하지 않는다.
Q/R config 경로·후보 launch 인자·초기화·controller 소비 state를 바꾸려면 지원 경로를 먼저
확인한다. 누락된 전달 기능은 validator 담당자에게 별도 변경 요청으로 제시한다.
다른 provider를 수동으로 함께 켜거나 공용 production 파일을 모든 조건에 덮어써 A/B하지 않는다.
Source를 수정한 실험은 관련 패키지 증분 설치·노드 재시작·actual command/ROS param 확인이 필요하다.

`gt_source=competition`은 Competition heading/velocity 입력이라는 뜻이며 완전한 GT 위치로
localization을 대체했다는 뜻이 아니다. 음영 시작점은 공식 START가 아니므로 임의로
`initial_position_source=official_start`를 적용하지 않는다. 개발 초기 Ego 위치와 연속 GT 주입을 구분한다.

## 7 기록과 독립 평가

기본 `basic.json`만으로 모든 localization 내부 입력이 저장된다고 가정하지 않는다.
개인 토픽 프로필에 아래 실제 발행 토픽을 포함하고 종료 후 message count를 확인한다.

```text
/Ego_topic /Competition_topic /competition/raw_twist /Object_topic
/sensor/gps /sensor/imu /localization/imu
/localization/gt/orientation /localization/gt/twist
/localization/gnss/odom /localization/odometry/local /localization/odom
/localization/kinematic_state /localization/control_kinematic_state
/tf /tf_static /diagnostics /rosout
/planning/trajectory /purepursuit_autoware/debug/lateral_error
/control/command/control_cmd /control/command/steering /ctrl_cmd
/CollisionData /vehicle/status/gear /vehicle/status/control_mode
```

실제 ROS 타입·이름·발행자와 추가 diagnostics를 검토자가 확인한다. GT pose 조건에는
fusion 내부 토픽이 없는 것이 정상일 수 있으므로 조건별 필수 목록을 나눈다.
Localization-only에는 카메라/LiDAR 원본 bag이 필수는 아니다. `include_sensors: false`는
MORAI 센서 생성 OFF가 아니며, 영상·RViz 부하도 조건 간 동일하게 유지하거나 명시한다.

Evaluator는 원시 Ego로 독립 GT를 만들며 추정 localization/TF를 정답으로 재사용하지 않는다.
MR raw heading→물리 yaw의 −1.347°는 estimator와 evaluator에서 각자 한 번 적용한다.
이미 보정된 state에는 다시 빼지 않는다. MRP·과거 GT pose bag에는 보정을 일괄 적용하지 않는다.
GT pose 대조의 yaw 계약도 실제 launch와 정지 시나리오 yaw로 확인한다.

ROS header·bag receipt·monotonic·simulator 시간축과 취득 지연을 구분한다. 로컬 MORAI
비실시간율을 실제 속도 bias로 보정하지 않는다. 평가 보간은 긴 공백·ScenarioLoad 순간이동을
넘지 않게 분절하고, 불확실한 시각 표본을 다른 시간 pose로 채우지 않는다.

기존 분석기 `scripts/evaluation/localization/analyze_localization_stability_bag.py`와
`EXPERIMENT_TEMPLATE.md`에서 시작한다. 다음 항목을 음영 전·중·재획득 직후·후속 회복으로 나눈다.

- Global/control state의 독립 GT 대비 횡·종·yaw 오차: 평균·중앙값·p90·p95·최대와 표본 수.
- GPS invalid/stale 구간과 체류시간, state rate·gap·중복·역행·NaN, TF co-time closure.
- 실제 Ego의 route/trajectory 대비 횡오차와 controller debug 값. 추정 state 기준 debug만으로
  실제 차량의 횡오차를 인증하지 않는다. 정상 장애물 회피 변위를 드리프트로 세지 않는다.
- steering-rate·sign flip·제어 출력·정차·완주, 재획득 correction·reset·회복 시간.
- Heading·sensor delay가 만든 가짜 횡오차, Ego oracle의 frame 시간 불균일 가능성을 별도 진단한다.

일회성 bag 분석은 `planner_logs/analysis/<run_id>/`, validator 녹화는
`planner_logs/validation/<campaign_timestamp>/`에 보존한다. 재사용 가능한 결론·계약만 docs에 정리한다.

## 8 종료와 승격 경계

Collision·topic loss·route 실패·timeout은 원본과 로그를 남긴다. 실패를 PASS로 바꾸거나
안전 기준을 완화하지 않는다. 제어기 종료·brake·송신기 종료·bag 마감 순서를 유지한다.
Braking 명령을 보냈다는 사실은 실제 P·정지 확인이 아니다. 통신 종료 후 추정으로 Park를 선언하지 않는다.
공용 master 유지와 시험 소유 노드 정리를 구분하고, 불확실한 종료에서는 다음 자동 주행을 막는다.

PASS는 목적지·시간·충돌 조건의 결과다. 녹화 완전성과 측위 드리프트 해결 여부는 별도다.
후보는 효과·회귀·한계를 ADOPT/REJECT/HOLD로 기록하고 운영 승격은 별도 승인으로 수행한다.
대회 LAN·팀 PC 검증을 로컬 한-PC 시험으로 대체하지 않는다.

## 9 Validator 에이전트 검토 목록

- [ ] 최신 CLI·workspace·snapshot/resume 안내가 실제 구현과 일치하는가.
- [ ] Localization 후보 인자/config 경로를 안전하게 전달·기록할 기존 경로가 있는가.
- [ ] 소스 수정 뒤 설치본·실행 command·resolved param 차이를 확인할 수 있는가.
- [ ] 무장애물 기준선·GT pose·od_gt 조건의 준비/필수 토픽 차이를 반영했는가.
- [ ] 로드 후 Ego 초기 안정·제어 소유권·충돌·통신 실패·실제 정지 확인이 충분한가.
- [ ] 개인 토픽 프로필이 실제 GNSS·IMU·Competition·Local/Global·Smooth·TF·제어를 저장하는가.
- [ ] 모델 E24를 선택한 후속에서 실제 모델 receipt를 확인하며 구 E25 기본을 혼용하지 않는가.
- [ ] 브랜치·모델·센서·시간·heading·GT provenance와 분석/공유 묶음이 보존되는가.

검토 결과는 지원됨/초안 교정/기능 변경 필요/미검증으로 구분하고 관련 코드 위치를 남긴다.
필요한 validator 구현 변경은 별도 목록으로 제시한다. 검토만으로 MORAI를 실행하거나
계측·제어·측위 기본값을 수정하지 않는다. 검토 완료 뒤 문서 상태와 실제 검증 범위를 갱신한다.
