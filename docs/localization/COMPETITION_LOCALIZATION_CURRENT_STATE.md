# Competition Localization 현재 계약

상태: CURRENT. 코드 대조·문서 정리: 2026-10-24. 보존된 주행 근거의 마지막 갱신은
2026-10-15이며, 이번 정리로 LAN/공식 checkpoint 검증이 추가된 것은 아니다.

## Production

| 항목 | MR·실대회 | MRP |
|---|---|---|
| alias / motion source | `localization` / `/Competition_topic` heading, velocity.x | `localization_mrp` / `/Ego_topic` |
| heading correction | `-1.347 deg` | `0 deg`; 시나리오·지도 yaw 독립 대조 미완료 |
| 초기 위치 / yaw | 대회 명시 옵션: 공식 START 위치 / 보정된 Competition heading. 개발 alias 기본은 첫 Ego 위치 | 첫 Ego 위치·heading |
| GNSS horizontal sigma / Global Qxy | `0.10 m` / `0.0005` | 동일 |
| Global-only soft body vy prior | ON, sigma `0.20 m/s` | 동일 기본값; MRP 폐루프 미검증 |
| Smooth longitudinal / lateral / yaw tau | `0.5 / 0.1 / 0.5 s`, vehicle pivot | 동일 |
| lateral correction rate cap | `0.3 m/s`, 차량 운동이 아닌 **Global 위치 보정 성분**에만 적용 | 동일 |
| GNSS motion compensation | 기본 OFF; 선택 `gnss_delay_ms:=0..100` (기존 초 인자 호환) | 동일 기본값; 별도 측정 필요 |
| relocation guard | `enable_relocation_guard=false`, monitor-only 기본 true | 동일 |
| quantization-aware control candidate | `use_quantized_gnss_control_state=false` | 동일 |

공통 고정 계약:

- status heading/vx와 IMU yaw-rate/ax는 Local/Global EKF에, GPS map XY는 Global에만 전달한다.
  IMU ay mask는 이미 OFF다. soft vy prior는 실측 Competition vel_y가 아닌 모델 prior다.
  Local에는 이 prior를 넣지 않으며, launch 인자가 publisher와 Global mask를 함께 확정한다.
- `two_d_mode=true`, `predict_to_current_time=true`, `publish_sensor_tf=false`.
- [UDP receipt-time](morai_udp_receipt_time.md), `/use_sim_time=false`가 live 계약이다.
  receipt-time은 userspace dequeue 시각이지 물리 measurement/NIC 시각이 아니다.
- provider는 한 번에 하나만 실행한다. 대회에서 Ego 직접 GT provider는 fallback이 아니다.
- heading correction은 MR convention 보정이다. 이미 보정된 localization 출력에 재적용하지 않는다.
  raw Ego oracle을 평가할 때만 해당 실행의 estimator와 같은 offset을 준다.
- production `velocity.x` scale, Q/R, UDP timestamp를 임의 변경하지 않는다.

## GNSS 지연 보상 선택: ms 실행 인자

2026-10-29부터 기존 위치 전진 보상에 `gnss_delay_ms` 실행 인자를 제공한다.
양수20은 GNSS map XY에 `vx × 0.020 × [cos(yaw), sin(yaw)]`를 더한다는 뜻이다.
추가로20ms 대기하거나 UDP/관측 header를 변경하지 않는다. EKF 과거 상태 재필터링이 아닌
기존 단기 정속·방향 전진 근사이며, 이미 보정된 map yaw에 heading offset을 다시 주지 않는다.

```bash
localization gnss_delay_ms:=20
# 대회 공식 START 초기화는 별도이며 반드시 함께 명시한다.
localization initial_position_source:=official_start gnss_delay_ms:=20
# 짧은 현장 요약 화면도 동일 인자를 그대로 전달한다.
field localization initial_position_source:=official_start gnss_delay_ms:=20
```

인자를 생략하거나0을 주면 기존 OFF다. 음수·100ms 초과·NaN/Inf는 시작 시 거부한다.
기존 `gnss_motion_compensation_sec:=0.02`는 유지하되, nonzero 초 인자와 ms 인자를
함께 지정하면(같은20ms라도) 오류로 알려 중복 보상을 막는다. 실행 중 동적 변경하지 않으며
값 변경은 정차 후 해당 provider 재시작으로 적용한다. provider를 두 개 동시에 실행하지 않는다.

시작 원문에 `GNSS advance 20.0ms; receipt stamps unchanged; no added wait`를 기록한다.
보조 motion/heading이 없거나 오래되거나 stamp pair가 맞지 않으면 그 fix는 **무보상 원본으로
발행**하며 `GNSS motion compensation fallback` 경고를 낸다. GPS zero 거부는 유지한다.
이는 GNSS 음영 중 누적 drift나 시뮬레이터 부하로 위치·속도 자체가 어긋나는 문제의 해결책이 아니다.

| 측정 환경 | 먼저 비교할 후보 | 근거·한계 |
|---|---|---|
| 팀장님 제공 LAN bag | `10ms` | raw GNSS 상대epoch 약12ms; 입력단 종 MAE16.75→8.40cm |
| 희중님 제공 LAN bag | `20ms` | raw GNSS 상대epoch 약22ms; 입력단 종 MAE20.32→5.25cm |
| luv 로컬 MORAI + E12, RViz OFF | `30ms` | 두 주행 상대epoch 약32ms; 입력단 종 MAE약36→8cm |
| luv 새 무객체 od_gt, RViz OFF | `10~15ms` | 원큐OFF 추천15ms; 실제ON15ms에서 Global 종 MAE11.99→4.97cm |
| luv 경량GT / 다른 부하·PC / 현장 새 구성 | 기본 `0ms`부터 대조 | 경량GT는약12~18ms, 옛 현장 출력은약4~6ms로 다름 |

초기 LAN/E12 수치는 **같은 bag 입력을 전진시킨 오프라인 후보**이며 사람별 영구 calibration이나
전체 EKF·실제 차량 정차/음영 성능 보장이 아니다. 실제 callback 순서에 따른 보상 적용률도
별도 확인해야 한다. [수치·provenance](../../planner_logs/analysis/261029_gnss_delay_profiles/REPORT.txt)를 따른다.
별도 [실제 OFF/ON 결과](../../planner_logs/analysis/261029_gnss_delay_live_compare/REPORT.txt)는 위 새od_gt 조건에만 해당한다.
공통 기본값은0ms이고 사용자별 후보를 alias에 몰래 고정하지 않는다.

Validator의 개인 YAML에서도 `stack.localization_args.gnss_delay_ms: 20`을 선택할 수 있다.
일반 validator GUI의 **실행 탭 → 측위 옆 `GNSS 보상(ms)`**에서도 같은 값을 입력한다(0~100, 기본 0).
YAML 저장 후 새 실행부터 적용되며 실행 중에는 잠긴다. GT 측위에서는 비활성화되고 GNSS 보상을 적용하지 않는다.
기존 초 단위 값은 ms로 표시하되, 숫자를 바꾸지 않고 저장하면 원본 단위를 유지한다.
빠른 GT 대조는 임시 실행 사본에서만 미사용 측위 인자를 제외하며 원본의 GNSS 설정은 보존한다.
기존 실행 snapshot·resume 인벤토리는 바꾸지 않는다. 보정값이 다른 시험은 새 실행으로 시작한다.

## 원큐 측정: 녹화와 validator

각 PC의 **현재 부하·센서·네트워크 구성에서** 새로 측정한다. 사용자명으로 값을 자동 선택하지 않는다.
기존 `ajou` 환경에서 다음 둘 중 하나만 실행한다.

```bash
# 수동 풀스택(측위 + od_gt + planning/control)이 이미 주행 중: 별도 터미널
gnss_delay_record --duration 90
# 수동 스택은 종료, rc와 MORAI만 연결: 실제 validator 주행부터 추천값까지
gnss_delay_validator
```

alias 없이도 저장소에서 `bash scripts/evaluation/localization/record_gnss_delay.sh` 또는
`./planning_validator --gnss-delay-calibration`을 실행할 수 있다.
Validator 모드는 K-City·`ego_network_validator_local`·`Sensor_competition_local`을 Load/Connect해 둔다.
다른 PC 네트워크/goal은 `--config <그 PC의 무객체 캠페인.yaml>`으로 명시한다. 원본 YAML은 덮어쓰지 않는다.
MORAI/roscore를 자동 실행하거나 수동 스택을 강제 종료하지 않는다. 이미 주행 노드가 있으면 이유를 알려 거부한다.

기본은 동일 무객체 경로2회, 실제 MR/Competition 측위·od_gt·기존 planner/controller, RViz/신호등/영상은 OFF다.
개인 파생 시나리오에서 GPS음영만 제거해 정상 GNSS 이동 표본을 모은다. 원본·동결 시나리오는 변경하지 않는다.
보상값은 기본0ms, `--gnss-delay-ms 20`으로 명시할 수 있다. 추천값을 자동 적용하거나 두 번째 주행에 몰래 반영하지 않는다.
`--planviz`로 부하 조건을 맞출 수 있고 `--keep-gps-shadow`는 음영 회귀용이다. od_gt에서 얻은 값을
더 무거운 BEVFusion 동시실행이나 다른 시뮬레이터 PC에 그대로 검증된 값이라고 적용하지 않는다.

**Codex 수행 순서:** 요청자가 rc/MORAI만 연결했다고 하면 중복 스택 확인 →
필요한 source/installed 버전 확인(불일치면 주행 전에 해당 패키지 증분 설치) →
`./planning_validator --gnss-delay-calibration` → 모든 회차의 실제 stop/P receipt 확인 →
`analysis/result.json`의 `status`·`recommendation_ms`·표본/불확실성 확인 → 값과 한계를 보고한다.
보상 ON 검증도 요청받았으면 같은 설정·부하로 OFF/ON 새 실행을 하고 최종 Global/Smooth 오차를 별도 비교한다.
숫자가 나온 것만으로 production 보정값이나 폐루프 정확도 인증으로 승격하지 않는다.

새 결과는 고유 `planner_logs/analysis/gnss_delay_calibration/<id>/`에 보존한다.
`workflow.json`, `validator.log`, 실제 validator execution·bag, `analysis/result.json`·`report.txt`를 함께 남긴다.
수동 recorder는 `planner_logs/analysis/gnss_delay/manual/<id>/`에 별도 저장한다.
카메라·LiDAR는 bag에 넣지 않는다. 수동 recorder의 Ctrl-C는 **녹화만 종료**하고 분석하며 차량은 정지시키지 않는다.
Validator는 자체 안전 정지/P 확인에 실패하면 성공 추천을 내지 않으며 기록을 보존한다.

분석에는 개발용 `/Ego_topic` oracle, raw `/sensor/gps`, 기록된 map/morai_map TF가 필요하다.
Ego 미사용 실제 대회 설정에서는 이 측정을 수행할 수 없다. Ego를 대회 측위 fallback으로 켜는 기능이 아니다.
최소 이동 표본·이동 시간·속도 변화·시차 추정 안정성이 부족하면 **추정 불충분**으로 끝낸다.
고정 위치 오차를 지연으로 오인하지 않도록 bias+속도 fit을 함께 본다. 수치는 물리적 GPS/UDP 전송시간이 아닌 상대 관측 epoch다.
기존 보상 ON bag도 **raw GNSS**로 계산하므로 추천값은 현재 설정에 더할 값이 아닌 **절대 설정값**이다.

기존 기록만 다시 분석할 때는 다음처럼 실행한다. 입력 bag·원본 실행 폴더는 변경하지 않는다.

```bash
gnss_delay_record --bag /path/to/record.bag
gnss_delay_validator --analyze-execution /path/to/validator_execution
```

## 출력·TF·GPS 음영

Local EKF는 `odom -> base_link`를 발행한다. 최종 publisher는 Local pose를 Global
stamp로 외삽해 `map -> odom`을 합성하며, EKF stamp를 보존한 map-frame base_link
Odometry `/localization/kinematic_state`를 발행한다. 최신 TF를 무시각 조회해 합성하지 않는다.

vehicle-pivot Smooth는 같은 시각 Local 운동과 Global correction을 결합해
`/localization/control_kinematic_state`를 발행하며 TF는 발행하지 않는다.
Planning/TF의 Global 출력 계약은 유지한다. Local odometry만 controller로 단순 remap하면
map trajectory와 frame이 달라지므로 금지한다. 이 문서는 planning/control 정책을 변경하지 않는다.

GPS `(0,0)`은 거부한다. 최초 유효 z 이후 GPS가 stale/invalid이면 마지막 z만 유지하고
Global의 planar state를 계속 발행한다. 최초 z조차 없으면 startup-ready가 아니다.
state 약50Hz 연속성과 실제 위치 정확도는 별개다.

## 초기화·checkpoint

합의한 대회 공식 START 출발에서는 다음 옵션을 명시한다. bare `localization`의
개발 기본값 `initial_position_source=ego`는 바꾸지 않았으므로 대회에서 인자를 생략하지 않는다.

```bash
localization initial_position_source:=official_start
```

규칙의 `start_finish.xyz`를 `morai_map -> map`으로 변환하고 yaw는 보정된
Competition heading을 사용한다. 공식 yaw와 차이 `1 deg` 초과는 경고만 남기고 관측
heading으로 초기화한다. START와 GNSS 거리 `3 m` 초과, GNSS/heading stamp 차이
`0.5 s` 초과면 고정 START 위치를 잘못 적용하지 않도록 초기화를 대기한다.
이 모드는 Ego를 구독/대기하지 않는다. [현장 원본·Ego 미사용 명령](../../configs/morai_field_reference/README.md#합의한-대회-실행-기준)을 함께 따른다.
MR 로컬 출발 정합은 통과했으나 새 LAN 검증 완료는 아니다. `Published GNSS+IMU initial pose from official_start` 로그와
`/localization/initialpose`를 확인한다. EKF는 센서만으로도 출력할 수 있어 odom 발행만으로
공식 START 초기화 성공을 판정하지 않는다. 사고 후 복귀에 START 초기화를 반복 적용하지 않는다.

보존된 운영 기록은 수정안 START+CP1–14와 Spawn 툴0–14가 일치한다고 정정했다.
원본 실행명 CP13/15는 구번호(현행12/14)이므로 원문을 고쳐 쓰지 않는다.
변경 전 로컬 툴14에서는 raw NMEA zero가 관측됐다. 운영측은 CP14 좌표·spawn을
유지하고 음영 범위를 조정하겠다고 답했으나 **새 환경 배포·GPS 수신·재출발 검증은 대기**다.
[공식 안내 보관](../competition/261015_공식_체크포인트_안내_보관.md)과 로컬 관측을 구분한다.
runtime rule monitor의 구15점 표 동기화는 이 정리 범위에서 수행하지 않았다.

유효 GNSS 복귀에서 Global/Smooth 정합 대기, fresh route, 기존 제어 재개를 검증하는
것이 우선이다. GPS도 새 절대위치도 없는 teleport는 soft prior/ZUPT/Q 조정으로 복구할 수 없다.
개발 helper의 known-checkpoint prior는 명시적 실험 입력이며 대회 기본 입력이 아니다.

guard는 OFF/monitor-only를 유지한다. active 실험은 정차 1초·GNSS 위치 안정 1초,
fresh/frame/quaternion/covariance 검사, 5m innovation 3표본, Global ACK를 요구한다.
timeout/발행 실패는 blocked latch다. Local XY를 강제 reset하지 않으며 heuristic은
공식 checkpoint 이벤트나 자동 제어 재개 허가가 아니다.

## 근거와 남은 gate

[CURRENT_EVIDENCE](CURRENT_EVIDENCE.md)는 현재 채택 근거와 역사적 후보를 구분한다.
MR 실제 soft/Smooth 완주·음영 pair·30초 Park hold 근거는 있지만 LAN, MRP 별도 폐루프,
slip/충돌, 변경 환경 CP14 및 배우·미션상태를 포함한 checkpoint 복귀는 미완료다.
20ms GNSS 보상은 입력별 개선/악화가 역전돼 기본 0초를 유지한다.

변경 승인 순서: 동일 raw bag offline A/B → `od_gt` 경량 폐루프 → GPS 음영
진입·유지·복귀 → LAN-to-LAN 동일 controller → production 승격.
횡/종 오차·frame jump·yaw·rate·steering-rate/sign flip을 함께 본다.
2026-10-15 soft/개선 Smooth는 사용자 승인으로 기본값이 됐으며, 승인과 로컬 결과를
미완료 LAN/slip gate의 통과로 기록하지 않는다.

Fast-LIO/직접 GT/구형 GNSS provider와 opt-in 연구 node는 설치·공통 launch·테스트
호환성 때문에 남아 있다. 그 경로가 존재한다는 사실은 production 선택 근거가 아니다.
