# Tracking·Prediction production 현재 상태

상태: CURRENT. 문서·launch 대조: 2026-10-29.
Detector는 [BEVFusion CURRENT](../../../../docs/bevfusion/CURRENT.md#runtime)의 E12이며 E24/E25는 명시적 복귀다.
현재 E12/E24 detection은 geometric/gravity center다. bottom center와 섞지 않고 fresh stack으로 전환한다.
과거 bag 실측·긴 설명은 [정리 전 원문](legacy/context_261024/TRACKING_PREDICTION_CURRENT.md)에 보존했다.

## Production 경계

```text
DetectedObjects → EKF → UNKNOWN stationary policy → Lanelet2 Map-CV → PredictedObjects
```

운영 진입점은 `docs/common_alias.txt`의 `tracking`이다. detector·localization은 별도 프로세스다.
다른 detector/tracker를 같은 canonical topic에 동시에 발행하지 않는다.

| 출력 경계 | Topic |
|---|---|
| Detection | `/perception/object_recognition/detection/objects` |
| Measurement track | `/perception/object_recognition/tracking/measurement_objects` |
| Motion-policy track | `/perception/object_recognition/tracking/objects` |
| Prediction | `/perception/object_recognition/objects` |

Production `tracking`/`tracking_prediction.launch`는 Route A/B 공통 구간에서 **판단에 보내는 Prediction만 vehicle-only**로 제한한다. 검출·measurement track·motion-policy track은 그대로 남으므로 이 정책은 객체 분류나 tracker 자체를 고치는 기능이 아니다. C 신호 동적 횡단 객체를 지난 lanelet `200258`의 30m 지점(지도 약 **71,587**)에서 ON, 고주로 끝 lanelet `200088` 종단(약 **202,160**)에서 OFF다. 로터리·고주로 전체를 포함하고 GPS 음영 예제의 가장 가까운 객체까지 해제 지점에서 약 118m다. 전체 그림은 `python3 scripts/evaluation/render_vehicle_only_corridor_map.py --map-version 94 --output /tmp/vehicle_only_corridor.png`로 생성한다. 현재 이 경계는 샘플 장면·v94 bag 기반으로 선택한 것으로 대회 객체 위치의 보장은 아니다.

v88/v94 지도 SHA와 Route A/B 경로 일치·ego `map` pose·수신 freshness(0.5초)·주행 방향/횡거리(≤12m)가 모두 맞을 때만 필터링한다. 불명확하면 **fail-open**으로 모든 class를 전달한다. 상태는 `/map_based_prediction/diagnostics/vehicle_only_active`에 `Bool`로 나온다. 임시 복귀는 `tracking vehicle_only_corridor_enabled:=false` 후 fresh stack 재시작이다. standalone `object_prediction.launch`는 기본 OFF다. [경계·저장 bag·로컬 MORAI 대조군](../../../../docs/bevfusion/evidence/261027_VEHICLE_ONLY_PREDICTION_CORRIDOR.md)의 ON/OFF 시험을 실제 측위+실검출+LAN 풀스택 안전성 인증으로 확대 해석하지 않는다.

World frame은 `map`이다. 내부 measurement state·30Hz predicted copy·마지막 detector 관측 age를 구분한다.
MORAI `/Object_topic`은 대회에 없으므로 `od_gt`는 부하 분리용 통제 실험이지 production fallback이 아니다.

## Tracker 운영값

| 항목 | 현재 계약 |
|---|---|
| 내부 모델 | CTRV(`prediction_model=1`), position noise 0.40m, detector velocity fusion OFF(7D detector) |
| 출력 시계 | 입력 measurement timestamp state를 current time으로 강제 보상하지 않음. 내부 scheduler 30Hz, 외부 fixed-rate adapter 0Hz |
| Measurement age | `tracking` alias 0.50s. generic/standalone launch 0.25s는 직접 실행 호환값이며 alias를 대체하지 않음 |
| Confirmed coasting | moving/pedestrian 0.50s, UNKNOWN 및 2m/s 이하 CAR/TRUCK 1.00s. stamp 경과시간 기준. tentative는 strict 유지 |
| 중복 입력 | ROS bridge의 same-class rotated BEV IoU 0.50 NMS |
| Duplicate birth guard | association된 mature track 중심 0.75m 이내 유사한 큰 unmatched box 억제. 전역 gate 5m 확장이 아니며 작은 pedestrian에 확대하지 않음 |
| 소형 UNKNOWN birth 후보 | `suppress_small_unknown_duplicate_birth=false` 기본 OFF 확정. 5개 실제 ON 대조와 근접 별개 객체의 측정오차 반례를 검토해 승격하지 않음. 명시적 ON 시험 후보로 남김 |
| 정지 UNKNOWN UUID 연속성 | `stationary_unknown_uuid_continuity=true` 운영 launch 기본 ON. 최근 사라진 정지 객체의 유일한 재검출에 출력 UUID만 계승. `false`로 기존 ID 정책 복귀. EKF state·박스·수명은 불변 |
| 차량 yaw | 1.0m/s 이상 motion 방향 canonicalize, enter/exit hysteresis 100/80deg |
| 보행자 순위 | 운영 pipeline `pedestrian_gaussian_association_ranking=true`. 기존 2m gate·birth·수명·차량/UNKNOWN 비용은 유지. generic core/standalone 기본 OFF |
| UNKNOWN motion policy | `force_unknown_stationary=true`: 관측 pose/box는 갱신하고 가짜 속도·가속도·yaw rate를 prediction 전에 0으로 함 |

보행자 복귀는 `tracking pedestrian_gaussian_association_ranking:=false`이며 fresh tracker로 비교한다.
[보행자 근거](../../../../docs/bevfusion/evidence/261014_PEDESTRIAN_ASSOCIATION_AND_LIDAR_MOTION_FOLLOWUP.md)를 따른다.
0.50초 허용치는 새 관측 없이 정확하다는 뜻이 아니다. detector age·coasting·prediction coverage를 함께 본다.
association은 cost 정렬 후 greedy matching이며 Hungarian이 아니다.

## Map-CV 운영값

| 항목 | 현재 계약 |
|---|---|
| Map | 팀 production 소스는 `kcity_map/lanelet_map/lanelet2_map_v101.osm`로 고정. 실제 설치본·기존 프로세스의 `map_file`도 별도 확인. 아래 v100 실험은 그 당시 로컬 설정의 기록 |
| Horizon | **로터리/일반 최대 3초·고주로 최대 10초**, 실제 지도 종단이 먼저면 그 시간까지만 Map-CV. 매칭 자체 실패만 baseline CTRV. step 0.1s |
| Lane match | 최소 속도 0.5m/s, 거리 ≤3.0m, yaw 오차 ≤45deg |
| 경로 | start lanelet 최대 3개, 최종 path 최대 6개, centerline lateral convergence 1.5s |
| 중복 hypothesis | 같은 downstream lanelet·전체 horizon 0.75m 이내 경로를 하나로 묶음 |
| Confidence | heuristic route score 정규화. calibrated probability가 아님 |
| Fallback | map 경로 없는 차량은 signed 속도 CTRV, 비차량은 Cartesian CV |

현재 위치·yaw로 start lanelet을 순위화하고 routing graph successor를 열거한다. CTRV 곡률로 lane rank를 고르지 않는다. Centerline Map-CV와 road-Z 수렴을 유지하며 gravity-center 높이 offset의 미래 보존을 인증하지 않는다.
geometry cache·arc index·독립 메시지 복사·동일 순서 DFS는 출력 동등성을 검증한 계산 최적화다.
[오프라인 CPU 근거](../../../../docs/bevfusion/evidence/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#cpu)를 따른다.
현재 통합 설정의 LC는 최고 confidence 정확한 동점 전체, Cruise inside/outside는 1개, 로터리는 ≥0.05 경로를 검사한다.
실제 consumed object/path는 별도 계측해야 하며 RViz Top-K 노출로 추정하지 않는다.

## 승격 범위와 선택적 후보

### 정지 UNKNOWN UUID 연속성 후보

사용자 승인으로 운영 `tracking`·`tracking_prediction.launch`·통합 `morai_planning_stack.launch`는 UUID 계승 기본 ON이다. `tracking stationary_unknown_uuid_continuity:=false` 또는 통합 launch의 같은 인자로 기존 ID 정책에 복귀할 수 있다. node 직접 실행의 기본값은 OFF이며 `force_unknown_stationary=false`이면 사용하지 않는다. 동시 존재하는 두 객체를 합치거나 검출·track을 삭제하지 않는다.
이전 UNKNOWN의 0.6초 이상 위치·속도 안정성과, 재검출의 공백 2초 이하·중심 0.35m 이하·footprint IoU≥0.35·형상/높이 일치를 요구한다. 현재 또는 과거 같은 raw track 생애에 1m 이내 다른 raw ID를 봤으면 계승 근거에서 제외한다. 잘못된 이웃 형상·class도 근접 이력에서 지우지 않는다. stamp/단조 publisher seq 역행·긴 callback 공백·동일 stamp의 변경된 입력은 이력을 초기화한다. 정상 uint32 seq wrap은 허용한다.
신규 선별 34개 bag의 raw 41,206개에서 Woodbox4/7만 계승했고, 별도 LAN 풀주행 raw 11,153개에서는 전체 출력이 동일했다. 위치·크기·class·공분산·속도·stamp·출력 수명은 변경하지 않았으며 detector AP 개선을 뜻하지 않는다.
실제 격리 ROS 1,086틱의 measurement는 같고 정책/Map-CV의 순차 ID는 3→1이었다. 고정 ego의 실제 BPP/SOA/MPT 대조에서 Woodbox7 후반 전부-0속도 출력은 두 번 모두 38→0틱, 성공 대조군은 0→0이었다. 초기 1틱은 남고, 동일 v99 지도 fixture와 기록 odom에서 계산한 acceleration을 사용했으므로 실제 MORAI 완주 인증이 아니다.
한 번도 함께 보이지 않은 별개 UNKNOWN이 위치 오차로 같은 형상·위치처럼 보이는 경우는 입력만으로 완전 식별할 수 없다. seq가 항상 0인 publisher의 짧은 tracker-only 재시작도 식별할 수 없다. 다른 네 실패는 단일 객체 최적화·목적지 제외 등 별도 원인이며, 동시 검출 중복 전체를 해결했다고 보지 않는다. NMS·gate·coasting·안전 마진은 유지한다.
재현과 원문은 로컬 `planner_logs/analysis/261029_duplicate_cases/`의 `uuid_continuity_v2/summary.json`, `uuid_raw_regression/summary.json`, `actual_ros/comparison.json`, `case_audit/planner_replay_REPORT.md`에 보존했다. 재학습은 필요 없으며 새 패키지 설치 후 tracker/policy를 새로 실행해야 한다.
2026-10-29 후속 강화·반례·최종 source hash는 로컬 `planner_logs/analysis/261029_duplicate_followup/helper_review/`에 보존했다. `processing_time_ms` 기존 선택적 진단 JSON의 `uuid_continuity`에 계승·거절·초기화와 변경 객체 수를 남긴다. 콘솔 출력·새 topic·GT 입력은 추가하지 않는다.
추가 실제 MORAI 대조는 E12/TRT·측위·실제 판단/제어와 로컬 v100에서 4장면×OFF/ON 8회 모두 완주·충돌 0이었다. Woodbox4는 raw ID 2개→전달 ID 1개·SOA 기억 2→1개, Woodbox7은 raw 3개→전달 2개·기억 3→2개로 실제 계승됐다. 같은 주행의 기존 motion policy 대비 UUID 계승 이외 변경은 없었다. Woodbox7의 남은 재검출은 XY 1.063m·IoU 0.0565로 달라 계승을 거절했으며 gate를 넓히지 않았다. OFF도 모두 완주했고 목표 5m 밖 전부-0 궤적은 양쪽 모두 0이므로 역사적 중도 정지의 완치나 v101·전체 코스 인증이 아니다. 원형·목표·지도·모델·판단 설정을 맞춘 실제 기록과 영상은 로컬 `planner_logs/validation/duplicate_live_paired_261029_083146/`, 독립 감사는 `planner_logs/analysis/261029_duplicate_live/audit_paired/`에 보존했다. 사용자 승인 범위는 UUID 계승만 기본 ON이며 소형 birth 후보는 OFF다.

설치된 세 진입점의 기본 ON/명시 OFF 복귀 6개 roslaunch expansion도 확인했다. 강제 ON 인자 없이 설치 launch 기본값을 사용한 Woodbox4 추가 실제 주행은 완주·충돌 0·최종 P 확인, raw ID 3→전달 2개와 실제 계승 1회를 기록했다. 소형 후보는 OFF·거리 0.75m였다. 로컬 v100 단일 장면의 배포 경로 확인이지 전체 production 옵션·v101 풀주행 인증은 아니다. `planner_logs/analysis/261029_duplicate_distance_sweep/default_release_independent/`가 독립 근거다.

### 기존 0.75m birth 억제 거리

같은 C++ 코어·34사례·18,039입력을 0.75/1.0/1.25m로 비교했으며 **확정 track 전체 출력은 모두 동일**했다. 늘어난 거리에서는 미확정 UNKNOWN birth 2개만 줄었고 기존 실패 5개·객체 포착 범위·확정 중복은 개선되지 않았다. 반면 74조건×3거리 합성 대조에서는 mature 객체 옆에 늦게 등장한 별개 비겹침 장애물의 새 track을 지속 억제했다. 1.0m는 중심 간격 0.8/1.0m, 1.25m는 1.2m에서도 누락이 생겼으므로 전역 거리는 **0.75m 유지**다. 실제 발생률이나 모든 근접 장면의 보장은 아니다. 근거는 로컬 `planner_logs/analysis/261029_duplicate_distance_sweep/native/REPORT.txt`와 `adjacent_controls/report.json`이다.

### 소형 UNKNOWN 중복 birth 후보

`tracking suppress_small_unknown_duplicate_birth:=true`로 새 스택을 실행한다. 기존 `suppress_duplicate_track_birth=true`도 필요하며 두 후보는 독립적으로 비교한다. 이미 존재하는 두 track을 합치지 않는다. 같은 프레임에 실제 검출을 받은 유일한 mature UNKNOWN과, 새 unmatched UNKNOWN의 raw XY 크기가 모두 1m 미만·XYZ≥0.15m·중심≤0.25m·높이 차≤0.3m·정렬 치수비≤1.5·회전 IoU≥0.45일 때만 새 birth를 막는다. known-class 이력이나 복수 대응이 있으면 적용하지 않는다.
신규 34bag의 기록 검출 코어 재생에서 UNKNOWN birth 146→145, 중복 GT 대응 frame 863→797이며 0.5/1/2m coverage는 동일했다. 영향은 PE_Drum 한 사례이고 나머지 33개는 동일하다. 추가 21사례 10,082입력의 state·공분산과 14장면 catalog AP는 동일했다. 더 넓은 후보는 보조 객체의 0.5m coverage를 떨어뜨려 제외했다. `planner_logs/analysis/261029_duplicate_followup/native_small_final_summary.json`이 비교·한계·고정 source 근거다.
native 121개·소형 가드 14개 시험은 통과했지만 측정오차가 있는 근접 실객체는 구별 불가하다. 추가 현재 코어 합성 대조에서 실제 0.4m 물체 둘의 중심 간격 0.6m에 각 +0.23/−0.23m 오차를 주어 관측 간격 0.14m·IoU 0.481로 만들면 기존 NMS 0.5는 둘을 보존하지만 소형 ON은 20회 연속 신규 생성을 막아 확정 2→1개였다. 무오차 대조는 OFF/ON 모두 2개다. 관측된 실제 발생률이 아닌 입력 모호성 반례이며, 기존 1.0/1.25m 거리 확대 반례와도 별개다. 제한된 Drum 개선만으로 이 위험을 감수해 기본 승격하지 않고 OFF를 유지한다.
이전 8회와 추가 원실패 5개 ON 실제 주행 모두 소형 억제 발동 0회였다. 새 5개는 OFF 3PASS/2STUCK 대비 ON 2PASS/2STUCK/1COLLISION, 최종 정지/P 5/5였다. Barricade의 PASS→충돌 변화도 억제 발동이 없어 삭제 부작용이나 기능 효과로 단정하지 않는다. 원형·목적지·goal 제외거리는 같고 실제 주요 파라미터 차이는 해당 플래그 하나였다. 충돌 후 안전 중단·P 확인과 정상 재개로 나머지를 수행했으며 안전 gate·판정 이력을 우회하지 않았다. 현재 로컬 v100·독립 센서 입력 대조이고 ON 5개 완주 인증이나 최신 팀 지도/코드 인증이 아니다. 로컬 `planner_logs/analysis/261029_small_guard_on/{decision.json,comparison/summary.json,nearby_counterexample_results.xml}`에 조건·부하 혼재·원본 bag/영상 경로를 보존했다.

### 기존 후보와 지도 예측 계약

- 현재 source 기본 10초는 팀 planning `3e598eb5` 통합으로 넘어온 `config/map_based_prediction.yaml`의 고주로 설정이다. 일반 3초는 유지하며 설치·tracking 재시작 전 live 값은 별도로 확인한다. 이전 8초 시험을 새 10초 폐루프 인증으로 전용하지 않는다.
- 이전 승인·검증 기록(2026-10-24): 사용자·판단팀은 고주로 최대 8초·로터리 최대 3초를 승인했고 NPC 소멸 종단이 먼저면 5.6초·1.8초처럼 가용 Map-CV만 출력했다. CTRV 8초·직선 연장·끝점 padding은 쓰지 않았다. [당시 설정·근거](HIGHWAY_EIGHT_SECOND_HANDOFF.md)는 그 revision의 기록이며 현재 10초 기본값의 근거로 대체하지 않는다.
- 고주로 저속/정지도 지도 근거가 있으면 Map-CV를 사용한다. 방향 매칭 실패는 명확한 고주로 위치·도로 방향 합의가 있을 때 미래 경로만 복구한다. 현재 상태·score·수명은 불변이며 오검출 판별기가 아니다. 소멸차선의 종단을 무조건 지도 결함으로 단정하지 않는다. 기존 3/6초 시험은 [당시 기록](MIXED_PREDICTION_HORIZON.md)에 보존한다.
- [관측 일관성 후보](OBSERVATION_CONSISTENCY_REVIEW.md)의 `shape_consistent_association_ranking=false`, `guard_inconsistent_class_transitions=false`를 유지한다. [실제 ROS·Cruise 확인](OBSERVATION_CONSISTENCY_ROS_VALIDATION.md)은 LC·로터리·최종 제어 폐루프 인증이 아니다.
- 긴 coasting 0.75/1/2초, mature 차량 전용 확대, 빠른 차량 승격, 저점수 EKF 갱신, 초기 방향 guard, 재취득 공분산/reset, lane history·횡이동 순위 후보는 운영 미적용이다.
- 저장 검출 재생·실제 코어·격리 ROS·새 bag·실제 LAN·planner 폐루프를 서로 대체하지 않는다. E12 코어 결과를 새 TRT ROS 30Hz 인증으로 전용하지 않는다.
- detector 9D velocity fusion·temporal lane association·planning track-age gate는 별도 후속이다. 전역 5~7m 병합·속도 강제 양수화·원거리 라벨 삭제는 적용하지 않는다.

## 진단과 근거 선택

`publish_association_diagnostics` 강제 ON 기본은 false다. 최신 binary의 `diagnostics_auto_on_subscribe=true`는
녹화기가 구독할 때 `/perception/object_recognition/tracking/diagnostics/association` JSON을 발행한다.
입력 stamp/index·ID/UUID·연결/생성/억제/coasting/삭제·class vote와 최대 256행/누락수를 기록하며 정책은 바꾸지 않는다.
`snapshot_phase`·`candidate_scope`를 함께 읽고 최근접 한 후보의 gate 이유를 전체 후보로 일반화하지 않는다.
구독자 없이는 상세 수집/직렬화하지 않으며 `tracking diagnostics_auto_on_subscribe:=false`로 자동 모드를 끈다.
인자는 motion-policy·Map-CV에도 전달된다. 구버전 실행 노드는 자동 교체·재시작되지 않는다.
기존 `measurement_objects`는 30Hz copy이므로 assignment 진단을 대체하지 않는다.
processing 진단의 강제 ON 기본도 OFF다. [LAN 계측 절차](../../../../scripts/README.md#lan-두-pc를-함께-계측하기)를 따른다.

| 검토 | 근거 |
|---|---|
| E12 14장면 tracker·속도·3/5/6초·CV/CTRV | [64절](../../../../docs/bevfusion/evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-tracking-map-horizon-261023), [65절](../../../../docs/bevfusion/evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-map-motion-ablation-261023) |
| 낮은 score EKF·planner path 소비·lane 후보 | [48절](../../../../docs/bevfusion/evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-low-score-core-probe-261013), [49절](../../../../docs/bevfusion/evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-map-path-consumers-261014), [50절](../../../../docs/bevfusion/evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-map-cv-candidates-261014) |
| 입력 경합·TF pending 수정·차량 재취득 후보 | [차량 검증](../../../../docs/bevfusion/evidence/261014_VEHICLE_DIRECTION_REACQUISITION_REVIEW.md), [독립 검증](../../../../docs/bevfusion/evidence/261014_INDEPENDENT_VEHICLE_AND_LIDAR_50_51_REVIEW.md) |
| 과거 완주 bag·분모·한계·C++ 전환 gate | [정리 전 상세](legacy/context_261024/TRACKING_PREDICTION_CURRENT.md#최신-팀장님-풀주행-확인-2026-10-14), [노드 검토](../../../../docs/bevfusion/evidence/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md) |

LiDAR 50/51·900RPM·첫 패킷 stamp는 센서 담당 계약이다. 이 문서 정리로 센서·설치본·실행 상태를 변경하지 않았다.
