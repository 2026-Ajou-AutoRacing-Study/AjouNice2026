# Tracking·Prediction production 현재 상태

상태: CURRENT. 문서·launch 대조: 2026-10-27.
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
| Map | `kcity_map/lanelet_map/lanelet2_map_v94.osm` 소스 기본. planning·공용 리퀘스터도 v94이며 실제 `map_file` override·설치본도 확인 |
| Horizon | **로터리/일반 최대 3초·고주로 최대 10초**, 실제 지도 종단이 먼저면 그 시간까지만 Map-CV. 매칭 자체 실패만 baseline CTRV. step 0.1s |
| Lane match | 최소 속도 0.5m/s, 거리 ≤3.0m, yaw 오차 ≤45deg |
| 경로 | start lanelet 최대 3개, 최종 path 최대 6개, centerline lateral convergence 1.5s |
| 중복 hypothesis | 같은 downstream lanelet·전체 horizon 0.75m 이내 경로를 하나로 묶음 |
| Confidence | heuristic route score 정규화. calibrated probability가 아님 |
| Fallback | map 경로 없는 차량은 signed 속도 CTRV, 비차량은 Cartesian CV |

현재 위치·yaw로 start lanelet을 순위화하고 routing graph successor를 열거한다. CTRV 곡률로 lane rank를 고르지 않는다.
Centerline Map-CV와 road-Z 수렴을 유지하며 gravity-center 높이 offset의 미래 보존을 인증하지 않는다.
geometry cache·arc index·독립 메시지 복사·동일 순서 DFS는 출력 동등성을 검증한 계산 최적화다.
[오프라인 CPU 근거](../../../../docs/bevfusion/evidence/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#cpu)를 따른다.
현재 통합 설정의 LC는 최고 confidence 정확한 동점 전체, Cruise inside/outside는 1개, 로터리는 ≥0.05 경로를 검사한다.
실제 consumed object/path는 별도 계측해야 하며 RViz Top-K 노출로 추정하지 않는다.

## 승격 범위와 기본 OFF 후보

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
