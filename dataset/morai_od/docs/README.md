# MORAI 데이터셋 작업별 문서 인덱스

문서 상태: ROUTER. 정리 기준: 2026-10-17.
현재 Base5 작업은 [Base5 CURRENT](BASE5_V2_CURRENT.md)에서 시작한다.
아래 모든 문서를 차례로 읽지 않고 현재 요청에 필요한 행만 선택한다.

## Base5

| 작업 | 읽을 문서 |
|---|---|
| 승인 데이터·현재 release·다음 수집 상태 | [Base5 CURRENT](BASE5_V2_CURRENT.md) |
| 현재 판정: 확대 수집 중단·다음 원형 구성·남은 최종 검증 | [10/17 통합 검증](261017_base5_scale_split_qualification.md) |
| 배치 치수와 native GT의 과거 불일치·원형 크기 검사 | [크기 참조 전수 감사](261017_base5_geometry_reference_audit.md) |
| 같은 RGB·마스크에서 native 치수만 바뀐 혼합 반례 | [반복 원문 대조·cold/hot 계약](261017_base5_native_extent_repeat_mask_evidence.md) |
| 지도 전역 train coverage·실제 배치 그룹·파생 분할 | [known-map 분할 근거](261017_base5_known_map_layout_split_evidence.md) |
| 과거 10/16 계획: 정적 미션 범위·자산 scale 검증·생성기 통합 | [미션 범위와 크기 다양화 계획](261016_base5_mission_extent_and_asset_scale_plan.md). 확대는 10/17 중단 |
| 과거 정적 구역 확장·native 8모서리 크기·소량 캡처 결과 | [구역과 크기 변형 검증](261016_base5_native_scale_and_extent_evidence.md). 당시 승인과 현재 운영을 구분 |
| 과거 v154 당시 scale 1의 최종 검증·실행·재개 | [당시 본수집 전 검토](261016_base5_precollection_final_review.md). 최신 source 승인이 아님 |
| 기존 음영 이동 지면의 명시적 재사용·source pin·한계 | [지면 재사용 계약](261016_base5_shadow_ground_reuse.md) |
| 80~100m 차량·camera-only 근거·보조 차량 160개와 소량 검증 | [v153 원거리 보강](261016_base5_far_vehicle_reinforcement.md) |
| v151 전방 보강·원거리 차량·복합 성분별 소량 진단 | [10/15 실측·보존 계약·승격 전 남은 조건](261015_base5_front_far_compound_experiments.md) |
| v152 당시 작은 물리 primary 거리·3D 완전 가림 보충 | [당시 53장 전수·재개·release 근거](261015_base5_primary_obstacle_range.md) |
| v151 실패분 새 배치·부분/완료 재개·과거 차단 근거 | [당시 유한 보충과 release 검증](261015_base5_front_recovery_release.md) |
| 다음 25K 장애물 보강·MR 배치 실험·정책 수정 | [장애물 생성기 실험 계획](261013_base5_obstacle_generator_experiment_plan.md) |
| 과거 장애물 보강 구현·native 대조·실행 이력 | [실험 근거](261013_base5_obstacle_support_experiments.md), [v150 당시 런북](261013_base5_obstacle_support_runbook.md) |
| 긴 정지 NPC 보완·Sunny13 당시 검증 | [도로 지지 검증](261013_base5_parked_npc_support_evidence.md), [당시 수집 런북](261013_base5_obstacle_support_runbook.md) |
| 구 v142 환경 수집·resume·보충·대시보드 이력 | [새 환경 baseline 런북](261010_base5_fresh_environment_runbook.md). 최신 source는 CURRENT에서 확인 |
| 새 공간 배치·수집기 가속·실제 검증 | [종료 후 수집기 근거](261010_base5_postcollection_collector_work.md) |
| Sunny13 완료 ledger·유연 대체의 계보 | [v140 최종 근거](261010_base5_flexible_deficit_replacement.md) |
| 자산 eligibility·공식 제외·원래 의미 | [MR 자산 계약](MR_ASSET_COLLECTION_CURRENT.md) |
| JPEG·GT·가시성·Animal 학습 매핑 | [현재 GT 계약](BASE5_V2_CURRENT.md#gt), [모델 CURRENT](../../../docs/bevfusion/CURRENT.md#model) |
| 수동 높이와 두 waypoint 이동 | [사거리 근거](261008_base5_intersection_ground_evidence.md) |
| 로터리·음영·정지 yaw 배치 근거 | [로터리](261008_base5_roundabout_route_mission_evidence.md), [음영](261008_base5_shadow_approach_evidence.md), [yaw](261009_base5_final_appearance_v132.md) |

초기 10K·v105/v132·보충 중간 상태는 현재 시작 명령이 아니다.
원래 성공 raw·source 계약·selected ledger를 새 수집 release에 강제로 이어 붙이지 않는다.
실시간 collector와 simulator 상태는 실행 journal·프로세스에서 확인한다.

<a id="reproduction"></a>

## 명시적인 재현 작업

| 요청 범위 | 계약 |
|---|---|
| Base3-Online rosbag specialization | [보존된 Base3 기준](BASE3_ONLINE_CURRENT_STATE.md), [해당 수집 런북](base3_online_rosbag_collection_runbook.md) |
| Base4 temporal/velocity rosbag pilot | [Base4 capture 계약](BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md) |
| MR/MRP hybrid rosbag 자동화 | [hybrid 런북](morai_mr_mrp_hybrid_scenario_capture_runbook.md) |
| Base3-Static·이전 10K/소량 연구 | [legacy 인덱스](legacy/README.md), [개편 전 context](../../../docs/perception/legacy/context_261012/README.md) |

Base3-Online의 Motion V2/combined V3가 Base5 학습 기본 dataset이라는 뜻은 아니다.
MR 수집에 MRP quarantine을 적용하지 않는다. 재현은 당시 source·catalog·단위를 명시한다.
