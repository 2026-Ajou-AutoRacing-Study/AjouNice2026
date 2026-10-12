# MORAI 데이터셋 작업별 문서 인덱스

문서 상태: ROUTER. 정리 기준: 2026-10-12.
현재 Base5 작업은 [Base5 CURRENT](BASE5_V2_CURRENT.md)에서 시작한다.
아래 모든 문서를 차례로 읽지 않고 현재 요청에 필요한 행만 선택한다.

## Base5

| 작업 | 읽을 문서 |
|---|---|
| 승인 데이터·현재 release·다음 수집 상태 | [Base5 CURRENT](BASE5_V2_CURRENT.md) |
| 다음 25K 장애물 보강·MR 배치 실험·정책 수정 | [장애물 생성기 실험 계획](261013_base5_obstacle_generator_experiment_plan.md) |
| 장애물 보강 구현·native 대조·남은 live 검사 | [실험 근거](261013_base5_obstacle_support_experiments.md), [현재 후보 런북](261013_base5_obstacle_support_runbook.md) |
| 구 v142 환경 수집·resume·보충·대시보드 이력 | [새 환경 baseline 런북](261010_base5_fresh_environment_runbook.md). 현재 보강은 위 후보 런북 사용 |
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
