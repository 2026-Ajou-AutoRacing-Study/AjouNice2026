# BEVFusion 작업별 문서 인덱스

상태: ROUTER. 정리 기준: 2026-10-24.
현재 결정은 [CURRENT](CURRENT.md)만 따른다. 아래에서 요청에 맞는 한 행을 선택한다.
`legacy/`와 `evidence/`는 기본 검색에서 제외하며 필요한 문서·anchor만 명시적으로 읽는다.

## 실행·입력·학습

| 작업 | 읽을 문서 |
|---|---|
| Production E12·TRT/eager fallback·E24/E25 명시적 복귀 | [CURRENT runtime](CURRENT.md#runtime) → [production handoff](BASE5_BEVFUSION_PRODUCTION_HANDOFF.md#competition-startup), [E25 복귀](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md) |
| ROS 입력·시험 namespace·진단 | [ROS README](../../base_ws/src/perception/bevfusion_ros/README.md) |
| Tracker·stationary policy·Map-CV·OFF 후보 | [후단 CURRENT](../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md) |
| 학습·재개·백업 명령과 동결 run | [원격 실행 인덱스](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md) |
| 수집·native GT·변환·데이터 resume | [데이터 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md) |
| 운영 localization과 독립 평가 GT 좌표 | [GT 좌표 계약](GT_EVALUATION_COORDINATE_CONTRACT.md) |
| 새 bag 도착 후 분석·기록 항목 | [활성 우선순위](../ACTIVE_PRIORITIES.md#perception-latency), [공유 검토](evidence/261014_VALIDATOR_ANALYSIS_SHARE_HANDOFF.md) |
| 원거리·후단 연구 순서 | [개선 계획](evidence/261013_BASE5_RANGE_AND_TRACKING_IMPROVEMENT_PLAN.md) |

<a id="evaluation"></a>

## 근거 선택

동일 GPU·표본·입력·시계를 확인한다. 큰 근거 파일 전체를 현재 계약으로 읽지 않는다.

| 주제 | 필요한 근거 |
|---|---|
| 팀 v99 LAN 지연·실제 제동·중복 장애물과 후보 부작용 | [2026-10-28 검토](evidence/261028_TEAM_LAN_AND_DUPLICATE_TRACK_REVIEW.md) |
| v99 실제 주행의 최적화 60m/90m 비교·측위 시간축 교란·승격 보류 | [폐루프 4회 대조](evidence/261028_V99_WINDOW60_90_LIVE_REVIEW.md) |
| E12 완료·저장·백업 미완료 주의 | [56절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-e12-completion-261023) |
| E12/E24 동일 bag·80~100m·0점·1점 이상 보조 평가 | [57절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-e12-bags-far-front-261023), [58절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-zero-lidar-threshold-research-261023), [59절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#full30750-positive-lidar-evaluation-261023) |
| E12 TRT·head AP·연속 tracker·production 기동 | [60절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-partial-trt-cuda-experiment-261023), [61절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#continuous-trt-competition-startup-261023), [62절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#head-native-ap-runtime-261023), [63절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-production-ros1-deployment-261023), [배포 검토](evidence/261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md#e12-orin-scope-safe-startup-261023) |
| E12 tracking·coasting·3/5/6초·CV/CTRV | [64절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-tracking-map-horizon-261023), [65절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#e12-map-motion-ablation-261023) |
| 실제 노드 비용·raw packet 시계·gravity center·진단 | [오프라인 검토](evidence/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md), [Live 검토](evidence/261014_LIVE_PERCEPTION_CANDIDATE_REVIEW.md) |
| 보행자 Gaussian 순위의 운영 적용·복귀 | [연결 후속](evidence/261014_PEDESTRIAN_ASSOCIATION_AND_LIDAR_MOTION_FOLLOWUP.md) |
| 차량 초기 방향·재취득 후보·입력 경합 수정 | [차량 검증](evidence/261014_VEHICLE_DIRECTION_REACQUISITION_REVIEW.md), [독립 검증](evidence/261014_INDEPENDENT_VEHICLE_AND_LIDAR_50_51_REVIEW.md) |
| 저점수 EKF 원형·path 소비·lane-history 후보 | [48절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-low-score-core-probe-261013), [49절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-map-path-consumers-261014), [50절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-map-cv-candidates-261014) |
| E24/E9 threshold·구 Base3 교차평가·guard | [40절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-local-threshold-calibration-261012), [43절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-e9-e24-selection-261013), [44절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-on-base3-val-261013), [46절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-tracker-planning-policy-review-261013) |
| JPEG·강도·TCP·ROS 정합·추론 compile 예외 | [35절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-ros-input-continuity-261012), [39절](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-compile-actual-ros-tracker-261012) |
| 초기화·Animal·upright·heatmap·grid·학습 compile | [나머지 근거 anchor 인덱스](evidence/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md) |
| 512/576 crop·원거리 픽셀·입력 전수 검사 | [crop 진단](evidence/261016_BASE5_512_576_CROP_HEIGHT_DIAGNOSTIC.md), [본수집 전 검토](../../dataset/morai_od/docs/legacy/base5/261016_base5_precollection_final_review.md) |

## 과거 재현

[Legacy 인덱스](legacy/README.md)에 Base3/Base4 연구와 완료·기각 실험을 격리했다.
[정리 전 문서](legacy/context_261024/README.md)와 [2026-10-12 원문](../perception/legacy/context_261012/README.md)은 이력이다.
Base3 모델 config·공유 plugin·E25 handoff는 현재 Base5 의존성/명시적 복귀에 필요하므로 유지한다.
Base4 P7 config와 재귀 `_base_` 조상은 기존 manual-evaluation 기본 실행의 호환 의존성이므로 원래 config 경로에 유지한다.
PointPainting·FastBEV·Base4 temporal/velocity는 현재 production으로 자동 선택하지 않는다.
