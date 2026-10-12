# BEVFusion 작업별 문서 인덱스

문서 상태: ROUTER. 정리 기준: 2026-10-12.
현재 모델·실행 선택·남은 과제는 [CURRENT](CURRENT.md)에서 한 번만 관리한다.
이 문서는 필요한 근거를 고르는 안내이며 날짜별 진행 결과를 복사하지 않는다.

## 실행과 입력

| 작업 | 읽을 문서 |
|---|---|
| 현재 모델·checkpoint·옵션 선택 | [CURRENT](CURRENT.md) |
| Production E25 배포 | [production handoff](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md) |
| Base5 ROS 시험 | [패키지 README](../../base_ws/src/perception/bevfusion_ros/README.md) |
| 원격 학습·재개·백업 | [원격 실행 안내](261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md) |
| 생성·수집·GT 변환 | [Base5 데이터 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md) |
| Tracker·motion policy·prediction | [후단 CURRENT](../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md) |

<a id="evaluation"></a>

## 근거 선택

다음 링크는 큰 EVIDENCE 파일의 **관련 절만** 선택한다. 서로 다른 GPU·표본·단계를 한 결과로 합치지 않는다.

| 주제 | 근거 |
|---|---|
| 안정적인 이전 완주 E9/E25·RViz | [42절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-stable-whole-route-view-261012) |
| 100m bag GT·bounded coasting·중복/잔류 | [41절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-far-bag-coasting-261012) |
| E24 local threshold·실제 ROS 후보 비교 | [40절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-local-threshold-calibration-261012) |
| Compile의 전체 객체 분포·실제 ROS 영향 | [38절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-compile-population-distributions-261012), [39절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-compile-actual-ros-tracker-261012) |
| 학습된 4090 출력·비용·메모리 | [36절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-learned-runtime-vast-261012), [37절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-compile-delivery-tracking-review-261012) |
| ROS JPEG·강도·TCP 정합·연속 검출 | [35절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-ros-input-continuity-261012) |
| 완료 E9/E24·E25 공통 30m 비교 | [33절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-common30-e25-comparison-261012), [34절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-rosbag-quick-common30-261012) |
| 5K 가시성·crop·원거리 정보량 | [32절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-range-evidence-local-261012) |
| 학습 초기화·Animal 통합·학습 복구 | [28절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-training-health-repair-261011), [29절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-animal-pedestrian-merge-261011), [30절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-full-initializer-trial-261011) |
| W&B 거리별 AP·구간 mAP | [31절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-far-distance-resume-261012) |
| 실제 학습 compile OFF/ON·메모리 경계 | [25절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-fine-training-compile-vast-261011), [26절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-memory-batch-decision-261011) |
| JPEG 전환·정답 heatmap 축·upright BBox | [12절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#all-jpeg-policy-261011), [15절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-heatmap-target-alignment-261011), [16절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-upright-native-rpy-261011) |
| 격자·depth·Z 설정 비교 | [17절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-grid-depth-local-261011), [19절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-vast-depth-z-qualification-261011) |
| CUDA-BEVFusion·BEVPool·TensorRT 검토 | [배포 검토](261009_CUDA_BEVFUSION_AND_BEVPOOL_DEPLOYMENT_REVIEW.md) |
| 차량 velocity 기준과 bag GT | [GT 기준 조사](261005_VEHICLE_VELOCITY_GT_REFERENCE_REVIEW.md) |

## 과거 재현

[기존 연구 문서](legacy/README.md)와 [개편 전 context 원문](../perception/legacy/context_261012/README.md)은 재현용이다.
Base4 temporal/velocity, PointPainting, Fast-BEV는 현재 Base5 기본 입력·모델로 자동 선택하지 않는다.
Production E25 handoff는 이름에 Base3가 있어도 현재 운영 계약이므로 이력으로 폐기하지 않는다.
