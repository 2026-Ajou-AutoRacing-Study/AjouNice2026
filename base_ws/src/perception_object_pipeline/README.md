# Perception object pipeline

Detector 이후 EKF tracking·UNKNOWN stationary policy·Lanelet2 Map-CV prediction의 ROS1 경계를 관리한다.
현재값은 [Tracking CURRENT](docs/TRACKING_PREDICTION_CURRENT.md)만 따른다.
Detector는 [BEVFusion E12 production](../../../docs/bevfusion/CURRENT.md#runtime)이며 E24/E25는 명시적 복귀다.

## 운영 경로

```text
BEVFusion DetectedObjects
  → EKF measurement_objects
  → UNKNOWN stationary tracking/objects
  → Map-CV PredictedObjects (/perception/object_recognition/objects)
```

운영 진입점은 `docs/common_alias.txt`의 `tracking`이다. detector·localization·planning은 별도다.
실제 alias override와 launch 기본을 모두 확인하며 같은 canonical topic에 중복 publisher를 시작하지 않는다.
Map-CV는 **로터리/일반 3초·지도 경로가 충분한 고주로 8초**다. 부족하면 기존 Map-CV/CTRV 3초다.
시간은 `config/map_based_prediction.yaml` 한 곳에서 바꾼다. [설정·적용 안내](docs/HIGHWAY_EIGHT_SECOND_HANDOFF.md)를 따른다.

| 작업 | 진입점 |
|---|---|
| 운영 topic·coasting·age·motion policy·map | [CURRENT](docs/TRACKING_PREDICTION_CURRENT.md) |
| Detector 없이 후단 실행 | `launch/tracking_prediction.launch` (`start_detector=false`) |
| 후단 구성·실제 인자 | `launch/object_pipeline.launch`, `launch/object_tracking.launch`, `launch/object_prediction.launch` |
| 관측 일관성 후보 | [오프라인 검토](docs/OBSERVATION_CONSISTENCY_REVIEW.md), [ROS 후속](docs/OBSERVATION_CONSISTENCY_ROS_VALIDATION.md); shape/class 후보 기본 OFF |
| 고주로8초 운영·복귀 | [배포 안내](docs/HIGHWAY_EIGHT_SECOND_HANDOFF.md), [기존 mixed/native 비교](docs/MIXED_PREDICTION_HORIZON.md) |
| LAN·단계별 진단 | [기록 절차](../../../scripts/README.md#lan-두-pc를-함께-계측하기) |
| 통제 실험 GT 대체 | `tracking_prediction.launch use_morai_gt_detection:=true`; `/Object_topic` 없는 대회에서는 fallback 아님 |

Nodes는 private `~input/objects`·`~output/objects`를 사용하고 launch가 global topic을 remap한다.
기존 FastBEV/PointPainting generic detector 선택은 호환/과거 재현용이며 production 실행 안내가 아니다.
이전 README의 E25 production·실행 예·내부 구현 상세는 [보존 원문](docs/legacy/context_261024/README.md)에 있다.
현재 gravity-center·stamp·class·shape·UUID·path 순서/confidence 계약은 변경하지 않는다.
