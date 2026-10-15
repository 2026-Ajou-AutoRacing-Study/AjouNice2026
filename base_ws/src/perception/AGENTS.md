# ROS 인지 작업 지침

[인지 작업 안내](../../../docs/perception/README.md)에서 담당 노드를 먼저 선택한다.
이 지침은 `bevfusion_ros`와 `traffic_light_detection`에 적용하며 planning/control에는 확장하지 않는다.

- BEVFusion: [모델 CURRENT](../../../docs/bevfusion/CURRENT.md#runtime) → [패키지 README](bevfusion_ros/README.md) → 선택한 launch/config를 확인한다.
- 신호등 YOLO: [패키지 README](traffic_light_detection/README.md) → 현재 launch와 해당 호스트의 Python·device를 확인한다. BEVFusion 학습 문서를 먼저 읽지 않는다.
- 후단 tracker/prediction은 [별도 CURRENT](../perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)를 확인한다.

Production, 시험용 namespace, 다른 detector의 canonical publisher를 구분한다.
Base5 opt-in 시험과 사용자가 승격한 production 기본값을 구분한다. 실제 선택은 CURRENT와 실행 profile을 따른다.
실제 interpreter·GPU 사용은 launch와 실행 상태로 확인하고 Python 경로만으로 CPU/GPU를 단정하지 않는다.
원본 JPEG 전달·강도 변환·좌표계·sensor stamp·queue 정책을 변경하면 입력 동등성을 검증한다.
다른 agent의 planning_validator나 MORAI 세션을 임의로 시작·중단·변경하지 않는다.
현재 상태는 위 CURRENT에서 관리하고 이 파일에 실험 이력을 덧붙이지 않는다.
