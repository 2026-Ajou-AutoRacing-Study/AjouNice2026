# 인지 후단 작업 지침

이 디렉터리는 detector 이후 tracking·motion policy·prediction interface를 담당한다.
[후단 CURRENT](docs/TRACKING_PREDICTION_CURRENT.md)와 변경할 launch/topic 계약에서 시작한다.
detector까지 바꾸는 작업에서만 [BEVFusion CURRENT](../../../docs/bevfusion/CURRENT.md#runtime)를 추가한다.

Detector score와 track confidence, coasting과 planner 기억, 내부 EKF와 출력 motion policy를 구분한다.
저장된 검출 재생과 실제 ROS 입력·폐루프 검증을 서로 대체하지 않는다.
공용 launch 기본값과 운영 alias override를 모두 확인하고 production 값을 자동 변경하지 않는다.
planning/control 및 다른 agent의 MORAI 세션에는 이 지침을 확장하지 않는다.
`docs/legacy/`의 과거 PointPainting·v60 결과는 재현용이며 현재 기본값이 아니다.
현재 상태는 후단 CURRENT에서 관리하고 이 파일에 날짜별 진행 이력을 덧붙이지 않는다.
