# Dataset schemas

Scenario manifest, capture manifest와 검증 보고서의 JSON Schema를 관리한다.

Fast-BEV baseline v1의 실제 레코드는
`scripts/build_fastbev_sunny13_1500_dataset.py`가 다음 형태로 생성한다.

- `metadata/scenes.jsonl`: scene ID, split, scene type, topology, 기대 객체 수
- `metadata/frames.jsonl`: frame ID, 공통 MORAI timestamp, 4-camera/LiDAR 경로
- `annotations/boxes_3d.jsonl`: LiDAR 좌표 3D box와 camera visibility
- `metadata/calibrations/sensor6_v1.json`: intrinsic 및 sensor/ego/LiDAR transform
- `fastbev_infos/*.pkl`: Fast-BEV의 NuScenes-info 호환 입력

형식이 안정화되기 전까지 Python 생성기와 dependency-light smoke test를 실행 가능한
기준으로 삼고, StreamPETR temporal metadata v2와 함께 정식 JSON Schema를 추가한다.

# Dynamic clip metadata

동적 객체 수집본은 MORAI `/SaveSensorData` 센서 파일과 다음 경량 파일을 둔다.

- `dynamic_metadata/<scene>.frames.jsonl`: 저장 frame, 센서 경로, Ego·actor GT.
- `dynamic_metadata/<scene>.clip.json`: clip split, 환경, 주기, calibration 참조.

actor velocity는 MORAI 원시 message 값을 단위 변환 없이 보존하고, map position
중앙차분으로 계산한 `derived_velocity_mps`를 함께 둔다. actor acceleration,
yaw rate와 motion phase도 기록한다. 같은 `clip_id`의 모든 frame은 반드시 같은
split에 속해야 한다. 자세한 생성·검증 절차는
과거 temporal capture 계약은 `../docs/legacy/dynamic_object_capture_v1.md`에 보존한다.
