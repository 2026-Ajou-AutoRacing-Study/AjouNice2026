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
