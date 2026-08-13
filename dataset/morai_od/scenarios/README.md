# Scenarios

MORAI에서 불러올 수 있는 Internal Scenario JSON을 저장한다.

- `prototypes/`: 시뮬레이터 로드와 배치 규칙을 확인하는 소수의 프로토타입
- 향후 `train/`, `val/`, `test/`: 승인된 데이터셋 시나리오

시나리오 JSON은 재현성과 리뷰를 위해 Git으로 추적한다. 이미지·PCD·rosbag은
이 디렉터리에 저장하지 않는다.
