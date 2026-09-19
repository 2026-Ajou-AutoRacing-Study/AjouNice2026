# Scenarios

MORAI에서 불러올 수 있는 Internal Scenario JSON의 로컬 생성 위치다.

- `prototypes/`: 수동 검토한 최소 serialization 프로토타입. Git 추적 대상이다.
- `ros_capture_poc/`: 기본 capture runner 회귀검사용 소수 fixture. Git 추적 대상이다.
- 그 밖의 디렉터리: config와 seed로 재생성하는 로컬 산출물이다.

Production·pilot·trial 시나리오 JSON은 Git으로 추적하지 않는다. 해당 생성기와
config, asset policy, 테스트를 추적하고 capture 전에 재생성한다. 이미지·PCD·rosbag도
이 디렉터리에 저장하지 않는다.
