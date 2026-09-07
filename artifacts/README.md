# Local artifacts

이 디렉터리는 학습, 평가, 데이터 패키징, RViz 검증 과정에서 생성되는 로컬
산출물을 저장한다. 용량이 크거나 실행마다 달라지는 파일이므로 이 README를
제외한 모든 내용은 Git에서 추적하지 않는다.

주요 하위 경로는 다음과 같다.

- `detection3d/`: Fast-BEV 학습 데이터, pretrained weight, checkpoint, 로그,
  진단 결과
- `prediction_evaluation/`: prediction 평가의 `summary.json`,
  `worst_samples.jsonl` 등 실행 결과
- `rviz_bag_checks/`: rosbag과 RViz 연동 검증 결과
- `mvp_validation/`: 초기 MVP 검증 결과

재현에 필요한 코드, 설정, 실행 명령과 결과 해석은 각 ROS/Perception 패키지의
README 또는 `docs/`에 기록한다. 공유가 필요한 checkpoint와 데이터셋은 Git에
추가하지 않고 별도 스토리지에 보관하며, 필요하면 파일 경로와 checksum만
문서에 남긴다.
