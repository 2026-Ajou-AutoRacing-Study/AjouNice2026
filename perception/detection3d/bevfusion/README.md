# BEVFusion 구현과 작업 경로

현재 모델·학습·배포 선택은 [모델 CURRENT](../../../docs/bevfusion/CURRENT.md)에서 확인한다.
이 디렉터리는 config, MORAI plugin, 학습·추론·평가 도구를 관리한다.
파일명에 Base3가 있다는 이유로 현재 production 계약을 폐기하거나 Base5 시험을 production으로 승격하지 않는다.

| 경로 | 역할 |
|---|---|
| `configs/` | 모델·입력·학습·평가 설정. 동결 run의 resolved config와 구분 |
| `plugin/` | MORAI dataset·증강·직사각형 BEV·GT/head·연산 구현 |
| `tools/` | 변환·정합·비용·정확도·저장된 검출/ROS 검증 도구 |
| `scripts/` | 학습·배포·기존 실험 실행기 |
| `third_party/mmdetection3d` | 실제 사용 MMDetection3D 구현. 저장소 루트 기준 경로 |
| `docker/bevfusion` | GPU 실행 환경·추론 서버. 저장소 루트 기준 경로 |

현재 Base5는 [데이터 CURRENT](../../../dataset/morai_od/docs/BASE5_V2_CURRENT.md)와
[ROS 패키지](../../../base_ws/src/perception/bevfusion_ros/README.md)에서 입력/실행 계약을 연결한다.
[근거 인덱스](../../../docs/bevfusion/README.md)에서 요청에 필요한 절만 선택한다.

초기 Base3 feasibility·smoke·팀 학습 명령은
[무손실 원문](../../../docs/perception/legacy/context_261012/README.md)에 보존했다.
그 초기 README의 ‘production 대체 아님’, 12 epoch, ±30m 입력 등을 현재 Base5 기본값으로 쓰지 않는다.
