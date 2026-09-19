# morai_fastbev_ros

> **상태: LEGACY/RESEARCH.** 패키지는 재현용으로 보존하지만 현재 production detector는
> BEVFusion이다. 현재 실행법은 `docs/bevfusion/README.md`를 따른다.

MORAI `Sensor_6.json`의 네 압축 카메라 영상을 동기화하여 Fast-BEV를 온라인
추론하고, 기존 Tracking & Prediction 파이프라인에 연결하는 ROS1 Noetic
패키지다.

## 인터페이스

입력:

- `/sensor/camera/front/compressed`
- `/sensor/camera/front_left/compressed`
- `/sensor/camera/front_right/compressed`
- `/sensor/camera/back/compressed`

출력:

- `/perception/object_recognition/detection/objects`: `autoware_perception_msgs/DetectedObjects`
- `/fastbev/diagnostics`: `diagnostic_msgs/DiagnosticArray`
- `/hmi/perc/jsk_pillar_objects`: `jsk_recognition_msgs/BoundingBoxArray`
- `/fastbev/detection_markers`: `visualization_msgs/MarkerArray`

첫 출력은 detector-neutral tracker의 표준 입력이다. diagnostics는 Fast-BEV의 처리 시간과
drop 상태 전용이고, JSK/Marker는 RViz 전용이다. 이 package는 PointPainting custom
message에 의존하지 않는다.

## 실행

최초 한 번 `fastbev_rtx50`에 ROS의 순수 Python 의존성을 설치한다. 시스템
`dist-packages` 전체를 `PYTHONPATH`에 넣으면 Ubuntu Python 3.8용 NumPy와
Conda Python 3.11이 충돌하므로 사용하지 않는다.

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate fastbev_rtx50
python -m pip install rospkg catkin-pkg
```

```bash
cd ~/AjouNice2026

# Fast-BEV 검출과 RViz marker만 실행
base_ws/src/morai_fastbev_ros/scripts/run_fastbev.sh live

# Fast-BEV -> EKF tracking -> CTRV prediction -> Autoware 판단 입력
FASTBEV_WITH_TP=true \
base_ws/src/morai_fastbev_ros/scripts/run_fastbev.sh live
```

rosbag에서는 `live` 대신 `bag`을 사용하고 `rosbag play --clock`으로 재생한다.

기본 runtime은 test 222프레임에서 기존 PIL과 사실상 같은 정확도를 확인한
OpenCV 최적화 설정이다.

- `preprocess_backend=opencv`(기본): 고정 4카메라 resize/crop을 OpenCV로
  병렬 처리한다. PIL 대비 center mAP 변화는 -0.00022였고 CPU 변환은 약
  1.6배 빨랐다.
- `preprocess_backend=fixed`: 기존 학습/평가와 픽셀 결과가 완전히 같은 PIL
  resize를 사용하는 재현·rollback 경로다.
- `pipeline_mode=staged`: 네 카메라 CPU 전처리와 GPU 추론을 서로 겹쳐 수행한다.
- 카메라 네 장의 JPEG decode와 resize/normalize도 4 worker로 병렬 수행한다.
- 대기열은 각 단계마다 최신 세트 하나만 유지하므로 처리 지연이 누적되지 않는다.

기존 PIL 경로를 명시적으로 선택하려면 다음처럼 실행한다.

```bash
FASTBEV_PREPROCESS_BACKEND=fixed \
base_ws/src/morai_fastbev_ros/scripts/run_fastbev.sh bag
```

체크포인트를 바꾸려면 다음 환경변수를 사용한다.

```bash
export FASTBEV_CHECKPOINT=/absolute/path/to/checkpoint.pth
```

세부 계약과 검증 순서는 [docs/legacy/ONLINE_INTEGRATION.md](docs/legacy/ONLINE_INTEGRATION.md)에
정리한다.

runtime 구현·동등성·23/30/35 Hz 결과는
[docs/legacy/RUNTIME_OPTIMIZATION.md](docs/legacy/RUNTIME_OPTIMIZATION.md)에 정리한다.

MORAI Sensor/Ego 연결부터 Fast-BEV, T&P, RViz와 topic 모니터링까지의 전체
터미널 순서는
[`docs/runbooks/legacy/morai_fastbev_object_pipeline_live.md`](../../../docs/runbooks/legacy/morai_fastbev_object_pipeline_live.md)를
따른다.
