# morai_fastbev_ros

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
- `/fastbev/detections`: `morai_pointpainting_ros/PointPaintingDetectionArray`
- `/hmi/perc/jsk_pillar_objects`: `jsk_recognition_msgs/BoundingBoxArray`
- `/fastbev/detection_markers`: `visualization_msgs/MarkerArray`

첫 출력이 production tracker의 정식 입력이다. 나머지는 성능 진단과 RViz 및
과거 실험 재현용 legacy/debug 출력이며, track ID나 kinematics 전달에 사용하지
않는다.

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

체크포인트를 바꾸려면 다음 환경변수를 사용한다.

```bash
export FASTBEV_CHECKPOINT=/absolute/path/to/checkpoint.pth
```

세부 계약과 검증 순서는 [docs/ONLINE_INTEGRATION.md](docs/ONLINE_INTEGRATION.md)에
정리한다.
