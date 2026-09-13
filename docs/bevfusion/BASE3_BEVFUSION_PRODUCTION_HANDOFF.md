# Base3 BEVFusion production handoff

이 문서는 판단·플래닝 PC에서 `B3-BF0 E10`을 동일 조건으로 실행하기 위한
운영 계약이다. 과거 `260720_*zero_shot*` 문서는 MORAI 학습 전 nuScenes
checkpoint 실험 기록이며 production 실행법이 아니다.

## 고정된 production 구성

| 항목 | 값 |
|---|---|
| 모델 | `B3-BF0 E10`, Base3 4-camera + LiDAR BEVFusion |
| 모델 입력 | front/front-left/front-right/back JPEG + LiDAR XYZI |
| 출력 | `/perception/object_recognition/detection/objects` |
| confidence | Vehicle `0.40`, Pedestrian `0.15`, Obstacle `0.10` |
| 후단 | EKF tracking → UNKNOWN stationary policy → Map-CV prediction |
| Docker base | PyTorch 2.7.1, CUDA 12.8, cuDNN 9 |
| GPU | RTX 4090의 compute capability 8.9가 이미지 빌드 대상에 포함됨 |

학습 checkpoint와 별도로 전달되는 배포 파일 이름은
`base3_bf0_e10_inference.pth`이다. 파일을 아래 위치에 둔다.

```bash
mkdir -p "$HOME/bevfusion_artifacts/checkpoints"
cp /전달받은/경로/base3_bf0_e10_inference.pth \
  "$HOME/bevfusion_artifacts/checkpoints/"
sha256sum "$HOME/bevfusion_artifacts/checkpoints/base3_bf0_e10_inference.pth"
```

기대 SHA-256은 이 파일을 생성·검증한 뒤 본 문서의 `Release receipt`에
기록한다. 해시가 다르면 실행하지 않는다.

## 최초 1회 설치

호스트에는 NVIDIA driver, Docker Engine, Docker Compose v2 plugin,
NVIDIA Container Toolkit이 필요하다. CUDA toolkit이나 Python Conda 환경을
호스트에 별도로 맞출 필요는 없다. GPU 의존성은 Docker 이미지가 소유하고,
ROS1 bridge만 호스트 workspace가 소유한다.

```bash
cd "$HOME/AjouNice2026"
git submodule sync --recursive
git submodule update --init --recursive
git submodule status third_party/mmdetection3d

docker build \
  -f docker/bevfusion/Dockerfile \
  -t ajounice/bevfusion:dev-cu128 \
  .
```

MMDetection3D submodule 기준은 `19422fb451504c6c8ff8dbab0fd48f3e4d73a113`이다.
이미지를 다른 PC에서 복사하지 않고 각 GPU PC에서 위 Dockerfile로 빌드한다.

branch pull 후 BEVFusion bridge와 공용 `planviz` 변경이 설치 공간에 반영돼야
한다. 기존 AW149 overlay가 있는 팀 PC에서는 아래 관련 package만 증분 빌드하면
된다. 전체 planning workspace를 다시 빌드할 필요는 없다.

```bash
cd "$HOME/AjouNice2026"
source /opt/ros/noetic/setup.bash
test -f "$PWD/base_ws/install_aw149/setup.bash"
source "$PWD/base_ws/install_aw149/setup.bash"

catkin_make_isolated --install \
  --source "$PWD/base_ws/src" \
  --build "$PWD/base_ws/build_aw149" \
  --devel "$PWD/base_ws/devel_aw149" \
  --install-space "$PWD/base_ws/install_aw149" \
  --pkg bevfusion_ros tracking_prediction_evaluation \
        lanelet_map_visualizer bringup \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --make-args -j4 -l4
```

AW149 overlay 자체가 없는 새 checkout만 repository 표준 초기 빌드를 수행한다.

```bash
cd "$HOME/AjouNice2026"
scripts/workspace/setup_ws.sh
```

마지막으로 다음 setup이 존재하는지 확인한다.

```bash
test -f "$HOME/AjouNice2026/base_ws/install_aw149/setup.bash"
```

## 실행 순서

### 1. ROS와 MORAI sensor bridge

기존 planning 운영 절차대로 roscore 및 MORAI sensor/ego UDP bridge를 실행한다.
다음 입력이 실제 메시지를 발행해야 한다.

```bash
rostopic hz /sensor/lidar1
rostopic hz /sensor/camera/front/compressed
rostopic hz /sensor/camera/front_left/compressed
rostopic hz /sensor/camera/front_right/compressed
rostopic hz /sensor/camera/back/compressed
```

### 2. Docker inference server

로컬 `.env`를 요구하지 않는 실행법이다. repository나 weight 위치가 달라도
두 환경변수만 바꾸면 된다.

```bash
cd "$HOME/AjouNice2026"
AJOUNICE_ROOT="$PWD" \
BEVFUSION_CHECKPOINT="$HOME/bevfusion_artifacts/checkpoints/base3_bf0_e10_inference.pth" \
docker compose -f docker/bevfusion/compose.yaml up -d inference

docker compose -f docker/bevfusion/compose.yaml logs -f inference
```

다른 터미널에서 readiness를 확인한다.

```bash
cd "$HOME/AjouNice2026/docker/bevfusion"
PYTHONPATH=. python3 scripts/healthcheck.py
```

### 3. Host ROS bridge

```bash
export AJOUNICE_ROOT="$HOME/AjouNice2026"
source /opt/ros/noetic/setup.bash
source "$AJOUNICE_ROOT/base_ws/install_aw149/setup.bash"

roslaunch bevfusion_ros base3_online.launch start_rviz:=false
```

launch 기본값 자체가 `0.40/0.15/0.10`을 적용한다. Docker에는 가장 낮은
`0.10`까지 요청하고, ROS bridge가 class별 operating point를 적용한 뒤 하나의
canonical detection topic만 발행한다.

### 4. Planning stack

BEVFusion이 외부 detector이므로 planning에서 detector를 중복 실행하지 않는다.

```bash
export AJOUNICE_ROOT="$HOME/AjouNice2026"
source /opt/ros/noetic/setup.bash
source "$AJOUNICE_ROOT/base_ws/install_aw149/setup.bash"

roslaunch ajounice_planning_integration morai_planning_stack.launch \
  start_detector:=false
```

`start_detector:=false`는 detector를 끄겠다는 뜻이 아니라 planning launch가
FastBEV/PointPainting을 하나 더 띄우지 않게 한다는 뜻이다. BEVFusion bridge가
동일한 canonical topic을 공급한다.

### 5. 공용 planning RViz (`planviz`)

repository 공용 alias를 불러온 뒤 `planviz`를 실행한다.

```bash
cd "$HOME/AjouNice2026"
source docs/common_alias.txt
planviz
```

기본 화면의 `Perception Pipeline` group에는 다음 항목이 포함된다.

- MORAI LiDAR point cloud
- tracker 결과 BBox와 class/`UNKNOWN` label
- Map-CV predicted path의 1/2/3초 horizon
- MORAI GT BBox(분홍색, simulator debug 전용)
- 기존 Lanelet map, planning route/trajectory, Ego vehicle

raw detector BBox는 tracker BBox와 겹쳐 화면이 복잡해지는 것을 막기 위해 기본
OFF이며 RViz Displays에서 필요할 때 켠다. 분홍색 GT는 `/Object_topic`을 Marker로
바꾸는 시각화 reference일 뿐 perception, tracking, prediction, planning 입력에는
절대 사용되지 않는다.

대회 현장처럼 GT를 사용할 수 없거나 숨겨야 할 때는 alias 대신 다음과 같이
실행한다.

```bash
source "$HOME/AjouNice2026/scripts/workspace/source_aw149.sh"
roslaunch bringup aw149_planning_rviz.launch show_morai_gt:=false
```

`planviz`는 detector를 실행하지 않는다. 앞 단계의 BEVFusion bridge와 planning
stack이 실행 중이어야 detector → tracker → Map-CV 결과가 차례로 표시된다.

## 운영 확인

```bash
rostopic info /perception/object_recognition/detection/objects
rostopic hz /perception/object_recognition/detection/objects
rostopic echo -n 1 /perception/bevfusion/status
rostopic echo -n 1 /perception/bevfusion/latency_ms
rosnode info /bevfusion_ros_bridge
```

정상 상태는 detector publisher가 정확히 하나이고, status가 `ok`이며,
`dropped_raw`와 `dropped_prepared`가 지속적으로 증가하지 않는 상태다. RTX 5060
Ti에서 E10은 약 12.5–12.8 Hz 처리 용량과 약 78–84 ms p50/p95 end-to-end
latency를 보였다. RTX 4090 수치는 현장에서 다시 측정한다.

종료는 다음과 같다.

```bash
cd "$HOME/AjouNice2026"
docker compose -f docker/bevfusion/compose.yaml down
```

## 문제 해결 경계

- `.env`는 Git ignore 대상이며 다른 PC로 전달되지 않는다. 문서의 inline
  environment 방식이 기준이다. `.env.example`은 선택적 로컬 편의용이다.
- nuScenes 이름의 checkpoint는 초기화/zero-shot 실험용이며 production이 아니다.
- `/perception/object_recognition/detection/objects` publisher가 둘 이상이면
  PointPainting/FastBEV/별도 playback 중 하나를 종료한다.
- healthcheck가 실패하면 먼저 Docker log와 port `29500`을 확인한다.
- sensor topic이 없으면 모델 문제가 아니라 MORAI UDP bridge 입력 문제다.
- ROS bridge는 latest-only queue를 사용한다. GPU가 느리면 지연을 누적하지 않고
  frame을 drop하며 그 수가 status에 표시된다.
- `planviz`에서 지도만 보이면 canonical detection topic publisher, tracker,
  prediction topic을 차례로 확인한다. RViz 자체가 perception node를 대신 실행하지
  않는다.

## Release receipt

아래 값은 배포 checkpoint 생성 및 Docker load smoke 후 확정한다.

| 항목 | 값 |
|---|---|
| Git branch | `perception/bevfusion-base3-domain-generalization` |
| Git commit | 이 문서를 포함한 전달 commit (`git rev-parse HEAD`로 확인) |
| checkpoint | `base3_bf0_e10_inference.pth` |
| checkpoint SHA-256 | `b2d3f65befaa16382cdd189f9c8ac1aff2d2176e6c48e19a17ab329947eff419` |
| checkpoint size | `163,667,267 bytes` (약 156 MiB) |
| Docker image | `ajounice/bevfusion:dev-cu128` |
| MMDetection3D | `19422fb451504c6c8ff8dbab0fd48f3e4d73a113` |
