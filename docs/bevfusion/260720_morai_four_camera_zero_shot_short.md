# MORAI 4-camera BEVFusion zero-shot 빠른 실행

이 문서는 `Sensor.json`의 전방·전방 좌·전방 우·후방 카메라와
`lidar1`을 Docker BEVFusion에 전달하고, 결과를 RViz에서 실시간 확인하는
최소 실행 절차다. 처음에는 반드시 projection을 확인한 뒤 zero-shot 추론으로
넘어간다.

## 최초 1회 준비

```bash
# MMDetection3D 서브모듈을 포함해 저장소 의존 코드를 받는다.
cd ~/AjouNice2026
git submodule sync --recursive
git submodule update --init --recursive
```

```bash
# 이미지가 아직 없는 팀원 PC에서 검증된 CUDA 12.8 이미지를 빌드한다.
cd ~/AjouNice2026
docker build \
  -f docker/bevfusion/Dockerfile \
  -t ajounice/bevfusion:dev-cu128 \
  .
```

현재 zero-shot 확인에는 MORAI 전용 학습 모델이 아니라 MMDetection3D가 공개한
NuScenes LiDAR-camera BEVFusion checkpoint를 임시로 사용한다.

```bash
# 공식 NuScenes BEVFusion 임시 checkpoint를 팀 공통 경로에 내려받는다.
mkdir -p ~/bevfusion_artifacts/checkpoints
wget -c \
  -O ~/bevfusion_artifacts/checkpoints/bevfusion_lidar_cam_nuscenes.pth \
  https://download.openmmlab.com/mmdetection3d/v1.1.0_models/bevfusion/bevfusion_lidar-cam_voxel0075_second_secfpn_8xb4-cyclic-20e_nus-3d-5239b1af.pth
```

```bash
# 다운로드한 checkpoint가 현재 검증에 사용한 160 MB 파일과 같은지 확인한다.
echo "e68c6c084d8971f4f879751b0ed3981f575c24ee3cf385a1e39197c26a9bb3ab  ${HOME}/bevfusion_artifacts/checkpoints/bevfusion_lidar_cam_nuscenes.pth" \
  | sha256sum --check
```

정상이면 다음 문구가 출력된다.

```text
/home/<사용자>/bevfusion_artifacts/checkpoints/bevfusion_lidar_cam_nuscenes.pth: OK
```

```bash
# 새 ROS bridge를 포함해 workspace를 빌드한다.
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
catkin build bevfusion_ros bringup
source devel/setup.bash
```

```bash
# 팀원 PC의 저장소와 체크포인트 절대 경로를 Compose에 등록한다.
cd ~/AjouNice2026/docker/bevfusion
cp -n .env.example .env
gedit .env
```

`.env`에서 최소한 다음 두 경로를 실제 PC에 맞춘다.

```dotenv
AJOUNICE_ROOT=/home/luv/AjouNice2026
BEVFUSION_CHECKPOINT=/home/luv/bevfusion_artifacts/checkpoints/bevfusion_lidar_cam_nuscenes.pth
```

## 1차: calibration projection 확인

MORAI에서 `Sensor.json` 설정과 UDP 통신을 활성화한 후 다음 터미널을 각각
실행한다.

### 터미널 1: ROS master

```bash
# ROS master를 실행한다.
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roscore
```

### 터미널 2: MORAI UDP 센서 수신

```bash
# Sensor_4의 카메라·LiDAR·GPS·IMU UDP 패킷을 ROS topic으로 변환한다.
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:=/home/luv/AjouNice2026/configs/sensors/Sensor.json \
  network_config:=/home/luv/AjouNice2026/configs/network/ego_network_1.json
```

### 터미널 3: 센서 TF

```bash
# Sensor_4에서 base_link→camera/lidar static TF를 발행한다.
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roslaunch bringup sensing.launch \
  sensor_config:=/home/luv/AjouNice2026/configs/sensors/Sensor.json \
  start_rviz:=false \
  enable_vehicle_model:=false
```

### 터미널 4: projection 전용 Docker 서버

```bash
# 모델을 로드하지 않고 LiDAR→camera projection 서버만 실행한다.
cd ~/AjouNice2026/docker/bevfusion
PROJECTION_ONLY=true docker compose up inference
```

### 터미널 5: projection ROS bridge와 RViz

```bash
# 동기화된 센서를 서버로 보내고 네 카메라 projection topic을 발행한다.
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roslaunch bringup perception_bevfusion.launch \
  projection_only:=true \
  start_rviz:=true
```

RViz에서 projection 영상이 자동으로 보이지 않으면 `Add → By topic`에서 아래
네 `CompressedImage` topic을 추가한다.

```text
/perception/bevfusion/projection/camera_front/compressed
/perception/bevfusion/projection/camera_front_left/compressed
/perception/bevfusion/projection/camera_front_right/compressed
/perception/bevfusion/projection/camera_back/compressed
```

도로·차량·건물 윤곽에 LiDAR 점이 대략 맞아야 한다. 축이 반대이거나 점이
영상 밖에 몰리면 zero-shot 추론으로 넘어가지 않는다.

## 2차: 4-camera zero-shot 실시간 추론

projection용 터미널 4와 5를 `Ctrl-C`로 종료한 뒤 실행한다. 터미널 1~3은
계속 유지해도 된다.

### 터미널 4: BEVFusion 추론 서버

```bash
# 공식 NuScenes checkpoint를 로드해 localhost TCP 추론 서버를 실행한다.
cd ~/AjouNice2026/docker/bevfusion
PROJECTION_ONLY=false docker compose up inference
```

로그에 `BEVFusion model is ready`가 나온 후 다음 단계로 간다.

### 터미널 5: zero-shot ROS bridge와 RViz

```bash
# Sensor_4 입력을 추론하고 3D box·label을 RViz에 표시한다.
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roslaunch bringup perception_bevfusion.launch \
  projection_only:=false \
  start_rviz:=true
```

RViz의 Fixed Frame은 `base_link`이고 다음 display가 활성화돼 있어야 한다.

- `/sensor/lidar1`: 원본 PointCloud2
- `/perception/bevfusion/markers`: 3D box와 class·score
- TF: `base_link → lidar1` 연결 확인

## 실행 중 빠른 상태 확인

```bash
# 4개 카메라와 LiDAR 입력 주기를 확인한다.
rostopic hz /sensor/camera/front/compressed
rostopic hz /sensor/camera/front_left/compressed
rostopic hz /sensor/camera/front_right/compressed
rostopic hz /sensor/camera/back/compressed
rostopic hz /sensor/lidar1
```

```bash
# bridge 연결 상태와 end-to-end 처리 지연을 확인한다.
rostopic echo /perception/bevfusion/status
rostopic echo /perception/bevfusion/latency_ms
```

```bash
# Docker 서버가 protocol 요청에 응답하고 모델이 준비됐는지 확인한다.
cd ~/AjouNice2026/docker/bevfusion
PYTHONPATH=. python3 scripts/healthcheck.py
```

```bash
# 추론 서버의 최근 로그를 확인한다.
cd ~/AjouNice2026/docker/bevfusion
docker compose logs --tail=100 inference
```

## 종료

```bash
# BEVFusion 컨테이너를 정리한다.
cd ~/AjouNice2026/docker/bevfusion
docker compose down
```

현재 출력은 zero-shot 시각 검증용이다. NuScenes와 MORAI의 domain 차이 때문에
box가 보인다는 사실만으로 성능이 확보된 것은 아니며, Planning 입력으로 쓰기
전에 파인튜닝·정량평가·최종 detection message 계약이 필요하다.
