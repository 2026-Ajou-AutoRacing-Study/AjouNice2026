# MORAI Fast-BEV object pipeline live runbook

MORAI에서 `configs/sensors/Sensor_6.json`과
`configs/network/ego_network_1.json`을 로드하고 UDP 연결을 활성화한 뒤 아래
터미널을 순서대로 실행한다. 이 절차에는 신호등 인식과 Planning stack이
포함되지 않는다.

## 터미널 1 — ROS master

```bash
# ROS master 실행
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

roscore
```

## 터미널 2 — MORAI Sensor/Ego UDP

```bash
# Sensor_6의 카메라·LiDAR와 Ego 상태를 ROS topic으로 변환
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

rosparam set /use_sim_time false

roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$HOME/AjouNice2026/configs/sensors/Sensor_6.json" \
  network_config:="$HOME/AjouNice2026/configs/network/ego_network_1.json" \
  enable_control:=false \
  lidar_rpm:=900.0
```

## 터미널 3 — Sensor static TF

```bash
# base_link에서 네 카메라와 LiDAR까지의 static TF 발행
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

roslaunch bringup sensing.launch \
  sensor_config:="$HOME/AjouNice2026/configs/sensors/Sensor_6.json" \
  start_rviz:=false \
  enable_vehicle_model:=false \
  enable_ego_tf:=false
```

## 터미널 4 — MORAI GT localization과 Lanelet map

```bash
# /Ego_topic으로 map→morai_map→base_link TF와 localization 상태 발행
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

roslaunch bringup localization_morai_gt.launch \
  use_sim_time:=false \
  start_lanelet_map:=true \
  map_file:="$HOME/AjouNice2026/kcity_map/lanelet_map/lanelet2_map_v60.osm" \
  projector_file:="$HOME/AjouNice2026/kcity_map/lanelet_map/map_projector_info_v1.yaml"
```

## 터미널 5 — Fast-BEV, tracking, prediction

```bash
# Fast-BEV → EKF tracking → Lanelet2 Map P1 prediction 실행
cd ~/AjouNice2026

FASTBEV_WITH_TP=true \
FASTBEV_PREDICTION_MODEL=map_based \
CONDA_ENV=fastbev_rtx50 \
base_ws/src/morai_fastbev_ros/scripts/run_fastbev.sh live
```

## 터미널 6 — Object pipeline RViz

```bash
# 미래 궤적은 얇은 선, 1.5 s와 3.0 s는 작은 점으로 표시
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

roslaunch perception_object_pipeline object_pipeline_visualization.launch
```

## 터미널 7 — 입력과 파이프라인 Hz

```bash
# 카메라 입력과 detection/tracking/prediction 발행률 실시간 확인
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

rostopic hz --wall-time \
  /sensor/camera/front/compressed \
  /perception/object_recognition/detection/objects \
  /perception/object_recognition/tracking/objects \
  /perception/object_recognition/objects
```

## 터미널 8 — Planning 전달 직전 객체 확인

```bash
# 최종 PredictedObjects 메시지를 연속 출력
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

rostopic echo /perception/object_recognition/objects
```
