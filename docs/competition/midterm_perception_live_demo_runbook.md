# 중간점검 대회용 MORAI 실시간 인지 시연 Runbook

## 목적

- 대회용 MORAI `25.S4.MolitComp03`에서 허용된 UDP 센서만 사용한다.
- 고주로에서 `PointPainting OD -> EKF Tracking -> Lanelet2 Map-CV Prediction`을
  실시간으로 실행한다.
- 화면 오른쪽에는 MORAI, 왼쪽에는 RViz를 배치해 하나의 영상으로 녹화한다.
- 판단·제어는 아직 개발 중이므로 수동 주행과 인지 pipeline만 시연한다.
- MORAI `/Object_topic` GT, GT BBox, GT future path는 사용하지 않는다.

## 시연 기준 모델

- PointPainting: `S1g E37`
- Detection score threshold: `0.40`
- PointPainting input: single-frame, 9-feature, direct velocity output 비활성화
- Tracking: EKF history 기반 velocity 추정
- Prediction: Lanelet2 v60 Map-CV, 1·2·3초 multi-path

## MORAI 설정

1. 대회용 런처 `mr`을 실행한다.
2. Map은 `R_KR_PR_K-city_2025`, 차량은 `2023_Hyundai_Ioniq5`, 날씨는
   `Sunny`로 설정한다.
3. 고주로 시연은 다음 scenario를 사용한다.

   ```text
   bench_highway_merge_raw.json
   ```

   저장소 원본은
   `configs/scenario/perception_benchmark/bench_highway_merge_raw.json`이며 대회용
   launcher의 Scenario 디렉터리에도 설치되어 있다.
4. F5에서 `configs/sensors/Sensor_6.json`과 동일한 sensor set을 load한다.
   - Camera 4대, LiDAR 1대, GPS 1대, IMU 1대.
   - 모든 카메라의 2D/3D BBox 및 Ground Truth를 끈다.
5. Network에서는 다음 대회 허용 UDP 항목만 MORAI에서 활성화한다.
   - Competition Vehicle Status -> destination port `9319`.
   - Collision Data -> destination port `9313`.
   - 수동 주행 시 Ego Ctrl Cmd는 활성화하지 않는다.
   - `/Ego_topic`, `/Object_topic`, ROS bridge 및 MORAI service는 사용하지 않는다.
6. Viewport에서 `Show Performance`, `Show Vehicle Cluster and Map Info`를 켠다.
7. Scenario load 후 `P`로 Parking 상태를 확인한다.

저장소의 `ego_network_competition_local.json`은 UDP receiver가 packet layout과 port를 읽는 내부
설정으로 사용할 수 있지만, MORAI GUI에서는 대회에서 허용되지 않은 Ego/Object/Service
연결을 활성화하면 안 된다.

## Terminal 1 — ROS master

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
roscore
```

대회용 실시간 UDP 경로에는 `rosbridge_websocket`이 필요하지 않다.

## Terminal 2 — MORAI UDP sensor/status receiver

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

rosparam set /use_sim_time false

roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$HOME/AjouNice2026/configs/sensors/Sensor_6.json" \
  network_config:="$HOME/AjouNice2026/configs/network/ego_network_competition_local.json" \
  enable_control:=false \
  lidar_rpm:=900.0
```

## Terminal 3 — Competition localization, sensor TF와 Lanelet map

`/Competition_topic`의 heading에는 현재 검증된 K-City 보정 `-1.347 deg`를 한 번만
적용한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

roslaunch bringup midterm_perception_live_support.launch \
  use_sim_time:=false \
  use_heading_correction:=true \
  heading_yaw_offset_deg:=-1.347 \
  sensor_config:="$HOME/AjouNice2026/configs/sensors/Sensor_6.json" \
  map_file:="$HOME/AjouNice2026/kcity_map/lanelet_map/lanelet2_map_v60.osm" \
  projector_file:="$HOME/AjouNice2026/kcity_map/lanelet_map/map_projector_info_v1.yaml"
```

이 통합 launch가 다음을 모두 발행한다.

- `/Competition_topic` + GPS + IMU 기반 `map -> odom -> base_link`.
- `base_link -> lidar1/camera_*` static TF.
- `/map/lanelet_map` Lanelet2 v60 marker.

별도의 `localization_gnss_imu_gt_ctra.launch`, `sensing.launch` 또는
`kcity_lanelet_map.launch`를 동시에 실행하지 않는다.

## Terminal 4 — PointPainting, EKF Tracking, Map-CV Prediction

```bash
cd ~/AjouNice2026

POINTPAINTING_CFG="$HOME/OpenPCDet/output/custom_models/pointpillar_morai_pointpainting_base2_singleframe9_geom_aug_gtdb/base2_s1g_gtdb_e40_s0/pointpillar_morai_pointpainting_base2_singleframe9_geom_aug_gtdb.yaml" \
POINTPAINTING_CKPT="$HOME/OpenPCDet/output/custom_models/pointpillar_morai_pointpainting_base2_singleframe9_geom_aug_gtdb/base2_s1g_gtdb_e40_s0/ckpt/best_center_mAP_epoch_37.pth" \
POINTPAINTING_SEG_CKPT="$HOME/AjouNice2026/artifacts/detection3d/pointpainting/releases/260817_sunny13_1500_9feat_80e/lraspp_4class_best_miou.pth" \
POINTPAINTING_SCORE_THRESHOLD=0.40 \
POINTPAINTING_PREPROCESS_BACKEND=opencv \
POINTPAINTING_CAMERA_CPU_WORKERS=4 \
POINTPAINTING_TEMPORAL_NUM_SWEEPS=0 \
POINTPAINTING_LIDAR_DESKEW_MODE=off \
POINTPAINTING_WITH_TP=true \
POINTPAINTING_PREDICTION_MODEL=map_based \
CONDA_ENV=openpcdet_rtx50 \
base_ws/src/morai_pointpainting_ros/scripts/run_pointpainting.sh live
```

모델 warm-up이 끝나고 detection이 발행될 때까지 기다린다.

## Terminal 5 — RViz

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

roslaunch perception_object_pipeline object_pipeline_visualization.launch
```

RViz의 기본 표시 색상은 다음과 같다.

- Detection: 초록색 3D box.
- Tracking: class별 색상의 box와 label.
- Prediction: 1·2·3초 경로와 horizon marker.
- Lanelet: 차선 경계와 centerline.
- LiDAR: intensity point cloud.

`MORAI GT BBoxes (magenta)`와 `Matched GT Future Paths (purple)` display는 topic이
발행되지 않으므로 나타나지 않는다. 혹시 이전 개발 node가 남아 있다면 두 display를
반드시 끈다.

## Terminal 6 — 촬영 전 상태 확인

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

rostopic echo -n 1 /Competition_topic
rostopic echo -n 1 /localization/kinematic_state

rostopic hz --wall-time \
  /sensor/lidar1 \
  /sensor/camera/front/compressed \
  /Competition_topic \
  /localization/kinematic_state \
  /perception/object_recognition/detection/objects \
  /perception/object_recognition/tracking/objects \
  /perception/object_recognition/objects
```

다음도 확인한다.

```bash
rosrun tf tf_echo map base_link
```

- 차량이 움직일 때 `map -> base_link`가 연속적으로 변해야 한다.
- RViz의 Ego Vehicle이 Lanelet 도로 위에 있어야 한다.
- LiDAR와 detection box가 Ego Vehicle 주변에 붙어 있어야 한다.

## 화면 배치

1. RViz 창을 선택하고 `Super + Left`로 왼쪽 절반에 배치한다.
2. MORAI 창을 선택하고 `Super + Right`로 오른쪽 절반에 배치한다.
3. RViz 왼쪽 `Displays` panel의 폭을 줄여 map viewport를 확보한다.
4. RViz view는 처음에는 `TopDownOrtho`, 필요하면 추적이 쉬운 `ThirdPersonFollower`를
   사용한다.
5. MORAI 카메라는 Ego 후방 추적 시점으로 두고 NPC와 주행 차선이 보이게 한다.
6. 터미널은 녹화 화면 밖 다른 workspace에 둔다.

## 권장 영상 순서

총 45~70초 정도를 권장한다.

1. **0~5초: 정지 화면**
   - MORAI와 RViz가 동일 고주로 위치를 표시하는지 보여준다.
   - Lanelet, LiDAR, detection/tracking/prediction 색상을 짧게 설명한다.
2. **5~15초: 합류 접근 및 가속**
   - 수동으로 Parking을 해제하고 고주로 합류 방향으로 주행한다.
   - 차량이 검출되고 track ID가 유지되는 장면을 담는다.
3. **15~40초: 순항 및 상대 차량 접근**
   - 앞·옆 NPC가 있는 상태에서 일정 속도로 주행한다.
   - Detection box보다 Tracking box가 안정적으로 이어지고, predicted path가 Lanelet을
     따라가는 장면을 확보한다.
4. **40~55초: 추월 또는 차선 관계 변화**
   - 무리한 급조향 없이 상대 차량과의 종·횡방향 관계가 바뀌는 장면을 담는다.
   - 분기 또는 곡률 변화에서 prediction 경로가 차선을 따르는 모습이 잘 보이면 좋다.
5. **55~65초: 마무리 정지**
   - 안전하게 감속하고 인지 pipeline이 계속 동작하는 화면으로 종료한다.

## 촬영 시 피해야 할 것

- `/Object_topic` 또는 MORAI GT BBox 표시.
- 개발용 rosbridge 연결.
- 센서 Viz BBox 토글 활성화.
- RViz TF 오류, `No transform` 경고가 계속 떠 있는 상태.
- 모델 warm-up 중 빈 화면.
- 과도한 급조향으로 차량이 도로를 벗어나는 장면.
- 판단·제어가 완성된 것처럼 설명하는 표현.

## 중단 및 재시작

재촬영 전에는 실행 terminal을 역순으로 `Ctrl-C`하고 다음을 확인한다.

```bash
cd ~/AjouNice2026
bash scripts/runtime/morai_status.sh
```

동일 node가 남아 있으면 다음 촬영 전에 정리한다.

```bash
cd ~/AjouNice2026
bash scripts/runtime/morai_reset.sh
```
