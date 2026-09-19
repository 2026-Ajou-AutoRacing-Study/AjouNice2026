# GNSS + GT Motion + IMU CTRA-like EKF — vx tuned full v2

## 실행 조건

- 알고리즘: GNSS + GT motion + IMU CTRA-like EKF Fusion
- GT longitudinal velocity 표준편차: `0.01 m/s`
- Heading 보정: `-1.347 deg`
- GNSS 음영 처리: `(0, 0, 0)` 수신 시 GNSS 업데이트 차단
- 평가 범위: GNSS 음영 구간을 포함한 전체 약 `155.7 s` 주행
- 평가 raw rosbag:
  `rosbags/raw_sensor/competition_route_full_highspeed.bag`
- localization 결과 rosbag:
  `rosbags/eval/localization/gnss_imu_gt_ctra_heading_corrected_vx_tuned_full_v2/gnss_imu_gt_ctra_heading_corrected_vx_tuned_full_v2.bag`
- 최종 가속도 출력:
  `/localization/acceleration`
  (`geometry_msgs/AccelWithCovarianceStamped`)

## 결과 파일

- `results/full.txt`
- `results/full.csv`
- `gnss_imu_gt_ctra_heading_corrected_vx_tuned_full_v2.bag.info.txt`

평가는 `/Ego_topic.velocity`를 `m/s`로 해석하고, Ego heading에도
`-1.347 deg`를 적용한 뒤, 기존 v1과 동일하게 시작 yaw를 추정 정합했다.

## 전체 주행 결과

| 지표 | 결과 |
| --- | ---: |
| XY 평균 오차 | **0.127 m** |
| XY RMSE | **0.165 m** |
| XY 최대 오차 | 0.850 m |
| 종방향 MAE | **0.108 m** |
| 횡방향 MAE | **0.044 m** |
| Yaw MAE | 0.035 deg |
| `linear.x` MAE | **0.008 m/s** |
| 속도 MAE | **0.011 m/s** |

기존 `vx sigma=0.05 m/s` v1과 비교하면 전체 XY와 횡방향 오차는
사실상 같고, `linear.x`, speed 및 종방향 오차가 개선됐다.

## RViz 시각화

### 터미널 1 — ROS master

```bash
# 기존 ROS 노드를 종료한 뒤 ROS master 실행
cd ~/AjouNice2026/base_ws
source devel/setup.bash

roscore
```

### 터미널 2 — Lanelet v60·MGeo 중심선·차량·RViz

```bash
# Lanelet v60과 MGeo 중심선, 차량 모델 및 RViz 실행
cd ~/AjouNice2026/base_ws
source devel/setup.bash

rosparam set /use_sim_time true

roslaunch localization_manager \
  gnss_imu_lanelet_rviz.launch \
  lanelet_map_file:=/home/luv/AjouNice2026/kcity_map/lanelet_map/lanelet2_map_v60.osm
```

이 launch는 `map -> morai_map`을 발행하지 않는다. 터미널 4에서 재생하는
localization 결과 bag의 `/tf_static`을 사용한다.

### 터미널 3 — GT·측위 경로와 오차 Marker

```bash
# 절대 map 방향을 유지한 GT·측위 경로와 오차 Marker를 RViz에 발행
cd ~/AjouNice2026
source base_ws/devel/setup.bash

python3 scripts/evaluation/localization/eval_localization_state_with_rviz.py \
  rosbags/eval/localization/gnss_imu_gt_ctra_heading_corrected_vx_tuned_full_v2/gnss_imu_gt_ctra_heading_corrected_vx_tuned_full_v2.bag \
  --state-topic /localization/kinematic_state \
  --sensor-config configs/sensors/Sensor.json \
  --ego-velocity-unit mps \
  --ego-heading-yaw-offset-deg -1.347 \
  --alignment-reference-sec 0.0 \
  --alignment-yaw-mode none \
  --rviz-ground-z -0.3105 \
  --position-error-threshold 1.0 \
  --speed-error-threshold 0.5 \
  --yaw-rate-error-threshold 0.1 \
  --publish-rviz
```

`--alignment-yaw-mode none`은 RViz에서 평가 전용 사후 yaw 회전을 적용하지
않고 절대 map 방향을 유지한다. 위 결과표는 기존 v1과 동일한 정량 비교를
위해 기본 `estimate` 모드로 계산했으므로 RViz 실행 중 출력되는 XY 지표와
소폭 다를 수 있다.

### 터미널 4 — 원본 센서와 localization 결과 재생

```bash
# 원본 카메라·LiDAR와 v2 localization·가속도·TF를 함께 재생
cd ~/AjouNice2026
source base_ws/devel/setup.bash

rosbag play --clock --pause \
  rosbags/raw_sensor/competition_route_full_highspeed.bag \
  rosbags/eval/localization/gnss_imu_gt_ctra_heading_corrected_vx_tuned_full_v2/gnss_imu_gt_ctra_heading_corrected_vx_tuned_full_v2.bag \
  --topics \
  /sensor/lidar1 \
  /sensor/camera/front/compressed \
  /sensor/camera/front_left/compressed \
  /sensor/camera/front_right/compressed \
  /sensor/camera/back/compressed \
  /localization/kinematic_state \
  /localization/acceleration \
  /tf \
  /tf_static
```

`Space`로 재생을 시작한다.
