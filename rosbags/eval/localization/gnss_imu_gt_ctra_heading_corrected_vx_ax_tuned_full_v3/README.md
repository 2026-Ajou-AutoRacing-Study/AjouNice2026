# 260805 v3 - ax 공분산 수정본

- 알고리즘: GNSS + GT motion + IMU CTRA-like EKF Fusion
- GT longitudinal velocity 표준편차: `0.01 m/s`
- IMU longitudinal acceleration 표준편차: `0.05 m/s²`
  - 실제 covariance 대각 원소: `0.05² = 0.0025 (m/s²)²`
- Heading 관측 편차: `+1.347 deg`
- 적용한 Heading 보정량: `-1.347 deg`
- GNSS 음영 처리: `(0, 0, 0)` 수신 시 GNSS 업데이트 차단
- 평가 범위: GNSS 음영 구간을 포함한 전체 약 `155.7 s` 주행
- 평가 raw rosbag:
  `rosbags/raw_sensor/competition_route_full_highspeed.bag`
- localization 결과 rosbag:
  `rosbags/eval/localization/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3.bag`
- 최종 가속도 출력:
  `/localization/acceleration`
  (`geometry_msgs/AccelWithCovarianceStamped`)

## 결과 파일

- `results/full.txt`
- `results/full.csv`
- `gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3.bag.info.txt`
- 평가는 `/Ego_topic.velocity`를 `m/s`로 해석하고, Ego heading에도
  `-1.347 deg`를 적용한 뒤 시작 yaw를 추정 정합했다.

## 전체 주행 결과

| 지표 | 결과 |
| --- | ---: |
| XY 평균 오차 | **0.128 m** |
| XY RMSE | **0.166 m** |
| XY 최대 오차 | 0.846 m |
| 종방향 MAE | **0.109 m** |
| 횡방향 MAE | **0.044 m** |
| Yaw MAE | 0.035 deg |
| `linear.x` MAE | **0.008 m/s** |
| 속도 MAE | **0.011 m/s** |

## v2 대비 ax 표준편차 A/B 비교

실행마다 달라지는 표본 구성을 배제하기 위해 두 bag에 공통으로 존재하는
timestamp만 사용한 비교 결과다.

| 지표 | v2 `σ=0.01 m/s²` | v3 `σ=0.05 m/s²` | 변화 |
| --- | ---: | ---: | ---: |
| `linear.x` bias | 0.00035 m/s | 0.00039 m/s | 사실상 동일 |
| `linear.x` MAE | 0.00763 m/s | **0.00711 m/s** | **6.8% 감소** |
| `linear.x` RMSE | 0.02406 m/s | **0.02121 m/s** | **11.8% 감소** |
| `linear.x` 최대 오차 | 0.475 m/s | **0.435 m/s** | **8.5% 감소** |
| Speed MAE | 0.01064 m/s | **0.01013 m/s** | **4.8% 감소** |
| Speed RMSE | 0.02643 m/s | **0.02395 m/s** | **9.4% 감소** |
| 종방향 위치 MAE | 0.10849 m | **0.10132 m** | **6.6% 감소** |
| 횡방향 위치 MAE | 0.04370 m | 0.04277 m | 사실상 동일 |
| 전체 XY 평균 오차 | 0.12670 m | 0.12752 m | 사실상 동일 |

가속도 출력 자체는 v2가 입력을 더 강하게 추종했다.

| 가속도 지표 | v2 `σ=0.01 m/s²` | v3 `σ=0.05 m/s²` |
| --- | ---: | ---: |
| acceleration MAE | **0.177 m/s²** | 0.268 m/s² |
| acceleration RMSE | **0.703 m/s²** | 0.883 m/s² |
| 입력과의 상관계수 | **0.911** | 0.851 |

v3는 가속도 측정값을 그대로 재현하는 능력은 v2보다 낮지만, 순간적인
가속도 spike가 속도 state에 전달되는 정도를 줄였다. 그 결과 전체
`linear.x`와 speed MAE, RMSE 및 최대 오차가 개선됐다. 위치 차이는 매우
작아 ax 표준편차만의 확정적인 효과로 과해석하지 않는다.

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
  rosbags/eval/localization/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3.bag \
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

`--alignment-yaw-mode none`은 평가용 사후 yaw 회전을 적용하지 않고 절대
map 방향을 유지한다. 위 결과표는 기존 버전과 동일한 정량 비교를 위해
기본 `estimate` 모드로 계산했으므로 RViz 실행 중 출력되는 XY 지표와
소폭 다를 수 있다.

### 터미널 4 — 원본 센서와 localization 결과 재생

```bash
# 원본 카메라·LiDAR와 v3 localization·가속도·TF를 함께 재생
cd ~/AjouNice2026
source base_ws/devel/setup.bash

rosbag play --clock --pause \
  rosbags/raw_sensor/competition_route_full_highspeed.bag \
  rosbags/eval/localization/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3.bag \
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
