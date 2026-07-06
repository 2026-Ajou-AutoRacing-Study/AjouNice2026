python3 scripts/fix_bag_header_stamp.py \
  rosbags/fast_lio_lidar_v1.bag \
  rosbags/fast_lio_lidar_v1_fixed_stamp.bag

rosbag info ~/AjouNice2026/rosbags/fast_lio_lidar_v1_fixed_stamp.bag


chmod +x ~/AjouNice2026/scripts/check_cloud_points.py

cd ~/AjouNice2026/base_ws
source devel/setup.bash

python3 ~/AjouNice2026/scripts/check_cloud_points.py \
  ~/AjouNice2026/rosbags/fast_lio_lidar_v1_fixed_stamp.bag \
  /sensor/lidar1

cd ~/AjouNice2026/base_ws
source devel/setup.bash

cd ~/AjouNice2026

python3 scripts/convert_lidar_for_fastlio.py \
  rosbags/fast_lio_lidar_v1_fixed_stamp.bag \
  rosbags/fast_lio_lidar_v1_fastlio_timefix.bag \
  --input-topic /sensor/lidar1 \
  --output-topic /sensor/lidar1_fastlio


원본 bag: fast_lio_lidar_v1.bag
  문제 1: LiDAR / IMU header.stamp가 서로 안 맞음

1차 수정 bag: fast_lio_lidar_v1_fixed_stamp.bag
  해결: 모든 header.stamp를 bag time 기준으로 통일
  남은 문제: LiDAR point의 time field가 음수

2차 수정 bag: fast_lio_lidar_v1_fastlio_timefix.bag
  해결 목표: header.stamp는 이미 맞춘 상태 유지
             + point time field를 0부터 시작하도록 normalize


# 평가
cd ~/AjouNice2026/rosbags
rosbag record \
  /clock \
  /Ego_topic \
  /sensor/imu \
  /sensor/lidar1 \
  /localization/odom \
  /path \
  -O fast_lio_lidar_v1.bag