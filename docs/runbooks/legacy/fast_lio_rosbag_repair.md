# FAST-LIO rosbag 보정 절차

FAST-LIO offline 재생에서 LiDAR/IMU의 `header.stamp`가 bag 기록 시간과 어긋나거나,
LiDAR point의 `time` 필드가 음수여서 문제가 생길 때 사용하는 절차다.

## 1. header timestamp 보정

```bash
python3 scripts/rosbag/repair/fix_bag_header_stamp.py \
  rosbags/input.bag \
  rosbags/input_fixed_stamp.bag
```

모든 header 보정이 필요한 경우에만 `--all-header-topics`를 추가한다.

## 2. PointCloud 확인

```bash
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

python3 scripts/rosbag/inspect/check_cloud_points.py \
  rosbags/input_fixed_stamp.bag \
  /sensor/lidar1
```

## 3. FAST-LIO용 PointCloud time 보정

```bash
python3 scripts/rosbag/repair/convert_lidar_for_fastlio.py \
  rosbags/input_fixed_stamp.bag \
  rosbags/input_fastlio_timefix.bag \
  --input-topic /sensor/lidar1 \
  --output-topic /sensor/lidar1_fastlio
```

원본 bag은 보존하고 각 단계의 출력 bag을 별도 파일로 둔다. 초기 검증에서 사용한
고정 경로 스크립트는 `scripts/experiments/rosbag_legacy/`에만 보관한다.
