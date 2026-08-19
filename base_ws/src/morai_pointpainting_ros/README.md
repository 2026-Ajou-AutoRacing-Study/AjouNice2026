# MORAI PointPainting ROS

ROS1 Noetic package that converts four camera images and one LiDAR scan into
three-class oriented 3D detections.

## Why this package exists

The competition perception chain must not stop at an offline `.npy` result. The same node must work with:

1. `rosbag play` using the recorded sensor topics.
2. Live MORAI camera and LiDAR topics during the competition.
3. A downstream tracking/prediction node that consumes timestamped 3D boxes.

The node therefore subscribes to the original sensor topics, performs segmentation, PointPainting, PointPillars inference, and publishes a stable ROS message interface.

## Runtime graph

```text
/sensor/camera/front/compressed
/sensor/camera/front_left/compressed
/sensor/camera/front_right/compressed
/sensor/camera/back/compressed
          │
          ├── latest image at or before LiDAR timestamp
          ├── MORAI LRASPP MobileNetV3 4-class segmentation
          │
/sensor/lidar1
          ├── projection + camera-probability averaging
          ├── 9 features: XYZI + bg/ped/veh/obs/unknown
          ├── PointPillars 3-class inference
          ├── /perception/object_recognition/detection/objects (official)
          └── /pointpainting/detections (legacy diagnostics)
```

The LiDAR queue is intentionally size 1. When processing is slower than sensor input, stale scans are replaced by the newest scan instead of building latency.

## Default topics

Inputs:

- `/sensor/camera/front/compressed`
- `/sensor/camera/front_left/compressed`
- `/sensor/camera/front_right/compressed`
- `/sensor/camera/back/compressed`
- `/sensor/lidar1`

Outputs:

- `/perception/object_recognition/detection/objects`
- `/pointpainting/detections`
- `/pointpainting/detection_markers`
- `/pointpainting/painted_points` when debug publishing is enabled

## Prerequisites

- Ubuntu 20.04 / ROS Noetic
- CUDA GPU
- `openpcdet_rtx50` Conda environment
- OpenPCDet at `~/OpenPCDet`
- release bundle at
  `artifacts/detection3d/pointpainting/releases/260817_sunny13_1500_9feat_80e/`

Install ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-cv-bridge \
  ros-noetic-visualization-msgs
```

The release contains both the MORAI LRASPP checkpoint and PointPillars
checkpoint. Runtime does not download VOC segmentation weights.

## Build

```bash
mkdir -p ~/catkin_ws/src
cp -r morai_pointpainting_ros ~/catkin_ws/src/

cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

The shell must use the OpenPCDet Conda Python. Do not keep a second virtualenv active at the same time:

```bash
deactivate 2>/dev/null || true
conda activate openpcdet_rtx50
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
which python
```

Expected Python:

```text
$CONDA_PREFIX/bin/python
```

## Test with the recorded rosbag

Terminal 1:

```bash
roscore
```

Terminal 2:

```bash
deactivate 2>/dev/null || true
conda activate openpcdet_rtx50
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosparam set use_sim_time true

roslaunch morai_pointpainting_ros pointpainting_online.launch \
  publish_painted_points:=true
```

Terminal 3:

```bash
source /opt/ros/noetic/setup.bash
rosbag play --clock \
  ~/rosbags/sensor_cam_raw_ped1.bag
```

Terminal 4:

```bash
deactivate 2>/dev/null || true
conda activate openpcdet
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosrun morai_pointpainting_ros check_pointpainting_output.py
```

## RViz

```bash
rviz
```

Set Fixed Frame to `lidar1`, then add:

- `MarkerArray`: `/pointpainting/detection_markers`
- `PointCloud2`: `/pointpainting/painted_points` when enabled

## Live MORAI run

The node command is unchanged because the live simulator uses the same topics:

```bash
rosparam set use_sim_time false

roslaunch morai_pointpainting_ros pointpainting_online.launch \
  publish_painted_points:=false
```

Debug cloud publishing should be disabled for the final run to reduce copying and bandwidth.

## Real-time acceptance checks

The 2026-08-16 runtime benchmark uses four CPU workers for camera decode and
the training-compatible torchvision/PIL preprocessing path. At a stress input
of camera 34.6 Hz / LiDAR 15.0 Hz it measured:

- detection publish: 14.97 Hz
- total compute p50/p95: 54.9 / 59.5 ms
- arrival-to-publish p50/p95: 58.5 / 72.5 ms
- LiDAR replacement: 1/464 frames (0.22%)
- continuously growing queue: not observed

During rosbag and live tests, confirm:

- `total_ms` usually remains below the LiDAR period.
- `queue_delay_ms` does not grow continuously.
- `dropped_lidar_frames` does not increase continuously.
- `camera_age_max_ms` stays below 100 ms.
- output `header.stamp` is the representative object-point acquisition time
  when `lidar_measurement_stamp_mode:=object_points_median` (production
  default). `header` mode preserves the source LiDAR scan stamp for A/B tests.
- output `header.frame_id` is `lidar1`.

Run a reproducible benchmark with:

```bash
perception/pointpainting/tools/run_rosbag_runtime_benchmark.sh \
  rosbags/raw_sensor_w_cam/sensor_cam_raw_roundabout1.bag \
  1.73 \
  artifacts/detection3d/pointpainting/benchmarks/roundabout_x1p73
```

See
[`perception/pointpainting/docs/runtime_benchmark_260816.md`](../../../perception/pointpainting/docs/runtime_benchmark_260816.md)
for stage timings, throughput/latency separation, and the OpenCV comparison.

## Temporal matching limitation

Version 0.1 uses the newest camera image whose timestamp is not later than the LiDAR timestamp. This is causal and works for both rosbag and live operation.

The production rolling-LiDAR correction changes only the detection measurement
timestamp. The representative timestamp is the median of per-box point-time
medians, with the scan point median as fallback. This avoids letting one dense
box dominate the frame timestamp.

An optional `ego_pose_2d` point-coordinate deskew is also implemented, but it
remains disabled. Highway A/B improved static scan alignment while worsening
detection precision, tracking and Map-CV prediction with the current checkpoint.
See
[`rolling_lidar_ego_deskew_ab_260820.md`](../../../perception/pointpainting/docs/rolling_lidar_ego_deskew_ab_260820.md).

Experimental controls are:

- `lidar_deskew_mode:=off|ego_pose_2d`.
- `lidar_deskew_reference:=header|point_median`.
- `lidar_deskew_camera_reference:=header|reference`.

When deskew is enabled, `reference` camera matching keeps point coordinates and
camera observations at the same time and is the physically consistent default.

The production path does not apply ego-motion compensation between camera and
LiDAR timestamps. Enabling coordinate deskew without training-time deskew is
not recommended for the current model.

## Git repository policy

Commit:

- this ROS package
- camera/LiDAR calibration YAML
- launch files
- message definitions
- exact OpenPCDet config copies
- checkpoint checksum and evaluation summary

Do not commit:

- rosbags
- generated datasets
- OpenPCDet build outputs
- Conda environments
- large checkpoints unless the team intentionally uses Git LFS

Recommended branch:

```bash
git checkout -b feature/pointpainting-online-ros
```

See `docs/INTERFACE.md` for the downstream contract.
