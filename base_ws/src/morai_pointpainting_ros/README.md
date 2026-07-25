# MORAI PointPainting ROS

ROS1 Noetic package that converts four camera images and one LiDAR scan directly into oriented 3D pedestrian boxes.

## Why this package exists

The competition perception chain must not stop at an offline `.npy` result. The same node must work with:

1. `rosbag play` using the recorded sensor topics.
2. Live MORAI camera and LiDAR topics during the competition.
3. A downstream tracking/prediction node that consumes timestamped 3D boxes.

The node therefore subscribes to the original sensor topics, performs segmentation, PointPainting, PointPillars inference, and publishes a stable ROS message interface.

## Runtime graph

```text
/sensor/cam/front
/sensor/cam/front_left
/sensor/cam/front_right
/sensor/cam/back
          │
          ├── latest image at or before LiDAR timestamp
          ├── LRASPP MobileNetV3 semantic segmentation
          │
/sensor/lidar1
          ├── projection + semantic painting (10 point features)
          ├── PointPillars epoch-20 inference
          └── /pointpainting/detections
                └── tracking / trajectory prediction / planning
```

The LiDAR queue is intentionally size 1. When processing is slower than sensor input, stale scans are replaced by the newest scan instead of building latency.

## Default topics

Inputs:

- `/sensor/cam/front`
- `/sensor/cam/front_left`
- `/sensor/cam/front_right`
- `/sensor/cam/back`
- `/sensor/lidar1`

Outputs:

- `/pointpainting/detections`
- `/pointpainting/detection_markers`
- `/pointpainting/painted_points` when debug publishing is enabled

## Prerequisites

- Ubuntu 20.04 / ROS Noetic
- CUDA GPU
- existing `openpcdet` Conda environment
- OpenPCDet at `~/OpenPCDet`
- trained checkpoint:
  `~/OpenPCDet/output/custom_models/pointpillar_morai_pointpainting/pointpainting_20epoch/ckpt/checkpoint_epoch_20.pth`

Install ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-cv-bridge \
  ros-noetic-visualization-msgs
```

Pre-download segmentation weights before competition day:

```bash
conda activate openpcdet
python src/morai_pointpainting_ros/scripts/download_segmentation_weights.py
```

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
conda activate openpcdet
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
conda activate openpcdet
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

The current measured offline budget was approximately:

- segmentation + painting: 78.5 ms
- PointPillars: 21.2 ms
- total compute: about 100 ms

The original 8.7 Hz test bag gives about 115 ms per LiDAR scan. During the rosbag and live tests, confirm:

- `total_ms` usually remains below the LiDAR period.
- `queue_delay_ms` does not grow continuously.
- `dropped_lidar_frames` does not increase continuously.
- `camera_age_max_ms` stays below 100 ms.
- output `header.stamp` equals the source LiDAR stamp.
- output `header.frame_id` is `lidar1`.

## Temporal matching limitation

Version 0.1 uses the newest camera image whose timestamp is not later than the LiDAR timestamp. This is causal and works for both rosbag and live operation.

It does not yet apply ego-motion compensation between the camera and LiDAR timestamps. The recorded bag had camera matching errors in the tens of milliseconds, so the first integration test must inspect box stability while the ego vehicle is moving. Ego-motion compensation should be added only if that test shows a measurable projection error.

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
