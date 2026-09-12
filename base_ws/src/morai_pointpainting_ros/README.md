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

## Class-aware score thresholds

The online output applies the class-specific operating points selected from
the PP1 MORAI rosbag evaluation:

- Vehicle: `0.40`
- Pedestrian: `0.15`
- Obstacle/Unknown: `0.10`

They are exposed as the launch arguments `vehicle_score_threshold`,
`pedestrian_score_threshold`, and `obstacle_score_threshold`. The legacy
`score_threshold` remains the fallback for an unrecognized detector class.
OpenPCDet's internal post-processing floor must be no higher than the desired
ROS output threshold.

## Prerequisites

- Ubuntu 20.04 / ROS Noetic
- CUDA GPU
- `openpcdet_rtx50` Conda environment
- OpenPCDet at `~/OpenPCDet`
- PP1 production bundle at
  `artifacts/team_handoffs/260909_pointpainting_production_3model/`
- B3S-PP1 E34 detector and Base3 LRASPP E30 files from that bundle

Install ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-cv-bridge \
  ros-noetic-visualization-msgs
```

Runtime does not download model weights. Provision the bundle locally and pass
its path explicitly to the production launch.

## Workspace build

```bash
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
catkin build morai_pointpainting_ros perception_object_pipeline
source devel/setup.bash
```

The shell must use the OpenPCDet Conda Python. Do not keep a second virtualenv active at the same time:

```bash
deactivate 2>/dev/null || true
conda activate openpcdet_rtx50
source /opt/ros/noetic/setup.bash
source ~/AjouNice2026/base_ws/devel/setup.bash
which python
```

Expected Python:

```text
$CONDA_PREFIX/bin/python
```

## Production execution

Do not launch detector, tracker and predictor together from the detector Conda
environment. Use the explicit ROS launch entrypoints in separate terminals:

```bash
# Terminal 1
roscore

# Terminal 2: activate this machine's detector environment first
roslaunch morai_pointpainting_ros pointpainting_production.launch \
  python_executable:="$(command -v python)" \
  openpcdet_root:=/path/to/OpenPCDet \
  bundle_root:=/path/to/260909_pointpainting_production_3model

# Terminal 3: EKF -> UNKNOWN stationary policy -> v62 Map-CV
roslaunch perception_object_pipeline tracking_prediction.launch
```

The authoritative runbook is
`docs/runbooks/morai_pointpainting_object_pipeline_live.md`. The generic
`pointpainting_online.launch`, `pointpainting_with_tp.launch`, and
`object_pipeline.launch` entrypoints remain available for controlled research
and compatibility, but their defaults do not define production.

For recorded data use
`perception/pointpainting/tools/run_rosbag_pipeline_evaluation.sh`; it owns an
isolated ROS master and enforces simulated time across localization, EKF and
prediction.

## RViz

```bash
roslaunch perception_object_pipeline object_pipeline_visualization.launch
```

The production view uses fixed frame `map` and the canonical detection,
tracking and prediction messages. Legacy detector-only debugging may instead
use fixed frame `lidar1` with:

- `MarkerArray`: `/pointpainting/detection_markers`
- `PointCloud2`: `/pointpainting/painted_points` when enabled

Debug cloud publishing is disabled by the production launch to reduce copying
and bandwidth.

## Real-time acceptance checks

Production uses OpenCV preprocessing and four CPU camera workers. In the
2026-08-16 backend A/B at camera 34.6 Hz / LiDAR 15.0 Hz, that path measured:

- detection publish: 14.97 Hz
- total compute p50/p95: 45.2 / 48.9 ms
- arrival-to-publish p50/p95: 46.0 / 53.0 ms
- continuously growing queue: not observed

The historical PIL path measured 54.9 / 59.5 ms compute p50/p95 and one LiDAR
replacement in 464 frames. It is retained only as an A/B reference.

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
