# MORAI PointPainting production object pipeline

## Production launch entrypoints

The production object chain is deliberately split across three terminals.
This is the canonical execution method; do not use
`pointpainting_with_tp.launch` or the generic `object_pipeline.launch` as a
shortcut.

```text
PP1 PointPainting detector (Conda Python 3.11)
  /perception/object_recognition/detection/objects
                         |
                         v
EKF tracker, internal 30 Hz (system ROS Python 3.8 / C++)
  /perception/object_recognition/tracking/measurement_objects
                         |
                         v
UNKNOWN/Obstacle stationary policy
  /perception/object_recognition/tracking/objects
                         |
                         v
Lanelet2 v61 Map-CV predictor
  /perception/object_recognition/objects
```

Terminal 1:

```bash
roscore
```

Terminal 2 uses the Conda environment created for that computer. The name and
location are not part of the repository contract:

```bash
conda activate <pointpainting-environment>
source /opt/ros/noetic/setup.bash
source /path/to/AjouNice2026/base_ws/devel/setup.bash

roslaunch morai_pointpainting_ros pointpainting_production.launch \
  python_executable:="$(command -v python)" \
  openpcdet_root:=/path/to/OpenPCDet \
  bundle_root:=/path/to/260909_pointpainting_production_3model
```

Terminal 3 must be a clean system-ROS shell rather than the detector Conda
shell:

```bash
source /opt/ros/noetic/setup.bash
source /path/to/AjouNice2026/base_ws/devel/setup.bash
roslaunch perception_object_pipeline tracking_prediction.launch
```

`repo_root` is derived from the installed ROS package. The three explicit
paths in Terminal 2 are the only machine-local inputs: Python interpreter,
OpenPCDet checkout, and model bundle. GPU selection is left to PyTorch/CUDA.

## Prerequisites owned by other stacks

The three commands above own only ROS master and the object pipeline. Before
driving, the following inputs and TF must also exist:

- MORAI sensor bridge: four compressed cameras and `/sensor/lidar1`.
- localization: a valid `map -> ... -> base_link` transform at sensor time.
- sensor static TF: `base_link -> lidar1` and all camera frames.

Localization, sensing, traffic-light perception, planning and control remain
separate processes. When this split production procedure is used, start
planning with `competition_planning.launch`.
`morai_planning_stack.launch` now defaults `start_object_pipeline:=false` for
the same reason. Its old integrated ownership is compatibility-only and must
be enabled explicitly; doing so together with `tracking_prediction.launch`
would create a second EKF and predictor on the same topics.

## Pinned production contract

- Detector: `B3S-PP1 E34`.
- Segmentation: `Base3 LRASPP E30`.
- Input: single-frame, 9 features, OpenCV preprocessing with four workers.
- Score thresholds: Vehicle `0.40`, Pedestrian `0.15`, Obstacle `0.10`.
- Detector velocity: disabled.
- Detection timestamp: `object_points_median`.
- Intra-scan coordinate deskew: disabled.
- Tracker: vehicle yaw canonicalization, EKF internal 30 Hz prediction output.
- Current-time compensation: disabled; stale age cap `0.25 s`.
- Obstacle representation: Autoware `UNKNOWN`, velocity/acceleration forced to
  zero after tracking while pose updates remain live.
- Predictor: Lanelet2 v61 Map-CV, `0.1 s` step, `3.0 s` horizon, up to six
  vehicle hypotheses. Non-vehicle/off-lane cases use kinematic fallback.

`pointpainting_production.launch` and `tracking_prediction.launch` explicitly
pin these values. Generic launch defaults are compatibility/research defaults,
not the production contract.

## Planning boundary

Planning consumes only:

```text
/perception/object_recognition/objects
autoware_perception_msgs/PredictedObjects
```

It must not consume detector boxes, legacy PointPainting messages, or marker
topics directly.

## Runtime verification

```bash
rostopic info /perception/object_recognition/detection/objects
rostopic info /perception/object_recognition/tracking/measurement_objects
rostopic info /perception/object_recognition/tracking/objects
rostopic info /perception/object_recognition/objects
```

Expected node ownership:

- detection publisher: `/pointpainting_online`.
- measurement-track publisher: `/ekf_multi_object_tracking`.
- canonical track publisher: `/tracked_object_motion_policy`.
- planning-object publisher: `/map_based_prediction`.

Rates:

```bash
rostopic hz --wall-time /sensor/lidar1
rostopic hz --wall-time /perception/object_recognition/detection/objects
rostopic hz --wall-time /perception/object_recognition/tracking/objects
rostopic hz --wall-time /perception/object_recognition/objects
```

Detection follows fresh LiDAR input, normally about 12--15 Hz. Tracking and
prediction are 30 Hz while the most recent detection is no older than 0.25 s.
The 30 Hz scheduler extrapolates EKF state; it does not invent measurements.

## RViz

```bash
roslaunch perception_object_pipeline object_pipeline_visualization.launch
```

RViz is diagnostic-only. The production transport remains the typed Autoware
object topics above.

## Rosbag evaluation

Use the dedicated evaluator. It starts an isolated ROS master and explicitly
propagates `use_sim_time:=true` to localization and the object pipeline.

```bash
perception/pointpainting/tools/run_rosbag_pipeline_evaluation.sh \
  INPUT.bag OUTPUT_DIR LABEL
```

Never use the live launches for a bag evaluation without changing every
participating launch to simulated time. Mixing wall time and bag timestamps
causes the EKF stale guard to reject its fixed-rate outputs.

For the verified Highway 1 qualitative result:

```bash
bash perception/pointpainting/tools/play_pp1_production_highway1_rviz.sh
```

The detailed model choice, metrics and rejected alternatives are in
`perception/pointpainting/docs/production_runtime_contract_260913.md`.
