# Perception object pipeline

This package defines the production ROS1 object boundary shared by
PointPainting, Fast-BEV, EKF tracking, prediction and the Autoware-derived
planning stack.

현재 PP1 production을 실행하는 터미널별 명령은
[`docs/runbooks/morai_pointpainting_object_pipeline_live.md`](../../../docs/runbooks/morai_pointpainting_object_pipeline_live.md)에
정리되어 있다.

```text
PointPainting or Fast-BEV
  autoware_perception_msgs/DetectedObjects
  /perception/object_recognition/detection/objects
                         |
                         v
EKF tracker
  autoware_perception_msgs/TrackedObjects
  /perception/object_recognition/tracking/measurement_objects
                         |
                         v
UNKNOWN/Obstacle stationary motion policy
  autoware_perception_msgs/TrackedObjects
  /perception/object_recognition/tracking/objects
                         |
                         v
CTRV/CV or Lanelet2 map-based predictor
  autoware_perception_msgs/PredictedObjects
  /perception/object_recognition/objects
                         |
                         v
Planning (unchanged)
```

## Production runtime

Production is the split three-terminal path below. Detector and downstream
must not share one Conda/roslaunch process tree.

```bash
# Terminal 1
roscore

# Terminal 2: B3S-PP1 E34 detector, from its Conda environment
roslaunch morai_pointpainting_ros pointpainting_production.launch \
  python_executable:="$(command -v python)" \
  openpcdet_root:=/path/to/OpenPCDet \
  bundle_root:=/path/to/260909_pointpainting_production_3model

# Terminal 3: EKF 30 Hz -> UNKNOWN stationary -> v63 Map-CV
roslaunch perception_object_pipeline tracking_prediction.launch
```

The detector runs in the machine's compatible Conda environment; tracking and
prediction run with ROS system Python 3.8. ROS messages are the only boundary
between them. Environment names, checkout paths and GPU models are not encoded
as repository aliases.

Production values are pinned by `pointpainting_production.launch` and
`tracking_prediction.launch`. The final
`/perception/object_recognition/objects` topic is the only object input
intended for Planning.

### MORAI GT detection replacement

To replace only PointPainting object detection with MORAI ground truth while
keeping the production EKF tracker and map-based predictor unchanged, do not
start PointPainting and run:

```bash
roslaunch perception_object_pipeline tracking_prediction.launch \
  use_morai_gt_detection:=true
```

The adapter converts `/Object_topic` (`morai_msgs/ObjectStatusList`) to the
canonical `/perception/object_recognition/detection/objects` topic in the
`morai_map` frame. It intentionally omits GT velocity so that downstream
tracking still estimates object motion. Passenger vehicle actor reference
positions are shifted by the audited 1.34 m longitudinal bbox-center offset.

## Generic and research runtime selection

Nodes use private `~input/objects` and `~output/objects` interfaces. Launch
remaps select the global topic, so model names are not hard-coded downstream.

```bash
# Fast-BEV (run under its Conda Python)
roslaunch perception_object_pipeline object_pipeline.launch \
  detector:=fastbev \
  python_executable:="$HOME/miniconda3/envs/fastbev_rtx50/bin/python"

# PointPainting (run under its Conda Python)
roslaunch perception_object_pipeline object_pipeline.launch \
  detector:=pointpainting \
  python_executable:="$HOME/miniconda3/envs/openpcdet_rtx50/bin/python"
```

These generic examples retain compatibility defaults and are not the current
competition profile. Current PointPainting production is B3S-PP1 E34 with
Base3 LRASPP E30 and class thresholds Vehicle 0.40, Pedestrian 0.15 and
Obstacle 0.10. The exact ignored artifact paths are pinned by the production
wrapper.

If a detector is already running in another environment, launch only the
standard tracker and predictor:

```bash
roslaunch perception_object_pipeline object_pipeline.launch \
  start_detector:=false
```

Different Python versions communicate safely because ROS serializes generated
messages across processes. All processes must source the same catkin workspace
so their message MD5 sums match.

## Standard RViz visualization

Launch the visualization adapter and the shared live RViz configuration after
the detector, tracker and predictor are running:

```bash
roslaunch perception_object_pipeline object_pipeline_visualization.launch
```

The visualization consumes only the canonical object messages. It does not
subscribe to the legacy PointPainting/Fast-BEV markers.

- translucent green boxes: detections in the detector measurement frame
- class-colored boxes and labels: map-frame tracks with stable short UUIDs
- class-colored lines: map-frame 0.0--3.0 second predicted paths
- small path spheres: 0.5-second samples (the message itself remains 0.1 s)

The RViz fixed frame is `map` and the camera follows `base_link`, so live use
requires the sensor static TF and the `map -> morai_map -> base_link` chain.

## Coordinate and timestamp contract

- Detector output uses its real measurement timestamp and sensor/Ego frame.
- EKF looks up that timestamp's TF and tracks in `map`.
- PointPainting uses the median acquisition time of points belonging to
  detected boxes for rolling LiDAR. A scan-wide median is used as fallback.
- EKF current-time output extrapolation exists as an experimental launch
  option but is disabled by default: roundabout/highway A/B evaluation showed
  that it degraded prediction compared with retaining measurement-time state.

### Tracking publish cadence

`object_pipeline.launch` uses the EKF-internal 30 Hz scheduler in production:

```bash
tracking_internal_publish_rate_hz:=30.0
tracking_publish_rate_hz:=0.0
maximum_tracking_internal_prediction_age_sec:=0.25
compensate_tracking_output_to_current_time:=false
```

The detector updates the EKF only when a new measurement arrives (for example,
about 12--15 Hz from PointPainting/LiDAR). Between measurements, the scheduler
asks the EKF motion model for a predicted output copy at each 30 Hz ROS-time
slot; it does not invent measurements or mutate the measurement-time filter
state. It stops publishing when the latest measurement is older than 0.25 s.

Set `tracking_internal_publish_rate_hz:=0.0` only for the detector-driven A/B
baseline. `tracking_publish_rate_hz` is the downstream adapter retained for
the earlier experiment and remains disabled in production. Do not enable both
fixed-rate mechanisms simultaneously.

```text
tracking_publish_rate_hz:=0.0
```

Set a positive value to route the measurement-time EKF output through the
causal fixed-rate adapter. The adapter never mutates the EKF state, does not
invent measurement updates, and stops publishing once the latest track is
older than `maximum_tracking_resample_age_sec`.

```text
tracking_publish_rate_hz:=15.0
tracking_publish_rate_hz:=30.0
```

These downstream-adapter settings are experimental and are not the production
path. Bag A/B tests showed that fixed-rate output adds a small prediction error,
but the EKF-internal 30 Hz scheduler was selected for production to provide a
regular, current-timestamp Tracking/Prediction interface to Planning. The
generic EKF package itself still defaults to detector-driven output; the 30 Hz
choice is made by this competition production launch.

### UNKNOWN obstacle motion policy

PointPainting publishes its third `Obstacle` class as Autoware `UNKNOWN`.
`object_pipeline.launch` applies a post-tracking motion policy before
prediction and optional downstream fixed-rate resampling:

```text
force_unknown_stationary:=true
```

The default policy preserves the latest tracked pose, dimensions,
classification and covariance, but sets UNKNOWN linear velocity, angular
velocity and acceleration to zero and marks the object stationary. This stops
frame-to-frame obstacle box jitter from becoming a moving CTRV/CV path while
still allowing every new detector observation to update the obstacle pose.
Vehicle and pedestrian tracks are unchanged. Set
`force_unknown_stationary:=false` only for a controlled A/B comparison.

- `TrackedObjects` and `PredictedObjects` remain in `map`.
- The predictor emits the current pose followed by 30 uniformly spaced future
  poses from 0.1 through 3.0 seconds (`time_step=0.1s`, 31 path poses total),
  matching the team-agreed planning contract.
- The full 3.0-second path is a message/runtime contract, not a claim that all
  points have equal accuracy. The prior evaluation still identifies 1.5
  seconds as the more strongly validated safety horizon.
- JSK boxes and visualization markers are debug-only and never carry IDs or
  kinematics between production nodes.

## Evaluation and map-based prediction

Moving CAR/TRUCK tracks are canonicalized at the EKF `TrackedObjects` output
boundary by default. The tracker preserves its world-frame velocity and box
axis, but chooses between the equivalent `yaw` and `yaw + pi` representations
so forward longitudinal speed agrees with the measured motion direction.
The 100/80-degree enter/exit thresholds prevent toggling around 90 degrees;
tracks below 1.0 m/s retain their previous decision. Disable the behavior only
for controlled A/B evaluation:

```bash
roslaunch perception_object_pipeline object_pipeline.launch \
  canonicalize_vehicle_orientation_to_motion:=false
```

The audit of the legacy CTRV/CTRA evaluation, the replacement benchmark
design, and the staged ROS1 port of Autoware map-based prediction are recorded
in
[`docs/tracking_prediction_evaluation_map_prediction_plan.md`](docs/tracking_prediction_evaluation_map_prediction_plan.md).
The legacy evaluation numbers are motion-model studies based mainly on MORAI
GT state; they are not treated as end-to-end accuracy measurements of the
current standardized pipeline.

The ROS1 map-based vehicle predictor is available behind an explicit launch
switch. Current production uses the K-City v63 Lanelet2 map and publishes the same
`PredictedObjects` contract, and falls back to CTRV/CV for low-speed, off-lane,
wrong-direction and non-vehicle objects.

```bash
# Existing production baseline
roslaunch perception_object_pipeline object_pipeline.launch \
  start_detector:=false \
  prediction_model:=ctrv

# Competition Map-CV path
roslaunch perception_object_pipeline object_pipeline.launch \
  start_detector:=false \
  prediction_model:=map_based
```

The map-based output still contains the current pose plus 0.1--3.0 second
samples. At a routing branch it may contain multiple `PredictedPath` entries;
their confidences are normalized to sum to one. Raw routes that reach the same
downstream Lanelet and remain within `0.75 m` of one another throughout the
prediction horizon are treated as one physical motion hypothesis. Requiring
the same downstream Lanelet prevents two legitimate sibling branches from
being collapsed while they are still geometrically close near a fork. The
highest-scoring route represents each group, so nearby start-lanelet matches
cannot crowd a real sibling branch out of the six-path safety cap. Planning
does not need a topic or message change.

The per-group maximum score is a conservative implementation, not a
calibrated probability model. Raw heuristic scores must not simply be summed,
because a hypothesis could gain confidence only by being enumerated through
more duplicate start Lanelets. A later improvement should normalize
`P(start lane | observation)` and `P(branch | start lane)` separately and then
marginalize probability mass into each geometric hypothesis. The generic
launch default remains CTRV for compatibility; `tracking_prediction.launch`
explicitly selects `prediction_model=map_based` and v63.

The complete 2026-08-20 production contract, evaluated metrics, rejected
ablations and restart order are documented in
[`map_cv_production_handoff_260820.md`](docs/map_cv_production_handoff_260820.md).

## Debug A/B topics

For simultaneous model comparison, override each detector's `objects_topic`
with `/perception/detection/fastbev/objects` or
`/perception/detection/pointpainting/objects`. Only one source should be
remapped to the canonical detection topic during a competition run.

## Legacy compatibility and planned cleanup

The production data path uses only the typed Autoware object messages:

```text
DetectedObjects -> TrackedObjects -> PredictedObjects
```

PointPainting currently continues to publish its pre-refactor outputs for
backward compatibility:

- `/pointpainting/detections` (`PointPaintingDetectionArray`)
- `/pointpainting/detection_markers` (`visualization_msgs/MarkerArray`)
- `/pointpainting/painted_points` (`sensor_msgs/PointCloud2`)

Fast-BEV is already decoupled from these PointPainting messages. It publishes
standard `diagnostic_msgs/DiagnosticArray` runtime data and retains JSK/Marker
outputs only for RViz. The tracker and predictor consume only canonical
Autoware object messages.

The PointPainting compatibility layer is intentionally retained until its
maintainer confirms that no team runbook, rosbag evaluation, RViz configuration
or external consumer still requires it. Removing it is a PointPainting-owned
migration and is independent from the Fast-BEV runtime.

Before removal, complete this checklist:

1. Agree on the canonical `DetectedObjects` contract with the PointPainting
   maintainer.
2. Search launch files, scripts, RViz configs, rosbag tooling and team
   documentation for legacy-topic consumers.
3. Migrate any necessary diagnostic or evaluation consumer to
   `/perception/object_recognition/detection/objects`.
4. Remove obsolete publishers, parameters, message dependencies and docs.
5. Rebuild and rerun the end-to-end Detection -> Tracking -> Prediction test
   with both PointPainting and Fast-BEV.
