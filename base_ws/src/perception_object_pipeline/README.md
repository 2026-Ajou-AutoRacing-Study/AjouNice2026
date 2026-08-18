# Perception object pipeline

This package defines the production ROS1 object boundary shared by
PointPainting, Fast-BEV, EKF tracking, prediction and the Autoware-derived
planning stack.

MORAI live 환경을 처음부터 실행하는 터미널별 명령은
[`docs/runbooks/morai_fastbev_object_pipeline_live.md`](../../../docs/runbooks/morai_fastbev_object_pipeline_live.md)에
정리되어 있다.

```text
PointPainting or Fast-BEV
  autoware_perception_msgs/DetectedObjects
  /perception/object_recognition/detection/objects
                         |
                         v
EKF tracker
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

## Runtime selection

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

The PointPainting default profile is the 2026-08-16 Sunny13 release:

- LRASPP: `background/pedestrian/vehicle/obstacle`
- painted point contract: 9 features including `unknown`
- PointPillars: `Pedestrian/Vehicle/Obstacle`, with six obstacle anchor sizes
- online output range: 30 m, score threshold: 0.10

The checkpoints stay under ignored `artifacts/`; launch files point to the
stable release directory name rather than an external teammate workspace.

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

The first ROS1 map-based vehicle predictor is available behind an explicit
launch switch. It uses the K-City v60 Lanelet2 map, publishes the same
`PredictedObjects` contract, and falls back to CTRV/CV for low-speed, off-lane,
wrong-direction and non-vehicle objects.

```bash
# Existing production baseline
roslaunch perception_object_pipeline object_pipeline.launch \
  start_detector:=false \
  prediction_model:=ctrv

# P1 map-based prototype
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

The per-group maximum score is a conservative first implementation, not a
calibrated probability model. Raw heuristic scores must not simply be summed,
because a hypothesis could gain confidence only by being enumerated through
more duplicate start Lanelets. A later improvement should normalize
`P(start lane | observation)` and `P(branch | start lane)` separately and then
marginalize probability mass into each geometric hypothesis. CTRV remains the
default until the E0 map-based benchmark shows a measurable improvement.

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
