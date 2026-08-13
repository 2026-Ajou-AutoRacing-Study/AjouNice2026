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
CTRV/CV predictor
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
- `TrackedObjects` and `PredictedObjects` remain in `map`.
- The predictor emits the current pose followed by 30 uniformly spaced future
  poses from 0.1 through 3.0 seconds (`time_step=0.1s`, 31 path poses total),
  matching the team-agreed planning contract.
- The full 3.0-second path is a message/runtime contract, not a claim that all
  points have equal accuracy. The prior evaluation still identifies 1.5
  seconds as the more strongly validated safety horizon.
- JSK boxes and visualization markers are debug-only and never carry IDs or
  kinematics between production nodes.

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

Fast-BEV likewise retains its legacy detection, JSK box and marker outputs for
debugging and existing RViz configurations. These topics are diagnostic-only:
the tracker and predictor must not consume them, parse marker text, or derive
IDs and kinematics from visualization messages.

This compatibility layer is intentionally temporary. After reviewing the
change with the PointPainting maintainer and confirming that no team runbook,
rosbag evaluation, RViz configuration or external consumer still requires the
old interfaces, remove the legacy algorithm-facing detection publishers and
their unused custom-message dependencies. Keep a visualization output only if
it remains useful for RViz, and generate it from the canonical typed object
message rather than treating it as an inter-node interface.

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
