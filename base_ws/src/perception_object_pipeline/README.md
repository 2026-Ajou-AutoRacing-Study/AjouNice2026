# Perception object pipeline

This package defines the production ROS1 object boundary shared by
PointPainting, Fast-BEV, EKF tracking, prediction and the Autoware-derived
planning stack.

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

## Coordinate and timestamp contract

- Detector output uses its real measurement timestamp and sensor/Ego frame.
- EKF looks up that timestamp's TF and tracks in `map`.
- `TrackedObjects` and `PredictedObjects` remain in `map`.
- The predictor emits the current pose followed by five uniformly spaced poses
  from 0.3 through 1.5 seconds (`time_step=0.3s`).
- JSK boxes and visualization markers are debug-only and never carry IDs or
  kinematics between production nodes.

## Debug A/B topics

For simultaneous model comparison, override each detector's `objects_topic`
with `/perception/detection/fastbev/objects` or
`/perception/detection/pointpainting/objects`. Only one source should be
remapped to the canonical detection topic during a competition run.
