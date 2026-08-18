# Tracking prediction evaluation

This package contains the replacement evaluation pipeline for tracking and
prediction. It provides E0, a causal oracle benchmark that feeds MORAI GT
history directly to predictors, and E2, an end-to-end evaluator for
standardized tracking and prediction output bags.

E0 deliberately does **not** evaluate tracking. It excludes detector misses,
association, EKF state error and runtime latency. Its purpose is to measure the
error floor caused by the motion-model assumptions before a map-based model is
implemented.

The evaluator reconstructs persistent GT trajectories by causal spatial
correspondence. It does not trust MORAI `unique_id`, which is reused between
actors in the recorded bags, or `ObjectStatus.velocity`, whose unit differs
between recordings. Speed, yaw rate and acceleration are estimated only from
past GT poses.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

/usr/bin/python3 base_ws/src/tracking_prediction_evaluation/scripts/evaluate_oracle_prediction.py \
  rosbags/raw_sensor/roundabout1.bag \
  --classes vehicle \
  --include-map-based \
  --output-dir /tmp/ajounice_e0_roundabout
```

Use `/usr/bin/python3` even if `fastbev_rtx50` is active. ROS Noetic's rosbag
and Lanelet2 Python extensions target Ubuntu's Python 3.8, whereas the Fast-BEV
Conda environment uses Python 3.11.

Raw MORAI `/Object_topic` positions are expressed in `morai_map`. The evaluator
loads `configs/tf/morai_lanelet_static_tf.yaml` and evaluates all models in the
Lanelet `map` frame. Rigid transformation leaves the CTRV/CTRA error magnitude
unchanged, while preventing a false all-fallback Map P1 result. The report
records the transform path, v60 map hash, mapped-origin count and fallback count.

Map P1 is evaluated using its highest-confidence path. This is the top-1 path
that planning receives as the primary hypothesis; min-K and confidence
calibration remain later evaluation work.

## Oracle-GT Map P1 RViz

For prediction-only inspection, do not launch Fast-BEV or the EKF tracker.
`oracle_map_prediction_rviz.launch` converts `/Object_topic` directly into
map-frame `TrackedObjects`, estimates longitudinal speed from the previous
0.5 seconds of GT positions, runs the same Map P1 node used by the standard
pipeline, and opens RViz.

```bash
roslaunch tracking_prediction_evaluation oracle_map_prediction_rviz.launch
```

Then play a bag containing `/Object_topic` and `/Ego_topic` with `/clock`.
The adapter exists only for E0 evaluation and visualization; it must never be
connected to the production planning pipeline as a detector/tracker substitute.

Read `summary.json` first. `worst_samples.jsonl` contains only the configured
number of largest-FDE samples and is intended for targeted debugging.

The validated initial benchmark and discarded legacy assumptions are recorded
in [`docs/e0_oracle_baseline.md`](docs/e0_oracle_baseline.md).

The former custom-message `tracking_prediction` package and its evaluation
scripts were removed from the current tree after the audit. They remain in Git
history only. Their reported values must not be described as end-to-end results
of the current standardized object pipeline.

## E2 PointPainting pipeline comparison

`evaluate_pipeline_prediction.py` compares the real standardized pipeline
`PointPainting -> EKF tracking -> Map-CV prediction` against MORAI GT. It
reports 30 m vehicle tracking precision/recall/F1, MOTA, ID switches,
center/speed/yaw error, and 1/2/3-second prediction metrics. The legacy
`tracking.motp_center_error_m` key is retained for compatibility and aliases
the active GT reference selected by `--gt-reference`. With the default
calibrated policy, use the explicit
`tracking.bbox_center_absolute_distance_m` key for production comparison. With
`--gt-reference object-status`, the same legacy key remains an actor-pivot
distance and must not be described as bbox-center MOTP.

The CLI now defaults to `--gt-reference bbox-center-calibrated-v1` for the
production comparison. It shifts only MORAI GT, never detector/tracker output.
The calibrated policy currently supports:

- passenger vehicles up to `5.5 m`: `+1.340 m` along each GT heading;
- the observed `6.557 x 2.517 x 2.505 m` medium vehicle profile:
  `+1.383 m`;
- every other large or missing-size profile: excluded from the primary metric
  and counted under `unsupported_segment_count`.

Profiles are matched by the reported 3D dimensions rather than
`ObjectStatus.name`: the recorded highway bag reuses the same name for
different dimensions, so that field is not a stable asset identifier.

The same report retains the uncorrected legacy result under
`object_status_reference_evaluation`. To reproduce the historical primary
layout explicitly, pass `--gt-reference object-status`.
The legacy
`prediction.fde_m` field remains the planning-facing confidence Top-1 FDE.
`prediction.multimodal` additionally reports Top-1 ADE/FDE, minADE-K,
minFDE-K, Top-1 and best-of-K miss rates, GT-closest-path confidence,
Brier-minFDE, path-count statistics and the multi-path object rate.
Each horizon also contains `origin_aligned`: GT future positions are translated
by the matched estimate-minus-GT offset at the prediction origin. Keep absolute
metrics for end-to-end state accuracy, and use origin-aligned ADE/FDE to compare
motion models when MORAI `ObjectStatus.position` and detector bbox center use
different physical reference points. This alignment also removes the current
frame's detector/tracker translation error, so it is a conditional motion-model
diagnostic rather than a replacement production safety metric.

### ObjectStatus reference versus bbox center

`audit_bbox_reference_geometry.py` independently compares generated scenario
actor positions, SaveSensorData `Instance.txt` box centers, and online detector
boxes. Run it with the system ROS Python rather than a detector Conda Python:

```bash
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

rosrun tracking_prediction_evaluation audit_bbox_reference_geometry.py \
  --dataset-root \
  artifacts/detection3d/datasets/pointpainting_sunny13_1500_semantic_v1 \
  --raw-bag rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag \
  --output-bag \
  artifacts/prediction_evaluation/e2_highway1_pointpainting_epoch65_clustered_max/pipeline_outputs.bag \
  --output-dir \
  artifacts/prediction_evaluation/reference_geometry/highway1_epoch65
```

The current 1,500-frame capture audit found that passenger-sized vehicle
`Instance` bbox centers are `1.340 m` (median) ahead of scenario actor
positions, with only `-0.009 m` median lateral offset. This is a spatial actor
reference convention, not evidence for a fixed timestamp correction. Larger
bus/truck assets have different offsets, so a single `1.340 m` correction must
not be applied blindly to every vehicle class.

Use the metrics as follows:

- `object_status_reference_distance_m`: compatibility/debug distance to the
  MORAI actor reference point; not bbox-center MOTP.
- `origin_aligned`: motion/path-shape diagnostic that also removes genuine
  current-frame detector translation error.
- `bbox_center_absolute_distance_m`: production comparison against the
  independently calibrated bbox-center GT for supported vehicle profiles;
  inspect `unsupported_segment_count` before comparing runs.

When the output bag also contains canonical `DetectedObjects`, it additionally
associates detection and tracking independently to the same MORAI actor at the
same timestamp and reports the four yaw-flow cases: flip retained, flip
corrected, correct retained, and correct regressed. The same section now
reports detection/tracking center, signed GT-heading longitudinal/lateral,
range, motion-slice and equivalent-lag statistics. These stage metrics showed
that the EKF is center-neutral while substantially improving yaw; do not tune
EKF position noise to hide a center offset already present in detections.

The report also computes an Object_topic Map-CV oracle on the exact same
output timestamps, range gate and GT actors. This avoids an unfair global-GT
versus local-detector comparison.

```bash
/usr/bin/python3 \
  base_ws/src/tracking_prediction_evaluation/scripts/evaluate_pipeline_prediction.py \
  RAW_INPUT.bag PIPELINE_OUTPUTS.bag \
  --output artifacts/prediction_evaluation/SCENE/summary.json
```

Primary paths in the resulting report are:

```text
tracking.bbox_center_absolute_distance_m
prediction.multimodal.horizons.<sec>
prediction.multimodal.horizons.<sec>.origin_aligned
object_status_reference_evaluation
```

For production model selection, report the offline CV-stable prediction slice
as the primary metric. A sample is retained independently at each horizon only
when MORAI GT satisfies `abs(longitudinal_acceleration) <= 0.5 m/s^2` from the
prediction origin through the complete future horizon. This future-GT filter is
evaluation-only and is never available to the runtime predictor. Tracking
metrics remain evaluated over the complete recording.

```bash
/usr/bin/python3 \
  base_ws/src/tracking_prediction_evaluation/scripts/evaluate_pipeline_prediction.py \
  RAW_INPUT.bag PIPELINE_OUTPUTS.bag \
  --output artifacts/prediction_evaluation/SCENE/summary_cv_stable.json \
  --prediction-motion-slice cv-stable \
  --maximum-longitudinal-acceleration-mps2 0.5
```

Keep the all-motion result as a secondary robustness diagnostic rather than
using non-interacting spawn acceleration tails to choose the production model.

## Timestamp/reference-point audit

Do not infer a sensor timestamp offset from a GT shift sweep alone. A fixed
longitudinal difference between MORAI's actor origin and a detector bbox center
can produce the same apparent optimum. The audit keeps associations fixed,
checks the full header chain, evaluates detections in sensor coordinates, and
reports per-actor spatial/time decomposition.

```bash
/usr/bin/python3 \
  base_ws/src/tracking_prediction_evaluation/scripts/audit_timestamp_alignment.py \
  RAW_INPUT.bag PIPELINE_OUTPUTS.bag \
  --output-dir artifacts/prediction_evaluation/timestamp_alignment/SCENE
```

Read the compact `summary.json` first. The complete shift curves are stored
separately in `shift_sweep.csv`; they are intentionally not duplicated in the
JSON report.

Canonical evaluation inputs:

- Roundabout 1:
  `rosbags/raw_sensor_w_cam/sensor_cam_raw_roundabout1_roundabout_only.bag`
- Highway 1: `rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag`

The Roundabout 1 alias intentionally means the filtered bag containing only
the roundabout NPC population. The older mixed bag must not be used for new
quantitative comparisons.

The rejected `~0.11 s` temporal-alignment hypothesis, MORAI/bbox reference-point
separation, Roundabout wrong-branch
tail, PointPainting/EKF `pi` yaw flips, negative longitudinal speed and
straight off-road fallback are documented in
[`docs/e2_map_cv_failure_analysis.md`](docs/e2_map_cv_failure_analysis.md).

### v60 centerline-vector audit

The map audit compares direction-aware MGeo samples and observed NPC paths
against Lanelet2 v60. It reports the full global correction vector rather than
separate absolute X/Y errors, so an arbitrary-direction translation bias is
preserved.

```bash
/usr/bin/python3 \
  base_ws/src/tracking_prediction_evaluation/scripts/audit_lanelet_centerline_bias.py \
  rosbags/raw_sensor_w_cam/sensor_cam_raw_roundabout1_roundabout_only.bag \
  rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag \
  --output /tmp/lanelet_centerline_bias.json
```

The historical v23 transform left a bag-region MGeo-to-Lanelet p50 residual of
`0.0946 m` and required a mean correction vector of
`(-0.0856, +0.0005) m`. The v60 runtime-origin transform reduces these to
`0.0025 m` and `(+0.0028, -0.0003) m`. Production therefore uses
`configs/tf/morai_lanelet_static_tf.yaml`; reproduce already-recorded legacy
map-frame outputs with
`configs/tf/morai_lanelet_static_tf_v23_refined_legacy.yaml`.

### rolling-LiDAR timing audit

MORAI `/sensor/lidar1` contains a per-point `time` field, but PointPainting
currently consumes only XYZI and stamps every box with the scan header time.
The following audit associates raw points inside each detected vehicle box and
compares header-time geometry with the point acquisition-time diagnostic.

```bash
/usr/bin/python3 \
  base_ws/src/tracking_prediction_evaluation/scripts/audit_lidar_detection_timing.py \
  RAW_INPUT.bag PIPELINE_OUTPUTS.bag \
  --output /tmp/lidar_detection_timing.json
```

Acquisition-aligned distance is not a production metric: it compares both GT
and the Ego transform at the earlier point time. A production correction must
then propagate the object state to the common output timestamp. This audit is
intended to distinguish rolling-scan geometry from a false global timestamp
shift before changing runtime behavior.

## E2 pipeline output RViz

`pipeline_prediction_rviz.launch` displays the raw cameras and LiDAR together
with recorded detections, tracks and every predicted path. The confidence
Top-1 line retains its class color; lower-ranked paths are orange. Every path
uses the same alpha `0.95` and `0.08 m` line width, and its final point is
labeled with rank and confidence.

Start the launch and play the raw and output bags together. Always use
`--pause` so localization, map and visualization subscribers are ready before
the first sample.

```bash
roslaunch tracking_prediction_evaluation pipeline_prediction_rviz.launch

rosbag play --clock --pause \
  rosbags/raw_sensor_w_cam/sensor_cam_raw_roundabout1_roundabout_only.bag \
  artifacts/prediction_evaluation/e2_roundabout1_pointpainting_epoch65_clustered_max/pipeline_outputs.bag
```

For Highway 1, keep the launch running and use:

```bash
rosbag play --clock --pause \
  rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag \
  artifacts/prediction_evaluation/e2_highway1_pointpainting_epoch65_clustered_max/pipeline_outputs.bag
```
