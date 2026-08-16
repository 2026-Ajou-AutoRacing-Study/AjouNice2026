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
center/speed/yaw error, and 1/2/3-second prediction FDE plus coverage.

The report also computes an Object_topic Map-CV oracle on the exact same
output timestamps, range gate and GT actors. This avoids an unfair global-GT
versus local-detector comparison.

```bash
/usr/bin/python3 \
  base_ws/src/tracking_prediction_evaluation/scripts/evaluate_pipeline_prediction.py \
  RAW_INPUT.bag PIPELINE_OUTPUTS.bag \
  --output artifacts/prediction_evaluation/SCENE/summary.json
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

Canonical evaluation inputs:

- Roundabout 1:
  `rosbags/raw_sensor_w_cam/sensor_cam_raw_roundabout1_roundabout_only.bag`
- Highway 1: `rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag`

The Roundabout 1 alias intentionally means the filtered bag containing only
the roundabout NPC population. The older mixed bag must not be used for new
quantitative comparisons.

The measured `~0.11 s` temporal-alignment hypothesis, Roundabout wrong-branch
tail, PointPainting/EKF `pi` yaw flips, negative longitudinal speed and
straight off-road fallback are documented in
[`docs/e2_map_cv_failure_analysis.md`](docs/e2_map_cv_failure_analysis.md).
