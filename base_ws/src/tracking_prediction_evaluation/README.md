# Tracking prediction evaluation

This package contains the replacement evaluation pipeline for tracking and
prediction. The first implemented layer is E0: a causal oracle benchmark that
feeds MORAI GT history directly to kinematic predictors.

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
