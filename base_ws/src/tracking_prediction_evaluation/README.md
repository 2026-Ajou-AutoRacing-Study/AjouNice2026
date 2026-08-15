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

python3 base_ws/src/tracking_prediction_evaluation/scripts/evaluate_oracle_prediction.py \
  rosbags/raw_sensor/roundabout1.bag \
  --output-dir /tmp/ajounice_e0_roundabout
```

Read `summary.json` first. `worst_samples.jsonl` contains only the configured
number of largest-FDE samples and is intended for targeted debugging.

The validated initial benchmark and discarded legacy assumptions are recorded
in [`docs/e0_oracle_baseline.md`](docs/e0_oracle_baseline.md).

The former custom-message `tracking_prediction` package and its evaluation
scripts were removed from the current tree after the audit. They remain in Git
history only. Their reported values must not be described as end-to-end results
of the current standardized object pipeline.
