# Localization evaluation scripts

This directory contains estimator-agnostic evaluation tools. The primary
contract is a `nav_msgs/Odometry` state compared with MORAI `/Ego_topic`.

- `eval_localization_state_with_rviz.py`: pose, twist, yaw-rate, interval, CSV,
  text report, and RViz evaluation for `/localization/kinematic_state`
- `eval_localization_pose_with_rviz.py`: common pose evaluation and RViz helpers
- `eval_localization_base_link_with_rviz.py`: body-to-`base_link` conversion
  wrapper for body-referenced odometry
- `eval_localization_vs_ego.py`: legacy minimal pose evaluator

Yaw alignment modes:

- `--alignment-yaw-mode estimate`: estimate one constant yaw rotation at the
  selected reference pose; this is the historical default for trajectories
  whose local starting frame is arbitrary.
- `--alignment-yaw-mode none`: preserve the estimator's absolute yaw and align
  translation only. Use this to measure a known global heading bias.

The evaluation script is shared by all localization providers. Provider
specific code must not be imported here.

## Heading provenance

`analyze_localization_stability_bag.py` compares the estimator with raw MORAI
`/Ego_topic`. Its default `--heading-offset-deg -1.347` is only correct for an
MR/competition estimator run that applied the same correction.

- MR/competition `localization`: use `-1.347`.
- MRP `localization_mrp`: use `0`.
- A direct MORAI GT provider that did not apply the correction: use `0`.
- Never apply the offset again to `/localization/kinematic_state` itself.

Do not infer this value from the bag filename or recording computer. Recover
the launcher, alias, `gt_source`, and runtime parameter provenance first. See
`docs/localization/260924_LOCALIZATION_EVALUATION_BAG_CATALOG.md`.

## Smooth controller-state pre-screen

`analyze_smooth_control_state_bag.py` combines recorded Global and Local EKF
outputs while smoothing only their `map -> odom` alignment. This is a
counterfactual offline pre-screen: it does not re-run the controller and cannot
approve a production change.

```bash
python3 scripts/evaluation/localization/analyze_smooth_control_state_bag.py \
  rosbags/eval/localization/local_mr_stability/<bag>.bag \
  --heading-offset-deg -1.347 \
  --output /tmp/smooth_control_state.json
```

`run_controller_state_open_loop_ab.sh` replays one recorded trajectory twice
and changes only the state consumed by both trajectory-follower nodes. This is
stronger than output-only smoothing but still has no vehicle feedback.

```bash
scripts/evaluation/localization/run_controller_state_open_loop_ab.sh INPUT.bag
```

Set `LOCALIZATION_CONTROL_AB_TAUS='0.2 0.5 1.0'` for a tau sweep. Set
`LOCALIZATION_CONTROL_AB_INCLUDE_GLOBAL=false` when a previously validated
Global result is being reused.

## Competition yaw-rate diagnostic

The native UDP packet includes angular velocity, but the legacy
`EgoVehicleStatus` type does not. New recordings preserve it on
`/competition/raw_twist`. Compare that signal with IMU yaw-rate and the
derivative of Competition heading using:

```bash
python3 scripts/evaluation/localization/analyze_competition_yaw_rate.py \
  <new_competition_bag.bag> \
  --output /tmp/competition_yaw_rate.json
```

Old bags without `/competition/raw_twist` cannot answer this question. Do not
fuse Competition and IMU yaw-rate as independent sensors merely because both
topics exist.

## Motion-input ablation

`run_motion_input_ablation.sh` re-runs the current filter with IMU yaw-rate and
ax enabled/disabled independently. It defaults to 1x replay because faster
replay can reduce the final-state rate and invalidate the comparison.

```bash
scripts/evaluation/localization/run_motion_input_ablation.sh INPUT.bag
```
