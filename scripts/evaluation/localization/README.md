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

For a new bag that already records `/localization/control_kinematic_state`,
`analyze_localization_stability_bag.py` reports Global pose, controller-state
pose, and controller chatter in one result. Old bags remain compatible and
report `control_state.available=false`.

The analyzer also emits `ego_oracle_quality`. MORAI Ego pose has a measurable
per-frame longitudinal timing residual even though its full-drive position and
velocity integral agree. Treat longitudinal frame jump as diagnostic only when
it is comparable to that residual; lateral frame jump remains the primary
pose-continuity metric.

When a bag records Local odometry, final kinematic state, and `/tf`, the same
analyzer emits `tf_contract`. It checks that recorded `map -> odom` composed
with co-timed `odom -> base_link` closes onto the final map-frame state.

## Repeatable GNSS outage/reacquisition replay

The outage runner creates a small derived input bag containing only Ego,
Competition, GPS, IMU, and static TF, removes GPS in a selected interval, then
re-runs the production estimator while recording both Global and experimental
smooth controller states. Use a moving interval and keep the generated
`metadata.txt` with every result.

```bash
LOCALIZATION_OUTAGE_OUTPUT_DIR=/tmp/localization_outage \
scripts/evaluation/localization/run_gnss_outage_reacquisition.sh \
  rosbags/raw_sensor_w_cam/competition_full_run/\
260822_005713_competition_full_route_raw_hz_run01.bag \
  15 8 turning_8s
```

기본은 기존 smooth control-state replay다. Quantized-GNSS 후보는 다음처럼 같은
runner에서 선택한다.

```bash
LOCALIZATION_OUTAGE_CONTROL_MODE=quantized \
scripts/evaluation/localization/run_gnss_outage_reacquisition.sh \
  INPUT.bag 15 8 quantized_turning_8s
```

두 모드 모두 1×가 유효 기준이다. 배속 replay에서 output state가 50 Hz를 유지하지
못하면 결과를 폐기한다.

## Repeatable checkpoint-relocation replay

The relocation runner keeps the same raw-input contract, injects a 20 m
northward GPS displacement halfway through a 25 s slice, and runs the active
relocation guard with the quantization-aware controller state. It is a
mechanism test, not approval to enable the guard in production.

```bash
scripts/evaluation/localization/run_checkpoint_relocation_replay.sh INPUT.bag
```

The generated `analysis.json` records reset count, GNSS-to-Global recovery,
Global-to-control agreement, reset jump, and both output rates. Keep replay at
1× and retain `metadata.txt` with the result.

## GNSS quantization lattice audit

`analyze_gnss_quantization_lattice.py` measures the smallest non-zero raw
latitude/longitude step and projects both cell axes with the same WGS84→UTM
52N conversion used by localization.

```bash
python3 scripts/evaluation/localization/analyze_gnss_quantization_lattice.py \
  BAG1 BAG2 BAG3 --output /tmp/gnss_quantization_lattice.json
```

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

Add the quantization-aware state with:

```bash
LOCALIZATION_CONTROL_AB_INCLUDE_QUANTIZED=true \
scripts/evaluation/localization/run_controller_state_open_loop_ab.sh INPUT.bag
```

`LOCALIZATION_CONTROL_AB_INCLUDE_SMOOTH=false` can isolate the quantized run.
The runner fails if its state publisher exits early and writes provenance to
each profile's `metadata.txt`.

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

Old bags without `/competition/raw_twist` cannot answer this question. The raw
angular value is clockwise degrees/s, not ROS rad/s; the analyzer and
experimental replacement adapter apply `-pi/180`. Do not fuse Competition and
IMU yaw-rate as independent sensors because both originate from the same
simulator state.

## Motion-input ablation

`run_motion_input_ablation.sh` re-runs the current filter with IMU yaw-rate and
ax enabled/disabled independently. It defaults to 1x replay because faster
replay can reduce the final-state rate and invalidate the comparison.

```bash
scripts/evaluation/localization/run_motion_input_ablation.sh INPUT.bag
```
