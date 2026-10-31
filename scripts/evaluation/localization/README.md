# Current localization evaluation tools

Read [CURRENT](../../../docs/localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md),
[BAG_CATALOG](../../../docs/localization/BAG_CATALOG.md) and
[CURRENT_EVIDENCE](../../../docs/localization/CURRENT_EVIDENCE.md) before evaluating.
This cleanup changed no estimator, evaluator or validator implementation.

- `analyze_localization_stability_bag.py`: recorded Global/Smooth pose, rate,
  continuity, controller chatter, Ego oracle quality and co-timed TF closure.
- `analyze_gps_shadow_validator.py` and its test: phase-aware recorded GPS-shadow
  error/recovery/availability analysis, bounded interpolation and teleport exclusion.
  Sibling events select the driving window; replayed commands are not a vehicle rerun.
- `eval_localization_state_with_rviz.py`: common Odometry pose/twist/RViz evaluator.
  `eval_localization_base_link_with_rviz.py` and `eval_localization_pose_with_rviz.py`
  remain because it imports them. Provider-specific code is not imported here.
- `run_validator_intervention.py` and its test remain unchanged at their existing
  paths because they integrate with the active validator. This development helper's
  Park/ScenarioLoad/known-prior interventions are not the production reset contract
  or permission to operate a live validator.

The stability analyzer defaults to MR oracle heading offset `-1.347deg`.
MRP and uncorrected direct Ego GT require `--heading-offset-deg 0`.
Recover alias/gt_source/runtime parameters; never infer heading from a filename or PC,
and never apply correction a second time to the estimator state.

## One-shot GNSS delay calibration

After `ajou`, run `gnss_delay_record --duration 90` beside an already-running
manual localization/od_gt/planning/controller stack. Only small raw/reference
topics are recorded; Ctrl-C finalizes this recorder and analyzes, **not** the car.

For the complete real-validator workflow, keep only roscore and connected MORAI,
then run `./planning_validator --gnss-delay-calibration` (`gnss_delay_validator`
alias). It owns actual localization/od_gt/planner/controller driving, recording,
stop/P audit, then calls the same `analyze_gnss_delay.py`. Existing manual stacks
are refused, never killed. Default two empty-route runs use a fresh derived scene
without GPS shadows; original scenarios, sensors and mission actors are unchanged.
`--keep-gps-shadow` is for separate recovery trials. `--check`/`--dry-run` do not
contact ROS, create files or move the car.

The analyzer reads raw `/sensor/gps`, offline `/Ego_topic`, recorded map/morai_map
TF and motion context. It reports `result.json`/`report.txt` with an **absolute**
`recommendation_ms`, not an increment to current compensation. Fixed-position bias,
moving sample count/time, speed variation, interpolation gaps and fit stability
are checked; insufficient evidence produces no guessed number. No parameter is
automatically changed. [Current contract and usage](../../../docs/localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md#원큐-측정-녹화와-validator).
For the RViz evaluator, `--alignment-yaw-mode none` preserves absolute estimator yaw;
historical `estimate` alignment can hide a constant yaw bias.

[legacy/](legacy/) contains completed ablations, old Smooth/output counterfactuals,
synthetic outage/relocation runners and the minimal old evaluator. They are frozen
source snapshots, not current executable entrypoints. Use only for an explicit
historical reproduction; their old paths, source imports and defaults are not updated.
Current soft/Smooth values do not retroactively relabel old experiments.
