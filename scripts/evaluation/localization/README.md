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
