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
