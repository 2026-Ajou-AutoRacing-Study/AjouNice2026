# Archived localization experiments

These original files preserve completed/rejected/held experiments byte-for-byte.
They are not current production tools. Current entrypoints live in [../README.md](../README.md).

- Q/R covariance sweep, motion-input and Competition yaw-rate ablations.
- GNSS lattice/input filtering and old origin-based Smooth counterfactuals.
- Recorded-controller open-loop A/B, synthetic outage/reacquisition and teleport replay.
- Historical minimal pose evaluator.

The files deliberately retain original hardcoded paths and repository-root depth
assumptions. Moving an original is not a claim that it runs from this new location.
For an explicitly requested reproduction, use its recorded source revision in an
isolated worktree/environment and retain the original bag/snapshot/provenance.
Do not restore old paths/defaults into the live workspace or execute these runners
against existing ROS/MORAI processes. Recorded trajectory replay and synthetic
teleports are not new vehicle closed-loop evidence.
