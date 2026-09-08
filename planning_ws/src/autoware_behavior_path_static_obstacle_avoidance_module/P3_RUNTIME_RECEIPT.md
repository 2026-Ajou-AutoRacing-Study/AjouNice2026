# P3 static obstacle avoidance receipt

Status: candidate-only evidence; no official AW149 state was promoted.

On 2026-09-05, the r025 BPP-common host artifact
`build-host-abi-r028-common-r025` was exercised under a scoped foreground ROS
master with the matching r025 BPP runtime provider.

Results:

- `test_static_obstacle_utils`: 17/17 passed (one separately disabled test is
  not counted as a pass).
- `test_static_obstacle_plugin_discovery`: passed; declared BPP plugin created
  `StaticObstacleAvoidanceModuleManager`.
- `test_static_obstacle_manager_init`: passed after
  `rosparam load config/static_obstacle_avoidance.ros1.yaml` supplied the full
  rendered ROS1 parameter tree, including its structured velocity map.

The original manager-init failure was a fixture error: its C++ process supplied
no YAML despite claiming a complete parameter surface.  The source policy
values remain the staged `per_shift_line`, `best_effort`, `best_effort`, and
`none`; no policy was weakened.

Boundary: this is manager initialization and plugin evidence, not an obstacle
avoidance scene-behavior equivalence or official promotion receipt.
