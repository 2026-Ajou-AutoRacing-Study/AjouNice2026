---
source_type: rendered_autoware_docs
site: autoware_universe
title: Autoware Trajectory Optimizer
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_optimizer/
rendered_html: rendered/autoware_universe_html/index-175.html
generated: 2026-06-24T14:59:32.806077+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Autoware Trajectory Optimizer

The `autoware_trajectory_optimizer` package generates smooth and feasible trajectories for autonomous vehicles using a plugin-based optimization pipeline. It takes candidate trajectories as input and applies a sequence of optimization plugins to produce smooth, drivable trajectories with proper velocity and acceleration profiles.

## Features

- **Plugin-based architecture** - Modular optimization pipeline where each step is a separate plugin
- **Multiple smoothing methods**:
  - Elastic Band (EB) smoother for path optimization
  - Akima spline interpolation for smooth path interpolation
  - QP-based smoother with quadratic programming for path smoothing with jerk constraints
- **Velocity optimization** - Jerk-filtered velocity smoothing from `autoware_velocity_smoother`
- **Temporal MPT optimization** - acados MPC (8 s / 0.1 s horizon) on a kinematic bicycle with x, y, yaw, and speed states
- **Trajectory validation** - Removes invalid points and fixes trajectory orientation
- **Backward trajectory extension** - Extends trajectory using past ego states
- **Dynamic parameter reconfiguration** - Runtime parameter updates supported

## Architecture

The package uses a pluginlib-based architecture where optimization plugins are dynamically loaded at startup. Each plugin inherits from `TrajectoryOptimizerPluginBase` and is loaded via the ROS 2 pluginlib system.

### Plugin Loading and Execution

Plugins are loaded based on the `plugin_names` parameter, which defines both which plugins to load and their execution order:

```
plugin_names:
  - "autoware::trajectory_optimizer::plugin::TrajectoryPointFixer"
  - "autoware::trajectory_optimizer::plugin::TrajectoryQPSmoother"
  - "autoware::trajectory_optimizer::plugin::TrajectoryEBSmootherOptimizer"
  - "autoware::trajectory_optimizer::plugin::TrajectorySplineSmoother"
  - "autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer"
  - "autoware::trajectory_optimizer::plugin::TrajectoryExtender"
  - "autoware::trajectory_optimizer::plugin::TrajectoryPointFixer"
```

### Available Plugins

1. **TrajectoryPointFixer** - Removes invalid/repeated points and fixes trajectory direction
2. **TrajectoryQPSmoother** - QP-based path smoothing with jerk constraints
3. **TrajectoryEBSmootherOptimizer** - Elastic Band path smoothing
4. **TrajectorySplineSmoother** - Akima spline interpolation
5. **TrajectoryMPTOptimizer** - Model predictive trajectory optimization with adaptive corridor bounds. Uses bicycle kinematics model for trajectory refinement. Disabled by default (experimental). See [docs/mpt\_optimizer.md](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_optimizer/docs/mpt_optimizer/) for details.
6. **TrajectoryTemporalMPTOptimizer** - Temporal acados MPC path tracking on a time-ordered reference (8.0 s horizon, 0.1 s discretization). Must be listed in `plugin_names` and enabled via `use_temporal_mpt_optimizer`. See [docs/temporal\_mpt\_optimizer.md](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_optimizer/docs/temporal_mpt_optimizer/) for details.
7. **TrajectoryVelocityOptimizer** - Velocity profile optimization with lateral acceleration limits
8. **TrajectoryExtender** - Extends trajectory backward using past ego states
9. **TrajectoryKinematicFeasibilityEnforcer** - Enforces Ackermann steering and yaw rate constraints

Each plugin can be enabled/disabled at runtime via activation flags (e.g., `use_qp_smoother`) and manages its own configuration independently.

### ⚠️ Important: Plugin Ordering Constraints

**The order of plugin execution is critical and must be carefully maintained:**

- **QP Smoother must run before EB/Akima smoothers**: The QP solver relies on constant time intervals (Δt) between trajectory points (default: 0.1s). Both Elastic Band and Akima spline smoothers resample trajectories without preserving the time domain structure, which breaks the QP solver's assumptions. Therefore, when using multiple smoothers together, the QP smoother must execute first.

- **Trajectory Extender positioning**: The trajectory extender has known discontinuity issues when placed early in the pipeline. It negatively affects the QP solver results and introduces artifacts. For this reason, it has been moved to near the end of the pipeline and is **disabled by default** (`use_trajectory_extender: false`). Fixing the extender's discontinuity issues is future work.

- **Temporal MPT vs spatial MPT**: `TrajectoryTemporalMPTOptimizer` is independent of `TrajectoryMPTOptimizer` (different solver, no corridor bounds). It can replace the entire post-`TrajectoryPointFixer` plugin chain (kinematic enforcer, QP/EB/spline smoothers, velocity optimizer, spatial MPT) with a single MPC step; see [docs/temporal\_mpt\_optimizer.md](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_optimizer/docs/temporal_mpt_optimizer/). It overwrites at most the first 81 points (8.0 s at 0.1 s spacing).

## Design Specification

For a detailed description of the optimizer's core assumptions, the constant-dt contract,
how velocity and acceleration are derived from positions, plugin pipeline specification, and
known limitations, see [docs/trajectory\_optimizer\_specification.md](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_optimizer/docs/trajectory_optimizer_specification/).

## QP Smoother

The QP smoother uses quadratic programming (OSQP solver) to optimize trajectory paths with advanced features:

- **Objective**: Minimizes path curvature while maintaining fidelity to the original trajectory
- **Decision variables**: Path positions (x, y) for each trajectory point
- **Constraints**: Fixed initial position (optionally fixed last position)
- **Velocity-based fidelity**: Automatically reduces fidelity weight at low speeds for aggressive smoothing of noise
- **Post-processing**: Recalculates velocities, accelerations, and orientations from smoothed positions

**For detailed documentation**, see [docs/qp\_smoother.md](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_optimizer/docs/qp_smoother/) which covers:

- Mathematical formulation
- Velocity-based fidelity weighting (sigmoid function)
- Parameter tuning guidelines
- Usage examples
- Performance characteristics

## Dependencies

- [acados](https://github.com/acados/acados) - Required to build the temporal MPT solver (`acados_interface_temporal`)
- `autoware_motion_utils` - Trajectory manipulation utilities
- `autoware_osqp_interface` - QP solver interface for QP smoother
- `autoware_path_smoother` - Elastic Band smoother
- `autoware_velocity_smoother` - Velocity smoothing algorithms
- `autoware_utils` - Common utilities (geometry, ROS helpers)
- `autoware_vehicle_info_utils` - Vehicle information

## Parameters

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| plugin\_names | array | List of plugin class names to load in execution order. Plugins will be loaded dynamically at startup and executed in the order specified. | ['autoware::trajectory\_optimizer::plugin::TrajectoryPointFixer', 'autoware::trajectory\_optimizer::plugin::TrajectoryKinematicFeasibilityEnforcer', 'autoware::trajectory\_optimizer::plugin::TrajectoryQPSmoother', 'autoware::trajectory\_optimizer::plugin::TrajectoryEBSmootherOptimizer', 'autoware::trajectory\_optimizer::plugin::TrajectorySplineSmoother', 'autoware::trajectory\_optimizer::plugin::TrajectoryVelocityOptimizer', 'autoware::trajectory\_optimizer::plugin::TrajectoryExtender', 'autoware::trajectory\_optimizer::plugin::TrajectoryPointFixer'] | N/A |
| use\_akima\_spline\_interpolation | boolean | Enable Akima spline interpolation for trajectory smoothing | False | N/A |
| use\_eb\_smoother | boolean | Enable Elastic Band smoother for trajectory smoothing | False | N/A |
| use\_qp\_smoother | boolean | Enable QP-based trajectory smoothing with explicit jerk constraints | True | N/A |
| use\_trajectory\_point\_fixer | boolean | Remove repeated or invalid points, or points that go against the general trajectory direction | True | N/A |
| use\_trajectory\_extender | boolean | Enable trajectory extender to extend trajectory backward using ego pose history | False | N/A |
| use\_velocity\_optimizer | boolean | Enable velocity profile optimization with jerk filtering and constraint enforcement | True | N/A |
| use\_kinematic\_feasibility\_enforcer | boolean | Enable kinematic feasibility enforcer plugin (Ackermann + yaw rate constraints) | False | N/A |
| use\_mpt\_optimizer | boolean | Enable MPT (Model Predictive Trajectory) optimizer plugin for geometry refinement with adaptive bounds | False | N/A |
| trajectory\_point\_fixer.resample\_close\_points | boolean | Enable resampling of close proximity points | True | N/A |
| trajectory\_point\_fixer.min\_dist\_to\_remove\_m | float | Minimum distance threshold for removing close proximity points [m] | 0.01 | ≥0 |
| trajectory\_point\_fixer.min\_dist\_to\_resample\_m | float | Minimum distance threshold for merging close proximity points [m] | 0.05 | ≥0 |
| trajectory\_point\_fixer.stop\_detection\_velocity\_threshold\_mps | float | Velocity threshold used by the stop-detection logic: points whose speed falls below this value while decelerating are treated as stop candidates by both the cluster-based and velocity-profile-based detectors [m/s] | 0.3 | ≥0 |
| trajectory\_kinematic\_feasibility.max\_yaw\_rate\_rad\_s | float | Maximum yaw rate [rad/s]. Default: 0.7 rad/s (~40 deg/s) aligns with MPC controller. Typical range: 0.5-1.0 for autonomous vehicles. | 0.7 | ≥0.1 ≤2.0 |
| trajectory\_kinematic\_feasibility.time\_step\_s | float | Time step for kinematic feasibility enforcement [s]. Default: 0.1 s. | 0.1 | ≥0.01 ≤1.0 |
| trajectory\_mpt\_optimizer.corridor\_width\_m | float | Base corridor width for adaptive bounds generation [m]. Defines the lateral distance from trajectory centerline to left/right bounds. | 3.5 | ≥2.0 ≤10.0 |
| trajectory\_mpt\_optimizer.enable\_adaptive\_width | boolean | Enable adaptive corridor width based on curvature and velocity. When true, corridor widens in curves and at low speeds. | True | N/A |
| trajectory\_mpt\_optimizer.curvature\_width\_factor | float | Factor for curvature-based corridor widening [m/rad]. Higher values provide more space in curves. | 0.5 | ≥0.0 ≤2.0 |
| trajectory\_mpt\_optimizer.velocity\_width\_factor | float | Factor for velocity-based corridor widening [m]. Higher values provide more maneuvering room at low speeds. | 0.3 | ≥0.0 ≤2.0 |
| trajectory\_mpt\_optimizer.min\_clearance\_m | float | Minimum clearance from vehicle edges to corridor bounds [m]. Ensures minimum safe distance. | 0.5 | ≥0.0 ≤2.0 |
| trajectory\_mpt\_optimizer.reset\_previous\_data\_each\_iteration | boolean | Reset MPT's previous optimization data each cycle. Should be true for diffusion planner (new trajectories each cycle), false for incremental planners. | True | N/A |
| trajectory\_mpt\_optimizer.enable\_debug\_info | boolean | Enable debug visualization markers for bounds and reference trajectory. | False | N/A |
| trajectory\_mpt\_optimizer.output\_delta\_arc\_length\_m | float | Spacing between trajectory points in MPT optimization [m]. Smaller values provide finer resolution but increase computation. | 1.0 | ≥0.1 ≤5.0 |
| trajectory\_mpt\_optimizer.output\_backward\_traj\_length\_m | float | Length of trajectory extension backward from ego pose [m]. | 5.0 | ≥0.0 ≤20.0 |
| trajectory\_mpt\_optimizer.ego\_nearest\_dist\_threshold\_m | float | Distance threshold for finding nearest trajectory point to ego [m]. | 3.0 | ≥0.5 ≤10.0 |
| trajectory\_mpt\_optimizer.ego\_nearest\_yaw\_threshold\_deg | float | Yaw threshold for finding nearest trajectory point to ego [degrees]. | 45.0 | ≥10.0 ≤90.0 |
| trajectory\_mpt\_optimizer.acceleration\_moving\_average\_window | integer | Moving average window size for acceleration smoothing. Larger values produce smoother acceleration but increase lag. Window of 1 = no smoothing. | 5 | ≥1 ≤20 |
| trajectory\_extender.nearest\_dist\_threshold\_m | float | Distance threshold for trajectory matching [m] | 1.5 | ≥0 |
| trajectory\_extender.nearest\_yaw\_threshold\_deg | float | Yaw threshold for trajectory matching [deg] | 60.0 | ≥0 |
| trajectory\_extender.backward\_trajectory\_extension\_m | float | Length to extend trajectory backward using ego history [m] | 5.0 | ≥0 |
| trajectory\_spline\_smoother.interpolation\_resolution\_m | float | Interpolation resolution for Akima spline [m] | 0.5 | >0 |
| trajectory\_spline\_smoother.max\_distance\_discrepancy\_m | float | Maximum position deviation allowed for orientation copying [m] | 5.0 | ≥0 |
| trajectory\_spline\_smoother.preserve\_input\_trajectory\_orientation | boolean | Copy orientations from input trajectory to spline output | True | N/A |
| trajectory\_qp\_smoother.weight\_smoothness | float | Weight for path curvature minimization (geometric smoothness) [dimensionless] | 10.0 | ≥0 |
| trajectory\_qp\_smoother.weight\_fidelity | float | Weight for path fidelity (staying close to original path) [dimensionless] | 1.0 | ≥0 |
| trajectory\_qp\_smoother.time\_step\_s | float | Fixed time step for velocity/acceleration calculations [s] | 0.1 | >0 |
| trajectory\_qp\_smoother.osqp\_eps\_abs | float | OSQP absolute convergence tolerance | 0.0001 | >0 |
| trajectory\_qp\_smoother.osqp\_eps\_rel | float | OSQP relative convergence tolerance | 0.0001 | >0 |
| trajectory\_qp\_smoother.osqp\_max\_iter | integer | OSQP maximum solver iterations | 100 | ≥1 |
| trajectory\_qp\_smoother.osqp\_verbose | boolean | Enable OSQP verbose solver output | False | N/A |
| trajectory\_qp\_smoother.preserve\_input\_trajectory\_orientation | boolean | Copy orientations from input trajectory to smoothed output | True | N/A |
| trajectory\_qp\_smoother.max\_distance\_for\_orientation\_m | float | Max distance for nearest neighbor matching when copying orientations [m] | 5.0 | ≥0 |
| trajectory\_qp\_smoother.use\_velocity\_based\_fidelity | boolean | Enable velocity-dependent fidelity weighting (lower fidelity at low speeds) | True | N/A |
| trajectory\_qp\_smoother.velocity\_threshold\_mps | float | Velocity threshold at sigmoid midpoint for velocity-based fidelity [m/s] | 0.3 | ≥0 |
| trajectory\_qp\_smoother.sigmoid\_sharpness | float | Sigmoid steepness for velocity-based fidelity transition (higher = sharper) | 50.0 | ≥1 |
| trajectory\_qp\_smoother.min\_fidelity\_weight | float | Minimum fidelity weight at very low speeds [dimensionless] | 0.01 | ≥0 |
| trajectory\_qp\_smoother.max\_fidelity\_weight | float | Maximum fidelity weight at high speeds [dimensionless] | 1.0 | ≥0 |
| trajectory\_qp\_smoother.num\_constrained\_points\_start | integer | Number of points from the start of the trajectory to constrain as hard constraints (preserves initial state) | 3 | ≥0 |
| trajectory\_qp\_smoother.num\_constrained\_points\_end | integer | Number of points from the end of the trajectory to constrain as hard constraints | 3 | ≥0 |
| trajectory\_velocity\_optimizer.nearest\_dist\_threshold\_m | float | Distance threshold for trajectory matching [m] | 1.5 | ≥0 |
| trajectory\_velocity\_optimizer.nearest\_yaw\_threshold\_deg | float | Yaw threshold for trajectory matching [deg] | 60.0 | ≥0 |
| trajectory\_velocity\_optimizer.target\_pull\_out\_speed\_mps | float | Target speed during pull-out maneuver [m/s] | 1.0 | ≥0 |
| trajectory\_velocity\_optimizer.target\_pull\_out\_acc\_mps2 | float | Target acceleration during pull-out maneuver [m/s²] | 1.0 | ≥0 |
| trajectory\_velocity\_optimizer.max\_lateral\_accel\_mps2 | float | Maximum lateral acceleration [m/s²] | 1.5 | ≥0 |
| trajectory\_velocity\_optimizer.set\_engage\_speed | boolean | Set minimum speed during engage | False | N/A |
| trajectory\_velocity\_optimizer.limit\_speed | boolean | Enable speed limiting | True | N/A |
| trajectory\_velocity\_optimizer.limit\_lateral\_acceleration | boolean | Enable lateral acceleration limiting | False | N/A |
| trajectory\_velocity\_optimizer.smooth\_velocities | boolean | Enable jerk-filtered velocity smoothing | False | N/A |
| common.output\_delta\_arc\_length | float | Delta arc length for output trajectory [m] | 0.5 | >0 |
| common.output\_backward\_traj\_length | float | Backward length for backward trajectory from base\_link [m] | 5.0 | ≥0 |
| option.enable\_warm\_start | boolean | Enable warm start for optimization | True | N/A |
| option.enable\_optimization\_validation | boolean | Enable optimization validation | False | N/A |
| common.num\_points | integer | Number of points for optimization | 100 | ≥1 |
| common.delta\_arc\_length | float | Delta arc length for optimization [m] | 1.0 | >0 |
| weight.smooth\_weight | float | Weight for smoothness constraint | 1.0 | ≥0 |
| weight.lat\_error\_weight | float | Weight for lateral error constraint | 0.001 | ≥0 |
| elastic\_band\_params.ego\_nearest\_dist\_threshold | float | Distance threshold for ego nearest search [m] | 3.0 | ≥0 |
| elastic\_band\_params.ego\_nearest\_yaw\_threshold | float | Yaw threshold for ego nearest search [rad] | 1.046 | ≥0 |

Parameters can be set via YAML configuration files in the `config/` directory.

### Parameter Types

1. **Plugin Loading** (`plugin_names`) - Array of plugin class names determining load order and execution sequence
2. **Activation Flags** - Boolean flags for runtime enable/disable (e.g., `use_qp_smoother`, `use_temporal_mpt_optimizer`)
3. **Plugin-Specific Parameters** - Namespaced parameters for each plugin (e.g., `trajectory_qp_smoother.weight_smoothness`)

### Configuring Plugin Order

To change plugin execution order, modify the `plugin_names` array in `config/trajectory_optimizer.param.yaml`:

```yaml

# Example: Run spline smoother before velocity optimizer

plugin\_names:
- "autoware::trajectory\_optimizer::plugin::TrajectoryPointFixer"
- "autoware::trajectory\_optimizer::plugin::TrajectorySplineSmoother"
- "autoware::trajectory\_optimizer::plugin::TrajectoryVelocityOptimizer"
- "autoware::trajectory\_optimizer::plugin::TrajectoryPointFixer"
```

#### CRITICAL: QP Smoother Ordering Constraint

The `TrajectoryQPSmoother` plugin **MUST run before** any plugins that resample or modify trajectory structure:

- `TrajectorySplineSmoother` (Akima spline - resamples trajectory)
- `TrajectoryEBSmootherOptimizer` (Elastic Band - resamples trajectory)
- `TrajectoryVelocityOptimizer` (velocity smoothing with resampling)
- `TrajectoryExtender` (adds/modifies points at trajectory start)

The QP solver requires constant time intervals (Δt = 0.1s) between points. These plugins modify the time domain structure or add points, breaking the QP solver assumptions. If you need QP smoothing, it must appear first in the pipeline after `TrajectoryPointFixer`.

Note: Plugin order changes require node restart. Runtime enable/disable is controlled by activation flags.
