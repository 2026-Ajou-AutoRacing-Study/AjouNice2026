---
source_type: rendered_autoware_docs
site: autoware_universe
title: Path optimizer
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/
rendered_html: rendered/autoware_universe_html/index-162.html
generated: 2026-06-24T14:59:32.069483+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Path optimizer

## Purpose

This package generates a trajectory that is kinematically-feasible to drive and collision-free based on the input path, drivable area.
Only position and orientation of trajectory are updated in this module, and velocity is just taken over from the one in the input path.

## Feature

This package is able to

- make the trajectory inside the drivable area as much as possible
  - NOTE: Static obstacles to avoid can be removed from the drivable area.
- insert stop point before the planned footprint will be outside the drivable area

Note that the velocity is just taken over from the input path.

## Inputs / Outputs

### input

| Name | Type | Description |
| --- | --- | --- |
| `~/input/path` | autoware\_planning\_msgs/msg/Path | Reference path and the corresponding drivable area |
| `~/input/odometry` | nav\_msgs/msg/Odometry | Current Velocity of ego vehicle |

### output

| Name | Type | Description |
| --- | --- | --- |
| `~/output/trajectory` | autoware\_planning\_msgs/msg/Trajectory | Optimized trajectory that is feasible to drive and collision-free |

## Flowchart

Flowchart of functions is explained here.

![uml diagram](../../../../assets/autoware_universe/main/planning/autoware_path_optimizer/image-01-aaa1b5bd85.svg)

### createPlannerData

The following data for planning is created.

```cpp
struct PlannerData
{
// input
Header header;
std::vector traj\_points; // converted from the input path
std::vector left\_bound;
std::vector right\_bound;

// ego
geometry\_msgs::msg::Pose ego\_pose;
double ego\_vel;
};
```

### check replan

When one of the following conditions are met, trajectory optimization will be executed.
Otherwise, previously optimized trajectory is used with updating the velocity from the latest input path.

max\_path\_shape\_around\_ego\_lat\_dist

- Ego moves longer than `replan.max_ego_moving_dist` in one cycle. (default: 3.0 [m])
  - This is for when the ego pose is set again in the simulation.
- Trajectory's end, which is considered as the goal pose, moves longer than `replan.max_goal_moving_dist` in one cycle. (default: 15.0 [ms])
  - When the goal pose is set again, the planning should be reset.
- Time passes. (default: 1.0 [s])
  - The optimization is skipped for a while sine the optimization is sometimes heavy.
- The input path changes laterally longer than `replan.max_path_shape_around_ego_lat_dist` in one cycle. (default: 2.0)

### getModelPredictiveTrajectory

This module makes the trajectory kinematically-feasible and collision-free.
We define vehicle pose in the frenet coordinate, and minimize tracking errors by optimization.
This optimization considers vehicle kinematics and collision checking with road boundary and obstacles.
To decrease the computation cost, the optimization is applied to the shorter trajectory (default: 50 [m]) than the whole trajectory, and concatenate the remained trajectory with the optimized one at last.

The trajectory just in front of the ego must not be changed a lot so that the steering wheel will be stable.
Therefore, we use the previously generated trajectory in front of the ego.

Optimization center on the vehicle, that tries to locate just on the trajectory, can be tuned along side the vehicle vertical axis.
This parameter `mpt.kinematics.optimization center offset` is defined as the signed length from the back-wheel center to the optimization center.
Some examples are shown in the following figure, and it is shown that the trajectory of vehicle shape differs according to the optimization center even if the reference trajectory (green one) is the same.

![mpt_optimization_offset](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/mpt_optimization_offset.svg)

More details can be seen [here](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/docs/mpt/).

### applyInputVelocity

Velocity is assigned in the optimized trajectory from the velocity in the behavior path.
The shapes of the optimized trajectory and the path are different, therefore the each nearest trajectory point to the path is searched and the velocity is interpolated with zero-order hold.

### insertZeroVelocityOutsideDrivableArea

Optimized trajectory is too short for velocity planning, therefore extend the trajectory by concatenating the optimized trajectory and the behavior path considering drivability.
Generated trajectory is checked if it is inside the drivable area or not, and if outside drivable area, output a trajectory inside drivable area with the behavior path or the previously generated trajectory.

As described above, the behavior path is separated into two paths: one is for optimization and the other is the remain. The first path becomes optimized trajectory, and the second path just is transformed to a trajectory. Then a trajectory inside the drivable area is calculated as follows.

- If optimized trajectory is **inside the drivable area**, and the remained trajectory is inside/outside the drivable area,
  - the output trajectory will be just concatenation of those two trajectories.
  - In this case, we do not care if the remained trajectory is inside or outside the drivable area since generally it is outside the drivable area (especially in a narrow road), but we want to pass a trajectory as long as possible to the latter module.
- If optimized trajectory is **outside the drivable area**, and the remained trajectory is inside/outside the drivable area,
  - and if the previously generated trajectory **is memorized**,
    - the output trajectory will be the previously generated trajectory, where zero velocity is inserted to the point firstly going outside the drivable area.
  - and if the previously generated trajectory **is not memorized**,
    - the output trajectory will be a part of trajectory just transformed from the behavior path, where zero velocity is inserted to the point firstly going outside the drivable area.

Optimization failure is dealt with the same as if the optimized trajectory is outside the drivable area.
The output trajectory is memorized as a previously generated trajectory for the next cycle.

*Rationale*
In the current design, since there are some modelling errors, the constraints are considered to be soft constraints.
Therefore, we have to make sure that the optimized trajectory is inside the drivable area or not after optimization.

## Limitation

- Computation cost is sometimes high.
- Because of the approximation such as linearization, some narrow roads cannot be run by the planner.
- Roles of planning for `behavior_path_planner` and `path_optimizer` are not decided clearly. Both can avoid obstacles.

## Comparison to other methods

Trajectory planning problem that satisfies kinematically-feasibility and collision-free has two main characteristics that makes hard to be solved: one is non-convex and the other is high dimension.
Based on the characteristics, we investigate pros/cons of the typical planning methods: optimization-based, sampling-based, and learning-based method.

### Optimization-based method

- pros: comparatively fast against high dimension by leveraging the gradient descent
- cons: often converge to the local minima in the non-convex problem

### Sampling-based method

- pros: realize global optimization
- cons: high computation cost especially in the complex case

### Learning-based method

- under research yet

Based on these pros/cons, we chose the optimization-based planner first.
Although it has a cons to converge to the local minima, it can get a good solution by the preprocessing to approximate the problem to convex that almost equals to the original non-convex problem.

## How to Tune Parameters

### Drivability in narrow roads

- modify `mpt.clearance.soft_clearance_from_road`
  - This parameter describes how much margin to make between the trajectory and road boundaries.
  - Due to the model error for optimization, the constraint such as collision-free is not fully met.
    - By making this parameter larger, the is for narrow-road driving may be resolved. 12180
- modify `mpt.kinematics.optimization_center_offset`
  - The point on the vehicle, offset forward with this parameter from the base link` tries to follow the reference path.

- change or tune the method to approximate footprints with a set of circles.
  - See [here](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/docs/mpt/#collision-free)
  - Tuning means changing the ratio of circle's radius.

### Computation time

- under construction

### Robustness

- Check if the trajectory before or after MPT is not robust
  - if the trajectory before MPT is not robust
  - if the trajectory after MPT is not robust
    - make `mpt.weight.steer_input_weight` or `mpt.weight.steer_rate_weight` larger, which are stability of steering wheel along the trajectory.

### Other options

- `option.enable_skip_optimization` skips MPT optimization.
- `option.enable_calculation_time_info` enables showing each calculation time for functions and total calculation time on the terminal.
- `option.enable_outside_drivable_area_stop` enables stopping just before the generated trajectory point will be outside the drivable area.

### Parameters

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| option.enable\_skip\_optimization | boolean | Skip elastic band and model predictive trajectory optimization. | False | N/A |
| option.enable\_reset\_prev\_optimization | boolean | If true, optimization has no fix constraint to the previous result. | False | N/A |
| option.enable\_outside\_drivable\_area\_stop | boolean | Stop if the ego's trajectory footprint is outside the drivable area. | False | N/A |
| option.use\_footprint\_polygon\_for\_outside\_drivable\_area\_check | boolean | If false, only the footprint's corner points are considered for drivable area check. | False | N/A |
| debug.enable\_pub\_debug\_marker | boolean | Publish debug markers. | False | N/A |
| debug.enable\_pub\_extra\_debug\_marker | boolean | Publish extra debug markers. | False | N/A |
| debug.enable\_debug\_info | boolean | Enable debug information. | False | N/A |
| debug.enable\_calculation\_time\_info | boolean | Enable calculation time information. | False | N/A |
| common.output\_delta\_arc\_length | float | Delta arc length for output trajectory [m]. | 0.5 | N/A |
| common.output\_backward\_traj\_length | float | Backward length for backward trajectory from base\_link [m]. | 5.0 | N/A |
| common.vehicle\_stop\_margin\_outside\_drivable\_area | float | Vehicle stop margin to let the vehicle stop before the calculated stop point if it is calculated outside the drivable area [m]. | 0.0 | N/A |
| replan.max\_path\_shape\_around\_ego\_lat\_dist | float | Threshold of path shape change around ego [m]. | 2.0 | N/A |
| replan.max\_path\_shape\_forward\_lon\_dist | float | Forward point to check lateral distance difference [m]. | 100.0 | N/A |
| replan.max\_path\_shape\_forward\_lat\_dist | float | Threshold of path shape change around forward point [m]. | 0.1 | N/A |
| replan.max\_ego\_moving\_dist | float | Threshold of ego's moving distance for replan [m]. | 5.0 | N/A |
| replan.max\_goal\_moving\_dist | float | Threshold of goal's moving distance for replan [m]. | 15.0 | N/A |
| replan.max\_delta\_time\_sec | float | Threshold of delta time for replan [second]. | 0.0 | N/A |
| mpt.use\_acados | boolean | Enable/disable acados solver for MPT optimization | False | N/A |
| mpt.use\_acados\_circle\_constraints | boolean | Enable/disable the new MPT-style circle road-bound constraints in acados | True | N/A |
| option.steer\_limit\_constraint | boolean | Enable steer limit constraint. | False | N/A |
| option.visualize\_sampling\_num | integer | Number of samples to visualize. | 1 | N/A |
| option.enable\_manual\_warm\_start | boolean | Enable manual warm start. | False | N/A |
| option.enable\_warm\_start | boolean | Enable warm start. | False | N/A |
| option.enable\_optimization\_validation | boolean | Enable optimization validation. | False | N/A |
| common.num\_points | integer | Number of points for optimization. | 100 | N/A |
| common.delta\_arc\_length | float | Delta arc length for optimization [m]. | 1.0 | N/A |
| clearance.hard\_clearance\_from\_road | float | Clearance from road boundary [m] if collision\_free\_constraints.option.hard\_constraint is true. | 0.0 | N/A |
| clearance.soft\_clearance\_from\_road | float | Clearance from road boundary [m] if collision\_free\_constraints.option.soft\_constraint is true. | 0.1 | N/A |
| weight.soft\_collision\_free\_weight | float | Soft weight for lateral error around the middle point. | 1.0 | N/A |
| weight.lat\_error\_weight | float | Weight for lateral error. | 1.0 | N/A |
| weight.yaw\_error\_weight | float | Weight for yaw error. | 0.0 | N/A |
| weight.yaw\_error\_rate\_weight | float | Weight for yaw error rate. | 0.0 | N/A |
| weight.steer\_input\_weight | float | Weight for steering input. | 1.0 | N/A |
| weight.steer\_rate\_weight | float | Weight for steering rate. | 1.0 | N/A |
| weight.terminal\_lat\_error\_weight | float | Weight for lateral error at terminal point. | 100.0 | N/A |
| weight.terminal\_yaw\_error\_weight | float | Weight for yaw error at terminal point. | 100.0 | N/A |
| weight.goal\_lat\_error\_weight | float | Weight for lateral error at path end point. | 1000.0 | N/A |
| weight.goal\_yaw\_error\_weight | float | Weight for yaw error at path end point. | 1000.0 | N/A |
| avoidance.max\_bound\_fixing\_time | float | Maximum bound fixing time [s]. | 1.0 | N/A |
| avoidance.max\_longitudinal\_margin\_for\_bound\_violation | float | Maximum longitudinal margin for bound violation [m]. | 1.0 | N/A |
| avoidance.max\_avoidance\_cost | float | Maximum avoidance cost [m]. | 0.5 | N/A |
| avoidance.avoidance\_cost\_margin | float | Avoidance cost margin [m]. | 0.0 | N/A |
| avoidance.avoidance\_cost\_band\_length | float | Avoidance cost band length [m]. | 5.0 | N/A |
| avoidance.avoidance\_cost\_decrease\_rate | float | Decreased cost per point interval. | 0.05 | N/A |
| avoidance.min\_drivable\_width | float | Minimum drivable width [m]. The vehicle width and this parameter are guaranteed to keep for collision-free constraint. | 0.2 | N/A |
| weight.lat\_error\_weight | float | Weight for lateral error. | 0.0 | N/A |
| weight.yaw\_error\_weight | float | Weight for yaw error. | 10.0 | N/A |
| weight.steer\_input\_weight | float | Weight for yaw error. | 100.0 | N/A |
| option.l\_inf\_norm | boolean | Use L-infinity norm for collision-free constraints. | True | N/A |
| option.soft\_constraint | boolean | Enable soft constraints. | True | N/A |
| option.hard\_constraint | boolean | Enable hard constraints. | False | N/A |
| vehicle\_circles.method | string | Method to represent footprint as circles. | fitting\_uniform\_circle | N/A |
| bicycle\_model.num\_for\_calculation | integer | Number of circles for calculation. | 3 | N/A |
| bicycle\_model.front\_radius\_ratio | float | Front radius ratio. | 1.0 | N/A |
| bicycle\_model.rear\_radius\_ratio | float | Rear radius ratio. | 1.0 | N/A |
| uniform\_circle.num | integer | Number of circles. | 3 | N/A |
| uniform\_circle.radius\_ratio | float | Radius ratio. | 1.0 | N/A |
| fitting\_uniform\_circle.num | integer | Number of circles. | 3 | N/A |
| validation.max\_lat\_error | float | Maximum lateral error for validation [m]. | 5.0 | N/A |
| validation.max\_yaw\_error | float | Maximum yaw error for validation [rad]. | 1.046 | N/A |

## Spell-checking (CSpell)

Generated acados C code under `src/acados_mpc/c_generated_code/` is ignored in repo-wide CSpell runs. To spell-check only that folder, run from the **autoware\_universe** package root (e.g. `src/universe/autoware_universe`):

```
cd path/to/autoware/src/universe/autoware_universe
npx --yes cspell "planning/autoware_path_optimizer/src/acados_mpc/c_generated_code/**/*.{c,h}" --no-progress
```

Running CSpell from inside `c_generated_code` with a glob like `**/*.{c,h}` and `--config` pointing to the repo config makes the glob relative to the config directory, so the whole path\_optimizer package gets checked instead of just that folder.

## How To Debug

How to debug can be seen [here](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/docs/debug/).
