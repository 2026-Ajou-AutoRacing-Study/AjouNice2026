---
source_type: rendered_autoware_docs
site: autoware_universe
title: Autoware Trajectory Modifier
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_modifier/
rendered_html: rendered/autoware_universe_html/index-174.html
generated: 2026-06-24T14:59:32.747486+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Autoware Trajectory Modifier

The `autoware_trajectory_modifier` package provides a plugin-based architecture for post-processing trajectory points to improve trajectory quality and ensure vehicle safety. It takes candidate trajectories and applies various modification algorithms to enhance their feasibility and safety characteristics.

## Features

- Plugin-based architecture for extensible trajectory modifications
- Stop point fixing to prevent trajectory issues near stationary conditions
- Obstacle detection and stopping to prevent collision
- Configurable parameters to adjust modification behavior

## Architecture

The trajectory modifier uses a plugin-based system where different modification algorithms can be implemented as plugins. Each plugin inherits from the `TrajectoryModifierPluginBase` class and implements the required interface.

### Plugin Interface

All modifier plugins must inherit from `TrajectoryModifierPluginBase` and implement:

- `modify_trajectory()` - Main method to modify trajectory points
- `on_initialize()` - Initialize plugin members and parameters
- `update_params()` - Handle parameter updates
- `is_trajectory_modification_required()` - Determine if modification is needed

### Current Plugins

#### Stop Point Fixer

The Stop Point Fixer plugin addresses trajectory issues when the ego vehicle is stationary or moving at very low speeds. It prevents problematic trajectory points that could cause planning issues by replacing the trajectory with a single stop point when either of two independently configurable conditions is met:

- **Close stop**: the trajectory's last point (stop point) is within a minimum distance threshold from ego
- **Long stop**: the trajectory commands ego to remain stopped for longer than a minimum duration threshold

Both conditions are individually enabled or disabled via parameters, allowing fine-grained control over when the override is applied.

#### Obstacle Stop

The Obstacle Stop plugin serves as a deterministic safety shield operating independently of the generative model to:

- **Enforce Longitudinal Safety**: Monitors the gap to dynamic and static obstacles to ensure a safe distance is maintained under all kinematic conditions.
- **Ensure Definitive Stopping For Obstacles**: Guarantees zero-velocity set-points for stationary objects (e.g., traffic lights, stopped vehicles) to prevent "creeping" or oscillating behavior near obstacles.

#### Velocity Modifier

The Velocity Modifier plugins is responsible for ensuring the velocity profile is smooth and feasible, and adjusts the velocity profile if an anomaly is detected while respecting deceleration and jerk constraints.

## Dependencies

This package depends on the following packages:

- `autoware_internal_planning_msgs`: For candidate trajectory message types
- `autoware_planning_msgs`: For output trajectory message types
- `autoware_motion_utils`: Motion-related utility functions
- `autoware_trajectory`: Trajectory data structures and utilities
- `autoware_utils`: Common utility functions

## Input/Output

- **Input**: `autoware_internal_planning_msgs::msg::CandidateTrajectories`
- **Output**: Modified `autoware_internal_planning_msgs::msg::CandidateTrajectories` and selected `autoware_planning_msgs::msg::Trajectory`

## Parameters

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| plugin\_names | array | List of trajectory modifier plugins to load | ['autoware::trajectory\_modifier::plugin::ObstacleStop', 'autoware::trajectory\_modifier::plugin::StopPointFixer', 'autoware::trajectory\_modifier::plugin::VelocityModifier'] | N/A |
| use\_stop\_point\_fixer | boolean | Enable the stop point fixer modifier plugin | True | N/A |
| use\_obstacle\_stop | boolean | Enable the obstacle stop modifier plugin | True | N/A |
| use\_velocity\_modifier | boolean | Enable the velocity modifier plugin | True | N/A |
| trajectory\_time\_step | float | Time step for trajectory modification [s] | 0.1 | >0.0 |
| stop\_point\_fixer.force\_stop\_long\_stopped\_trajectories | boolean | Force zero velocity trajectory for trajectories with long stops | True | N/A |
| stop\_point\_fixer.force\_stop\_close\_stopped\_trajectories | boolean | Force zero velocity trajectory for trajectories with stops close to ego | True | N/A |
| stop\_point\_fixer.velocity\_threshold | float | Velocity threshold below which ego vehicle is considered stationary | 0.25 | ≥0.0 |
| stop\_point\_fixer.min\_distance\_threshold | float | Minimum distance threshold to trigger trajectory replacement | 1.0 | ≥0.0 |
| stop\_point\_fixer.min\_stop\_duration | float | Minimum duration of stop to consider for trajectory replacement | 0.5 | ≥0.0 |
| stopping\_constraints.nominal\_deceleration | float | Nominal deceleration during stopping [m/s^2] | 1.0 | ≥0.0 |
| stopping\_constraints.maximum\_deceleration | float | Maximum deceleration during stopping [m/s^2] | 4.0 | ≥0.0 |
| stopping\_constraints.jerk\_limit | float | Jerk limit during stopping [m/s^3] | 3.0 | ≥0.0 |
| obstacle\_stop.use\_objects | boolean | Enable the use of objects for obstacle stop | True | N/A |
| obstacle\_stop.use\_pointcloud | boolean | Enable the use of pointcloud for obstacle stop | True | N/A |
| obstacle\_stop.enable\_stop\_for\_objects | boolean | Enable stopping behavior when object is detected as obstacle | True | N/A |
| obstacle\_stop.enable\_stop\_for\_pointcloud | boolean | Enable stopping behavior when pointcloud is detected as obstacle | False | N/A |
| obstacle\_stop.stop\_margin | float | Distance to maintain from an obstacle when stopped [m] | 6.0 | ≥0.0 |
| obstacle\_stop.lateral\_margin | float | Lateral margin on top of ego width from trajectory to consider for obstacle stop [m] | 0.5 | ≥0.0 |
| obstacle\_stop.duplicate\_check\_threshold | float | Threshold to check if the stop point is already in the trajectory [m] | 0.5 | ≥0.0 |
| obstacle\_stop.arrived\_distance\_threshold | float | Threshold to check if the ego vehicle has arrived at the stop point [m] | 0.5 | ≥0.0 |
| obstacle\_tracking.on\_time\_buffer | float | Continuous detection duration to consider object/pointcloud to be persistent [s] | 0.5 | ≥0.0 |
| obstacle\_tracking.off\_time\_buffer | float | Continuous non-detection duration to consider object/pointcloud as not persistent [s] | 1.0 | ≥0.0 |
| obstacle\_tracking.object\_distance\_th | float | Distance threshold for object tracking [m] | 1.0 | ≥0.0 |
| obstacle\_tracking.object\_yaw\_th | float | Yaw threshold for object tracking [rad] | 0.1745 | ≥0.0 |
| obstacle\_tracking.pcd\_distance\_th | float | Distance threshold for pointcloud tracking [m] | 1.0 | ≥0.0 |
| obstacle\_tracking.grace\_period | float | Grace period for object tracking [s] | 0.2 | ≥0.0 |
| objects.object\_types | array | Object types to consider for obstacle stop | ['car', 'truck', 'bus', 'trailer', 'motorcycle', 'bicycle', 'pedestrian', 'unknown'] | N/A |
| objects.max\_velocity\_th | float | Maximum velocity threshold for target object [m/s] | 5.0 | ≥0.0 |
| objects.stopped\_velocity\_th | float | Velocity threshold for target object to be considered as stopped [m/s] | 0.25 | ≥0.0 |
| objects.max\_lateral\_velocity\_th | float | Maximum lateral velocity threshold for target object to be considered for obstacle stop [m/s] | 1.0 | ≥0.0 |
| objects.safety\_buffer | float | Safety buffer to expand object shape [m] | 0.0 | ≥0.0 |
| pointcloud.height\_buffer | float | Height buffer to add above ego height [m] | 0.5 | ≥0.0 |
| pointcloud.min\_height | float | Minimum height of pointcloud [m] | 0.2 | ≥0.0 |
| voxel\_grid\_filter.x | float | X dimension of voxel grid filter [m] | 0.2 | ≥0.0 |
| voxel\_grid\_filter.y | float | Y dimension of voxel grid filter [m] | 0.2 | ≥0.0 |
| voxel\_grid\_filter.z | float | Z dimension of voxel grid filter [m] | 0.2 | ≥0.0 |
| voxel\_grid\_filter.min\_size | float | Minimum size of voxel grid filter [m] | 3 | ≥0.0 |
| clustering.tolerance | float | Tolerance for clustering [m] | 0.5 | ≥0.0 |
| clustering.min\_height | float | Minimum height for clustering [m] | 0.5 | ≥0.0 |
| clustering.min\_size | float | Minimum size for clustering [m] | 10 | ≥0.0 |
| clustering.max\_size | float | Maximum size for clustering [m] | 10000 | ≥0.0 |
| rss\_params.enable | boolean | Enable RSS safety check | True | N/A |
| object\_decel.car | float | Deceleration for car type objects [m/s^2] | 1.5 | ≥0.0 |
| object\_decel.truck | float | Deceleration for truck type objects [m/s^2] | 1.5 | ≥0.0 |
| object\_decel.bus | float | Deceleration for bus type objects [m/s^2] | 1.5 | ≥0.0 |
| object\_decel.trailer | float | Deceleration for trailer type objects [m/s^2] | 1.5 | ≥0.0 |
| object\_decel.motorcycle | float | Deceleration for motorcycle type objects [m/s^2] | 3.0 | ≥0.0 |
| object\_decel.bicycle | float | Deceleration for bicycle type objects [m/s^2] | 5.0 | ≥0.0 |
| object\_decel.pedestrian | float | Deceleration for pedestrian type objects [m/s^2] | 5.0 | ≥0.0 |
| rss\_params.ego\_decel | float | Deceleration for ego vehicle [m/s^2] | 4.0 | ≥0.0 |
| rss\_params.reaction\_time | float | Reaction time to consider for RSS calculation [s] | 0.2 | ≥0.0 |
| rss\_params.safety\_margin | float | Safety margin to consider for RSS calculation [m] | 2.0 | ≥0.0 |
| rss\_params.lookahead\_horizon | float | Lookahead horizon for RSS calculation [s] | 1.5 | ≥0.0 |

Parameters can be set via YAML configuration files in the `config/` directory.

## Adding New Modifier Plugins

To add a new modifier plugin:

1. Create header and source files in `trajectory_modifier_plugins/`
2. Inherit from `TrajectoryModifierPluginBase`
3. Implement the required virtual methods
4. Register the plugin in the main node's `initialize_modifiers()` method
5. Add plugin-specific parameters to the schema and config files
