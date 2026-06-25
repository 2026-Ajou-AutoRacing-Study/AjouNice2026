---
source_type: rendered_autoware_docs
site: autoware_universe
title: Trajectory Validator
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_validator/
rendered_html: rendered/autoware_universe_html/index-184.html
generated: 2026-06-24T14:59:33.385549+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Trajectory Validator

## Purpose/Role

This package provides a pluginlib-based C++ library for evaluating candidate trajectories against a configurable set of safety and traffic-rule filters. Each filter plugin receives a world-state context snapshot (`ValidatorContext`) and returns a feasibility verdict for a single trajectory. Trajectories rejected by any *enforced* plugin are removed from the output set. The library is embedded in `autoware_trajectory_selector`.

## Algorithm Overview

For each input `CandidateTrajectories` message the library runs every loaded plugin against every trajectory:

1. **Plugin evaluation**: each plugin's `is_feasible()` is called with the trajectory points and the current `ValidatorContext` (odometry, predicted objects, acceleration, HD map, traffic light states).
2. **Feasibility decision**: a trajectory survives if every plugin listed in `filter_names` returns `is_feasible = true`. Plugins listed in `shadow_mode_filter_names` are evaluated and reported but never remove a trajectory.
3. **Diagnostics**: the wrapper publishes an Autoware diagnostics status — `OK` if all trajectories pass all enforced plugins, `WARN` if at least one is rejected, `ERROR` if none survive.

## Built-in Plugins

| Class name | Category | Description |
| --- | --- | --- |
| `safety::VehicleConstraintFilter` | `safety` | Rejects trajectories that exceed maximum speed, acceleration, deceleration, steering angle, or steering rate. |
| `safety::UncrossableBoundaryDepartureFilter` | `safety` | Rejects trajectories whose footprint crosses an uncrossable map boundary (e.g., road borders). |
| `traffic_rule::TrafficLightFilter` | `traffic_rule` | Rejects trajectories that cross a red or amber stop line when the vehicle could safely stop. |

## Interface

This package is a C++ library. The topics below are subscribed to by the hosting node (`trajectory_selector_node`) and passed to the validator as a `ValidatorContext`.

### Topics

| Direction | Topic name | Message Type | Description |
| --- | --- | --- | --- |
| Subscriber | `~/input/odometry` | `nav_msgs/msg/Odometry` | Current ego pose and velocity (mandatory) |
| Subscriber | `~/input/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | Current ego acceleration (mandatory) |
| Subscriber | `~/input/objects` | `autoware_perception_msgs/msg/PredictedObjects` | Surrounding dynamic obstacles (mandatory) |
| Subscriber | `~/input/traffic_signals` | `autoware_perception_msgs/msg/TrafficLightGroupArray` | Traffic light states (optional; absent data does not block validation) |
| Subscriber | `~/input/route` | `autoware_planning_msgs/msg/LaneletRoute` | Route with the lanelets followed by ego (only required by the `TrafficLightFilter`) |
| Subscriber | `~/input/lanelet2_map` | `autoware_map_msgs/msg/LaneletMapBin` | HD map loaded once at startup (transient local QoS; mandatory) |
| Publisher | `~/debug/validation_reports` | `autoware_trajectory_validator/msg/ValidationReportArray` | Per-trajectory validation verdict and per-metric values |
| Publisher | `~/debug/markers/<plugin_name>` | `visualization_msgs/msg/MarkerArray` | Per-plugin debug visualization markers |
| Publisher | `~/debug/plugin_report_text` | `visualization_msgs/msg/MarkerArray` | Text overlay summarizing how many paths each plugin filtered |
| Publisher | `~/debug/processing_time_ms` | `autoware_internal_debug_msgs/msg/Float64Stamped` | Total validator processing time [ms] |
| Publisher | `~/debug/<plugin_name>/processing_time_ms` | `autoware_internal_debug_msgs/msg/Float64Stamped` | Per-plugin processing time [ms] |
| Publisher | `~/debug/processing_time_text` | `autoware_internal_debug_msgs/msg/StringStamped` | Human-readable processing time breakdown |

### Parameters

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| filter\_names | array | List of validator plugin class names to enforce. Trajectories rejected by any plugin in this list are removed from the output. | [''] | N/A |
| shadow\_mode\_filter\_names | array | List of validator plugin class names that run in shadow mode: results are evaluated, logged, and reported via diagnostics but do not affect the output trajectory set. | [''] | N/A |
| vehicle\_constraint.max\_speed | float | Maximum allowed longitudinal speed [m/s]. | 16.7 | ≥0.0 |
| vehicle\_constraint.max\_acceleration | float | Maximum allowed longitudinal acceleration [m/s²]. | 5.0 | ≥0.0 |
| vehicle\_constraint.max\_deceleration | float | Maximum allowed deceleration magnitude [m/s²] (positive value representing deceleration). | 5.0 | ≥0.0 |
| vehicle\_constraint.max\_steering\_angle | float | Maximum allowed front-wheel steering angle [rad]. | 0.8 | ≥0.0 |
| vehicle\_constraint.max\_steering\_rate | float | Maximum allowed steering rate [rad/s]. | 0.3 | ≥0.0 |
| traffic\_light.deceleration\_limit | float | Deceleration limit used to decide whether ego can safely stop before an amber stop line [m/s²]. A trajectory crossing an amber light is rejected if ego could stop within this deceleration. | 2.8 | N/A |
| traffic\_light.jerk\_limit | float | Jerk limit used together with `deceleration_limit` to compute the minimum stopping distance [m/s³]. | 5.0 | N/A |
| traffic\_light.delay\_response\_time | float | Driver/system response delay used to estimate the earliest possible start of braking [s]. | 0.5 | ≥0.0 |
| traffic\_light.crossing\_time\_limit | float | A trajectory crossing an amber light is rejected unless ego can fully clear the stop line within this time [s]. | 2.75 | ≥0.0 |
| traffic\_light.treat\_amber\_light\_as\_red\_light | boolean | When true, amber lights are treated identically to red lights (no dilemma-zone logic is applied). | True | N/A |
| traffic\_light.stop\_overshoot\_margin | float | Maximum allowed distance between the trajectory's stop point and the stop line for the trajectory to be considered as stopping correctly [m]. | 0.5 | ≥0.0 |
| checked\_trajectory\_length.deceleration\_limit | float | Deceleration limit used to compute the trajectory length scanned for traffic light stop lines [m/s²]. | 2.0 | N/A |
| checked\_trajectory\_length.jerk\_limit | float | Jerk limit used together with `deceleration_limit` to compute the trajectory scan length [m/s³]. | 2.0 | N/A |
| boundary\_departure.lateral\_margin\_m | float | Lateral clearance margin from an uncrossable boundary [m]. Trajectories whose footprint comes within this margin are rejected. | 0.01 | N/A |
| boundary\_departure.longitudinal\_margin\_m | float | Longitudinal look-ahead margin applied when projecting the vehicle footprint along the trajectory [m]. | 1.0 | N/A |
| boundary\_departure.max\_deceleration\_mps2 | float | Maximum deceleration (negative value) used to compute whether ego can stop before the boundary [m/s²]. | -4.0 | N/A |
| boundary\_departure.max\_jerk\_mps3 | float | Maximum jerk (negative value) used together with `max_deceleration_mps2` for the stopping distance estimate [m/s³]. | -5.0 | N/A |
| boundary\_departure.brake\_delay\_s | float | Braking response delay [s]. | 1.0 | ≥0.0 |
| boundary\_departure.time\_to\_departure\_cutoff\_s | float | Time-to-boundary-crossing threshold below which a trajectory is flagged as departing [s]. | 2.0 | ≥0.0 |
| boundary\_departure.on\_time\_buffer\_s | float | Hysteresis buffer before activating boundary-departure detection [s]. | 0.15 | ≥0.0 |
| boundary\_departure.off\_time\_buffer\_s | float | Hysteresis buffer before deactivating boundary-departure detection [s]. | 0.15 | ≥0.0 |
| boundary\_departure.enable\_developer\_marker | boolean | Publish diagnostic visualization markers intended for developer use. | True | N/A |
| boundary\_departure.boundary\_types | array | List of lanelet boundary type strings to treat as uncrossable (e.g., 'road\_border'). | ['road\_border'] | N/A |
