> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_trajectory_validator

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_trajectory_validator`
- Version: `0.51.0`
- Package description: The autoware_trajectory_validator package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::trajectory_validator::TrajectoryValidator` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/src/trajectory_validator_node.cpp` |
| pluginlib_plugin | `safety::VehicleConstraintFilter -> autoware::trajectory_validator::plugin::ValidatorInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/src/filters/safety/vehicle_constraint_filter.cpp` |
| pluginlib_plugin | `safety::OutOfLaneFilter -> autoware::trajectory_validator::plugin::ValidatorInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/src/filters/safety/out_of_lane_filter.cpp` |
| pluginlib_plugin | `traffic_rule::TrafficLightFilter -> autoware::trajectory_validator::plugin::ValidatorInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/src/filters/traffic_rule/traffic_light_filter.cpp` |

## Purpose And Role

This node performs a safety check on each candidate trajectory before it enters the final ranking stage. It drops paths that are physically impossible for the ego vehicle or that create an obvious driving risk. The node operates in three broad steps: collect the latest environment inputs, screen trajectories through a set of feasibility checks, then republish whichever paths survive. Checks applied to each trajectory: - Data validity: removes trajectories that contain NaNs, non‑finite numbers, inconsistent timestamps, or are too short.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/lanelet2_map` | `LaneletMapBin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/src/trajectory_validator_node.cpp` |
| `~/input/trajectories` | `CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/src/trajectory_validator_node.cpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/include/autoware/trajectory_validator/trajectory_validator_node.hpp` |
| `~/input/objects` | `PredictedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/include/autoware/trajectory_validator/trajectory_validator_node.hpp` |
| `~/input/acceleration` | `AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/include/autoware/trajectory_validator/trajectory_validator_node.hpp` |
| `~/input/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/include/autoware/trajectory_validator/trajectory_validator_node.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/trajectories` | `CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/src/trajectory_validator_node.cpp` |
| `~/debug/processing_time_detail_ms/trajectory_validator_node` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/src/trajectory_validator_node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| _none detected by literal scan_ |  |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| test | parameter | `traffic_light.deceleration_limit` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/plugins/test_traffic_light_filter.cpp` |
| test | parameter | `traffic_light.jerk_limit` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/plugins/test_traffic_light_filter.cpp` |
| test | parameter | `traffic_light.delay_response_time` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/plugins/test_traffic_light_filter.cpp` |
| test | parameter | `traffic_light.crossing_time_limit` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/plugins/test_traffic_light_filter.cpp` |
| test | parameter | `traffic_light.treat_amber_light_as_red_light` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/plugins/test_traffic_light_filter.cpp` |
| test | parameter | `traffic_light.checked_trajectory_length.deceleration_limit` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/plugins/test_traffic_light_filter.cpp` |
| test | parameter | `traffic_light.checked_trajectory_length.jerk_limit` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/plugins/test_traffic_light_filter.cpp` |
| test | publisher | `/trajectory_validator_node/input/lanelet2_map` | `autoware_map_msgs::msg::LaneletMapBin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/node/test_trajectory_validator_node.cpp` |
| test | publisher | `/trajectory_validator_node/input/odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/node/test_trajectory_validator_node.cpp` |
| test | publisher | `/trajectory_validator_node/input/acceleration` | `geometry_msgs::msg::AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/node/test_trajectory_validator_node.cpp` |
| test | publisher | `/trajectory_validator_node/input/objects` | `autoware_perception_msgs::msg::PredictedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/node/test_trajectory_validator_node.cpp` |
| test | publisher | `/trajectory_validator_node/input/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/node/test_trajectory_validator_node.cpp` |
| test | publisher | `/trajectory_validator_node/input/trajectories` | `autoware_internal_planning_msgs::msg::CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/node/test_trajectory_validator_node.cpp` |
| test | subscription | `/trajectory_validator_node/output/trajectories` | `autoware_internal_planning_msgs::msg::CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/test/node/test_trajectory_validator_node.cpp` |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `filter_names.autoware` | `:trajectory_validator::plugin::OutOfLaneFilter",` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/trajectory_validator.param.yaml` |
| `filter_names.autoware` | `:trajectory_validator::plugin::VehicleConstraintFilter",` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/trajectory_validator.param.yaml` |
| `out_of_lane.time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/trajectory_validator.param.yaml` |
| `out_of_lane.min_value` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/trajectory_validator.param.yaml` |
| `vehicle_constraint.max_speed` | `16.7` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/trajectory_validator.param.yaml` |
| `vehicle_constraint.max_acceleration` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/trajectory_validator.param.yaml` |
| `vehicle_constraint.max_deceleration` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/trajectory_validator.param.yaml` |
| `vehicle_constraint.max_steering_angle` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/trajectory_validator.param.yaml` |
| `vehicle_constraint.max_steering_rate` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/trajectory_validator.param.yaml` |
| `shadow_mode_filter_names.safety` | `:OutOfLaneFilter",` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `shadow_mode_filter_names.safety` | `:VehicleConstraintFilter",` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `shadow_mode_filter_names.traffic_rule` | `:TrafficLightFilter",` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `out_of_lane.time` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `out_of_lane.min_value` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `vehicle_constraint.max_speed` | `16.7` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `vehicle_constraint.max_acceleration` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `vehicle_constraint.max_deceleration` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `vehicle_constraint.max_steering_angle` | `0.8` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `vehicle_constraint.max_steering_rate` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `traffic_light.deceleration_limit` | `2.8` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `traffic_light.jerk_limit` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `traffic_light.delay_response_time` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `traffic_light.crossing_time_limit` | `2.75` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `traffic_light.treat_amber_light_as_red_light` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `traffic_light.checked_trajectory_length.deceleration_limit` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |
| `traffic_light.checked_trajectory_length.jerk_limit` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/config/trajectory_validator.param.yaml` |

## Dependencies

`ament_cmake`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_deprecated_boundary_departure_checker`, `autoware_internal_planning_msgs`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_test_utils`, `autoware_traffic_light_utils`, `autoware_utils_debug`, `autoware_utils_diagnostics`, `autoware_utils_geometry`, `autoware_utils_rclcpp`, `autoware_vehicle_info_utils`, `builtin_interfaces`, `generate_parameter_library`, `geometry_msgs`, `nav_msgs`, `pluginlib`, `rclcpp`, `rclcpp_components`, `rosidl_default_generators`, `rosidl_default_runtime`, `tl_expected`, `unique_identifier_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 41 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 0 |
| `pluginlib` | 12 |
| `parameters` | 7 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_validator/README.md`
