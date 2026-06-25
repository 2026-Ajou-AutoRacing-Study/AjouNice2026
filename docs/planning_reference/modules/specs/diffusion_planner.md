> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_diffusion_planner

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_diffusion_planner`
- Version: `0.51.0`
- Package description: The autoware_diffusion_planner package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::diffusion_planner::DiffusionPlanner` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |

## Purpose And Role

The **Autoware Diffusion Planner** is a trajectory generation module for autonomous vehicles, designed to work within the [Autoware](https://autoware.org/) ecosystem. It leverages the [Diffusion Planner](https://github.com/ZhengYinan-AIR/Diffusion-Planner) model, as described in the paper ["Diffusion-Based Planning for Autonomous Driving with Flexible Guidance"](https://arxiv.org/abs/2501.15564) by Zheng et al. <!-- cSpell:ignore Zheng --> This planner generates smooth, feasible, and safe trajectories by considering:

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/vector_map` | `HADMapBin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/include/autoware/diffusion_planner/diffusion_planner_node.hpp` |
| `~/input/acceleration` | `AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/include/autoware/diffusion_planner/diffusion_planner_node.hpp` |
| `~/input/tracked_objects` | `TrackedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/include/autoware/diffusion_planner/diffusion_planner_node.hpp` |
| `~/input/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray, autoware_utils::polling_policy::All` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/include/autoware/diffusion_planner/diffusion_planner_node.hpp` |
| `~/input/turn_indicators` | `TurnIndicatorsReport` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/include/autoware/diffusion_planner/diffusion_planner_node.hpp` |
| `~/input/route` | `LaneletRoute, autoware_utils::polling_policy::Newest` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/include/autoware/diffusion_planner/diffusion_planner_node.hpp` |
| `~/input/vector_map` | `LaneletMapBin, autoware_utils::polling_policy::Newest` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/include/autoware/diffusion_planner/diffusion_planner_node.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `~/output/trajectories` | `CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `~/output/predicted_objects` | `PredictedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `~/debug/route_marker` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `~/debug/lane_marker` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `~/debug/linestring_marker` | `MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `~/output/turn_indicators` | `TurnIndicatorsCommand` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `~/output/debug/traffic_signal` | `autoware_perception_msgs::msg::TrafficLightGroup` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `~/debug/processing_time_detail_ms` | `autoware_utils::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `onnx_model_path` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `args_path` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `plugins_path` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `build_only` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `planning_frequency_hz` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `ignore_neighbors` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `ignore_unknown_neighbors` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `traffic_light_group_msg_timeout_seconds` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `batch_size` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `velocity_smoothing_window` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `stopping_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `turn_indicator_keep_offset` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `turn_indicator_hold_duration` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `shift_x` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `delay_step` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `line_string_max_step_m` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `use_time_interpolation` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `planning_factor.enable_stop` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `planning_factor.enable_slowdown` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `planning_factor.stop_velocity_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `planning_factor.stop_keep_duration_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `planning_factor.slowdown_accel_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `debug_params.publish_debug_map` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `debug_params.publish_debug_route` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |
| `debug_params.publish_debug_linestrings` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/src/diffusion_planner_node.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| test | publisher | `/diffusion_planner/input/odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/test/diffusion_planner_integration_test.cpp` |
| test | publisher | `/diffusion_planner/input/acceleration` | `geometry_msgs::msg::AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/test/diffusion_planner_integration_test.cpp` |
| test | publisher | `/diffusion_planner/input/tracked_objects` | `autoware_perception_msgs::msg::TrackedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/test/diffusion_planner_integration_test.cpp` |
| test | publisher | `/diffusion_planner/input/acceleration` | `geometry_msgs::msg::AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/test/diffusion_planner_integration_test.cpp` |
| test | publisher | `/diffusion_planner/input/tracked_objects` | `autoware_perception_msgs::msg::TrackedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/test/diffusion_planner_integration_test.cpp` |
| test | publisher | `/diffusion_planner/input/odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/test/diffusion_planner_integration_test.cpp` |
| test | publisher | `/diffusion_planner/input/tracked_objects` | `autoware_perception_msgs::msg::TrackedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/test/diffusion_planner_integration_test.cpp` |
| test | publisher | `/diffusion_planner/input/odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/test/diffusion_planner_integration_test.cpp` |
| test | publisher | `/diffusion_planner/input/tracked_objects` | `autoware_perception_msgs::msg::TrackedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/test/diffusion_planner_integration_test.cpp` |
| test | publisher | `/diffusion_planner/input/odometry` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/test/diffusion_planner_integration_test.cpp` |
| test | publisher | `/diffusion_planner/input/tracked_objects` | `autoware_perception_msgs::msg::TrackedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/test/diffusion_planner_integration_test.cpp` |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `plugins_path` | `$(find-pkg-share autoware_tensorrt_plugins)/plugins/libautoware_tensorrt_plugins.so` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `onnx_model_path` | `$(var data_path)/v4.0/diffusion_planner.onnx` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `args_path` | `$(var data_path)/v4.0/diffusion_planner.param.json` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `planning_frequency_hz` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `ignore_neighbors` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `ignore_unknown_neighbors` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `traffic_light_group_msg_timeout_seconds` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `batch_size` | `1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `temperature` | `[0.0]` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `velocity_smoothing_window` | `8` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `stopping_threshold` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `turn_indicator_keep_offset` | `-1.25` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `turn_indicator_hold_duration` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `shift_x` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `delay_step` | `0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `line_string_max_step_m` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `use_time_interpolation` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `planning_factor.enable_stop` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `planning_factor.enable_slowdown` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `planning_factor.stop_keep_duration_threshold` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `planning_factor.stop_velocity_threshold` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `planning_factor.slowdown_accel_threshold` | `-0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `debug_params.publish_debug_route` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `debug_params.publish_debug_map` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |
| `debug_params.publish_debug_linestrings` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cuda_dependency_meta`, `autoware_cuda_utils`, `autoware_internal_planning_msgs`, `autoware_lanelet2_extension`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_motion_utils`, `autoware_object_recognition_utils`, `autoware_perception_msgs`, `autoware_planning_factor_interface`, `autoware_planning_msgs`, `autoware_tensorrt_common`, `autoware_tensorrt_plugins`, `autoware_test_utils`, `autoware_traffic_light_utils`, `autoware_trajectory`, `autoware_utils`, `autoware_utils_diagnostics`, `autoware_utils_geometry`, `autoware_utils_uuid`, `autoware_vehicle_info_utils`, `geometry_msgs`, `lanelet2_core`, `lanelet2_routing`, `lanelet2_traffic_rules`, `nav_msgs`, `nlohmann-json-dev`, `rclcpp`, `rclcpp_components`, `std_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 141 |
| `lifecycle` | 0 |
| `component` | 4 |
| `tf2` | 2 |
| `pluginlib` | 0 |
| `parameters` | 26 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_diffusion_planner/README.md`
