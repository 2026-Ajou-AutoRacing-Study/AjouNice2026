> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_planning_evaluator

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/evaluator/autoware_planning_evaluator`
- Version: `0.51.0`
- Package description: ROS 2 node for evaluating planners
- Primary README: `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `planning_diagnostics::PlanningEvaluatorNode` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| rclcpp_component | `planning_diagnostics::MotionEvaluatorNode` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/motion_evaluator_node.cpp` |

## Purpose And Role

This package provides nodes that generate various metrics to evaluate the quality of planning. Metrics can be published in real time and saved to a JSON file when the node is shut down: - `metrics_for_publish`: - Metrics listed in `metrics_for_publish` are calculated and published to the topic. - `metrics_for_output`: - Metrics listed in `metrics_for_output` are saved to a JSON file when the node shuts down (if `output_metrics` is set to `true`). - These metrics include statistics derived from `metrics_for_publish` and additional information such as parameters and descriptions.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/twist` | `nav_msgs::msg::Odometry` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/motion_evaluator_node.cpp` |
| `~/input/trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/include/autoware/planning_evaluator/planning_evaluator_node.hpp` |
| `~/input/reference_trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/include/autoware/planning_evaluator/planning_evaluator_node.hpp` |
| `~/input/objects` | `PredictedObjects` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/include/autoware/planning_evaluator/planning_evaluator_node.hpp` |
| `~/input/modified_goal` | `PoseWithUuidStamped` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/include/autoware/planning_evaluator/planning_evaluator_node.hpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/include/autoware/planning_evaluator/planning_evaluator_node.hpp` |
| `~/input/route` | `LaneletRoute, autoware_utils::polling_policy::Newest` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/include/autoware/planning_evaluator/planning_evaluator_node.hpp` |
| `~/input/vector_map` | `LaneletMapBin, autoware_utils::polling_policy::Newest` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/include/autoware/planning_evaluator/planning_evaluator_node.hpp` |
| `~/input/acceleration` | `AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/include/autoware/planning_evaluator/planning_evaluator_node.hpp` |
| `~/input/steering_status` | `SteeringReport` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/include/autoware/planning_evaluator/planning_evaluator_node.hpp` |
| `~/input/turn_indicators_status` | `TurnIndicatorsReport` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/include/autoware/planning_evaluator/planning_evaluator_node.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/metrics` | `MetricArrayMsg` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `trajectory.min_point_dist_m` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `trajectory.lookahead.max_dist_m` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `trajectory.lookahead.max_time_s` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `trajectory.evaluation_time_s` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `obstacle.worst_only` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `obstacle.use_ego_traj_vel` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `obstacle.collision_thr_m` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `obstacle.stop_velocity_mps` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `obstacle.min_time_interval_s` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `obstacle.min_spatial_interval_m` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `limit.min_acc` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `stop_decision.time_count_threshold_s` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `stop_decision.dist_count_threshold_m` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `stop_decision.abnormal_deceleration_threshold_mps2` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `steer_change_count.window_duration_s` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `steer_change_count.steer_rate_margin` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `blinker_change_count.window_duration_s` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `output_metrics` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `ego_frame` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `stop_decision.topic_prefix` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/planning_evaluator_node.cpp` |
| `output_metrics` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/src/motion_evaluator_node.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `ego_frame` | `base_link` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `trajectory.min_point_dist_m` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `trajectory.evaluation_time_s` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `trajectory.lookahead.max_dist_m` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `trajectory.lookahead.max_time_s` | `3.0` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `obstacle.worst_only` | `true` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `obstacle.use_ego_traj_vel` | `false` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `obstacle.collision_thr_m` | `0.0` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `obstacle.stop_velocity_mps` | `0.2777` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `obstacle.min_time_interval_s` | `0.05` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `obstacle.min_spatial_interval_m` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `stop_decision.topic_prefix` | `/planning/planning_factors/` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `stop_decision.time_count_threshold_s` | `60.0` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `stop_decision.dist_count_threshold_m` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `stop_decision.abnormal_deceleration_threshold_mps2` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `blinker_change_count.window_duration_s` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `steer_change_count.window_duration_s` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |
| `steer_change_count.steer_rate_margin` | `0.2` | `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/config/planning_evaluator.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_index_cpp`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_factor_interface`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_utils`, `autoware_vehicle_info_utils`, `autoware_vehicle_msgs`, `eigen`, `geometry_msgs`, `nav_msgs`, `nlohmann-json-dev`, `pluginlib`, `rclcpp`, `rclcpp_components`, `tf2`, `tf2_ros`, `tier4_metric_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 112 |
| `lifecycle` | 0 |
| `component` | 5 |
| `tf2` | 35 |
| `pluginlib` | 1 |
| `parameters` | 25 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/evaluator/autoware_planning_evaluator/README.md`
