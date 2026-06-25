> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_trajectory_ranker

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/autoware_trajectory_ranker`
- Version: `0.51.0`
- Package description: The autoware_trajectory_ranker package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::trajectory_ranker::SimpleTrajectoryRanker` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/simple_trajectory_ranker_node.cpp` |
| rclcpp_component | `autoware::trajectory_ranker::TrajectoryRanker` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/trajectory_ranker_node.cpp` |
| pluginlib_plugin | `autoware::trajectory_ranker::metrics::TrajectoryConsistency -> autoware::trajectory_ranker::metrics::MetricInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/metrics/trajectory_consistency_metric.cpp` |
| pluginlib_plugin | `autoware::trajectory_ranker::metrics::TimeToCollision -> autoware::trajectory_ranker::metrics::MetricInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/metrics/time_to_collision_metric.cpp` |
| pluginlib_plugin | `autoware::trajectory_ranker::metrics::SteeringConsistency -> autoware::trajectory_ranker::metrics::MetricInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/metrics/steering_consistency_metric.cpp` |
| pluginlib_plugin | `autoware::trajectory_ranker::metrics::TravelDistance -> autoware::trajectory_ranker::metrics::MetricInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/metrics/distance_metric.cpp` |
| pluginlib_plugin | `autoware::trajectory_ranker::metrics::LateralDeviation -> autoware::trajectory_ranker::metrics::MetricInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/metrics/lateral_deviation_metric.cpp` |
| pluginlib_plugin | `autoware::trajectory_ranker::metrics::LongitudinalJerk -> autoware::trajectory_ranker::metrics::MetricInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/metrics/longitudinal_jerk_metric.cpp` |
| pluginlib_plugin | `autoware::trajectory_ranker::metrics::LateralAcceleration -> autoware::trajectory_ranker::metrics::MetricInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/metrics/lateral_acceleration_metric.cpp` |

## Purpose And Role

The `autoware_trajectory_ranker` package provides a flexible and extensible framework for evaluating and ranking multiple trajectory candidates in autonomous driving scenarios. It transforms a set of viable candidate paths into an ordered list by attaching a scalar score to every trajectory based on safety, comfort, and efficiency criteria. This decoupling of upstream path-generation logic from downstream motion-control logic enables: - Diverse trajectory generators (rule-based, ML-based, optimization-based) to coexist

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/candidate_trajectories` | `autoware_internal_planning_msgs::msg::CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/simple_trajectory_ranker_node.cpp` |
| `~/input/vector_map` | `LaneletMapBin` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/trajectory_ranker_node.cpp` |
| `~/input/route` | `LaneletRoute` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/trajectory_ranker_node.cpp` |
| `~/input/candidate_trajectories` | `CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/trajectory_ranker_node.cpp` |
| `~/input/objects` | `PredictedObjects` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/include/autoware/trajectory_ranker/trajectory_ranker_node.hpp` |
| `~/input/odometry` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/include/autoware/trajectory_ranker/trajectory_ranker_node.hpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/output/scored_trajectories` | `autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/simple_trajectory_ranker_node.cpp` |
| `~/debug/processing_time_detail_ms/trajectory_ranker` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/simple_trajectory_ranker_node.cpp` |
| `~/output/scored_trajectories` | `ScoredCandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/trajectory_ranker_node.cpp` |
| `~/debug/processing_time_detail_ms/trajectory_ranker` | `autoware_utils_debug::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/trajectory_ranker_node.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `ranked_generator_name_prefixes` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/simple_trajectory_ranker_node.cpp` |
| `trajectory_consistency.time_horizon` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/src/metrics/trajectory_consistency_metric.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| test | parameter | `wheel_radius` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_evaluator.cpp` |
| test | parameter | `wheel_width` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_evaluator.cpp` |
| test | parameter | `wheel_base` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_evaluator.cpp` |
| test | parameter | `wheel_tread` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_evaluator.cpp` |
| test | parameter | `front_overhang` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_evaluator.cpp` |
| test | parameter | `rear_overhang` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_evaluator.cpp` |
| test | parameter | `left_overhang` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_evaluator.cpp` |
| test | parameter | `right_overhang` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_evaluator.cpp` |
| test | parameter | `vehicle_height` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_evaluator.cpp` |
| test | parameter | `max_steer_angle` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_evaluator.cpp` |
| test | publisher | `~/input/candidate_trajectories` | `autoware_internal_planning_msgs::msg::CandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/simple_trajectory_ranker_node_test.cpp` |
| test | subscription | `~/output/scored_trajectories` | `autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/simple_trajectory_ranker_node_test.cpp` |
| test | parameter | `wheel_radius` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_metrics.cpp` |
| test | parameter | `wheel_width` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_metrics.cpp` |
| test | parameter | `wheel_base` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_metrics.cpp` |
| test | parameter | `wheel_tread` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_metrics.cpp` |
| test | parameter | `front_overhang` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_metrics.cpp` |
| test | parameter | `rear_overhang` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_metrics.cpp` |
| test | parameter | `left_overhang` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_metrics.cpp` |
| test | parameter | `right_overhang` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_metrics.cpp` |
| test | parameter | `vehicle_height` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_metrics.cpp` |
| test | parameter | `max_steer_angle` | `` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/test/test_metrics.cpp` |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `metrics.name` | `["autoware::trajectory_ranker::metrics::LateralAcceleration",` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `metrics.autoware` | `:trajectory_ranker::metrics::LongitudinalJerk",` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `metrics.autoware` | `:trajectory_ranker::metrics::TravelDistance",` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `metrics.autoware` | `:trajectory_ranker::metrics::TimeToCollision",` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `metrics.autoware` | `:trajectory_ranker::metrics::LateralDeviation",` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `metrics.autoware` | `:trajectory_ranker::metrics::SteeringConsistency",` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `metrics.autoware` | `:trajectory_ranker::metrics::TrajectoryConsistency"]` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `score_weight` | `[0.1, 0.1, 0.3, 0.6, 0.3, 0.3, 0.2]` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `trajectory_history_size` | `10` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `time_decay_weight.s0` | `[1.0, 0.8, 0.64, 0.51, 0.41, 0.33, 0.26, 0.21, 0.17, 0.13, 0.10, 0.08, 0.067, 0.053, 0.043, 0.034]` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `time_decay_weight.s1` | `[1.0, 0.8, 0.64, 0.51, 0.41, 0.33, 0.26, 0.21, 0.17, 0.13, 0.10, 0.08, 0.067, 0.053, 0.043, 0.034]` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `time_decay_weight.s2` | `[1.0, 0.8, 0.64, 0.51, 0.41, 0.33, 0.26, 0.21, 0.17, 0.13, 0.10, 0.08, 0.067, 0.053, 0.043, 0.034]` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `time_decay_weight.s3` | `[1.0, 0.8, 0.64, 0.51, 0.41, 0.33, 0.26, 0.21, 0.17, 0.13, 0.10, 0.08, 0.067, 0.053, 0.043, 0.034]` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `time_decay_weight.s4` | `[1.0, 0.8, 0.64, 0.51, 0.41, 0.33, 0.26, 0.21, 0.17, 0.13, 0.10, 0.08, 0.067, 0.053, 0.043, 0.034]` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `time_decay_weight.s5` | `[1.0, 0.8, 0.64, 0.51, 0.41, 0.33, 0.26, 0.21, 0.17, 0.13, 0.10, 0.08, 0.067, 0.053, 0.043, 0.034]` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `time_decay_weight.s6` | `[1.0, 0.8, 0.64, 0.51, 0.41, 0.33, 0.26, 0.21, 0.17, 0.13, 0.10, 0.08, 0.067, 0.053, 0.043, 0.034]` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/evaluation.param.yaml` |
| `trajectory_consistency.time_horizon` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/config/metrics/trajectory_consistency.param.yaml` |

## Dependencies

`ament_cmake`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_test_utils`, `autoware_utils_debug`, `autoware_utils_rclcpp`, `autoware_utils_uuid`, `autoware_vehicle_info_utils`, `autoware_vehicle_msgs`, `generate_parameter_library`, `geometry_msgs`, `lanelet2_core`, `magic_enum`, `nav_msgs`, `pluginlib`, `rclcpp`, `rclcpp_components`, `std_msgs`, `tf2`, `tf2_geometry_msgs`, `unique_identifier_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 95 |
| `lifecycle` | 0 |
| `component` | 5 |
| `tf2` | 14 |
| `pluginlib` | 19 |
| `parameters` | 23 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/autoware_trajectory_ranker/README.md`
