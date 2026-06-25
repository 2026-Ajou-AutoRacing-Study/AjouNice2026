> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_planning_data_analyzer

## Identity

- Source set: `release_workspace`
- Repository: `tools`
- Package path: `tools/planning/autoware_planning_data_analyzer`
- Version: `0.7.0`
- Package description: The autoware_planning_data_analyzer package
- Primary README: `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::planning_data_analyzer::AutowarePlanningDataAnalyzerNode` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |

## Purpose And Role

This package provides offline evaluation tools for trajectory planning performance analysis from recorded rosbag data. Open-loop evaluation computes both trajectory error metrics and planning-quality subscores. For each evaluated trajectory, the following are computed: - ADE - FDE - AHE - FHE - lateral deviation - longitudinal deviation - TTC trace - per-horizon metrics (full, and configured horizons such as 1.0s, 2.0s, 4.0s, 8.0s) To show multi-facet (comfort, progress, safety) quality of generated trajectory, EPDMS (Extended Predictive Driver Model Score) is computed based on the multiple subscores.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `input_bag_path` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `map_topic` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `map_marker_array` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `route_topic` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `odometry_topic` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `trajectory_topic` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `candidate_trajectories_topic` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `evaluation_interval_ms` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `sync_tolerance_ms` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.gt_source_mode` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.gt_trajectory_topic` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.gt_sync_tolerance_ms` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.hc.finite_difference_epsilon` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.hc.max_longitudinal_acceleration` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.hc.min_longitudinal_acceleration` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.hc.max_lateral_acceleration` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.hc.max_jerk_magnitude` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.hc.max_longitudinal_jerk` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.hc.max_yaw_rate` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.hc.max_yaw_acceleration` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.extended_comfort.max_acceleration_rms` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.extended_comfort.max_jerk_rms` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.extended_comfort.max_yaw_rate_rms` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.extended_comfort.max_yaw_acceleration_rms` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.extended_comfort.finite_difference_epsilon` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.lane_keep.max_lateral_deviation` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.lane_keep.max_continuous_violation_time` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `objects_topic` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `traffic_signals_topic` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `tf_topic` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `acceleration_topic` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `steering_topic` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `evaluation.mode` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `output_dir` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.metric_variant` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |
| `open_loop.evaluation_horizons` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/src/autoware_planning_data_analyzer_node.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `evaluation.mode` | `"open_loop"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `evaluation_interval_ms` | `100.0` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `sync_tolerance_ms` | `100.0` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `buffer_duration_sec` | `20.0` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `max_buffer_messages` | `10000` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `input_bag_path` | `""` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `output_dir` | `""` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `map_topic` | `"/map/vector_map"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `map_marker_array` | `"/map/vector_map_marker"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `route_topic` | `"/planning/mission_planning/route"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `odometry_topic` | `"/localization/kinematic_state"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `trajectory_topic` | `"/planning/trajectory"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `candidate_trajectories_topic` | `"/diffusion_planner/output/trajectories"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `objects_topic` | `"/perception/object_recognition/objects"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `traffic_signals_topic` | `"/perception/traffic_light_recognition/traffic_signals"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `tf_topic` | `"/tf"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `acceleration_topic` | `"/localization/acceleration"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `steering_topic` | `"/vehicle/status/steering_status"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.gt_source_mode` | `"kinematic_state"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.gt_trajectory_topic` | `"/planning/ground_truth_trajectory"` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.gt_sync_tolerance_ms` | `200.0` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.hc.finite_difference_epsilon` | `0.001` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.hc.max_longitudinal_acceleration` | `2.40` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.hc.min_longitudinal_acceleration` | `-4.05` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.hc.max_lateral_acceleration` | `4.89` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.hc.max_jerk_magnitude` | `8.37` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.hc.max_longitudinal_jerk` | `4.13` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.hc.max_yaw_rate` | `0.95` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.hc.max_yaw_acceleration` | `1.93` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.metric_variant` | `""` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.evaluation_horizons` | `[1.0, 2.0, 4.0, 8.0]` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.extended_comfort.max_acceleration_rms` | `0.7` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.extended_comfort.max_jerk_rms` | `0.5` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.extended_comfort.max_yaw_rate_rms` | `0.1` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.extended_comfort.max_yaw_acceleration_rms` | `0.1` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.extended_comfort.finite_difference_epsilon` | `0.001` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.lane_keep.max_lateral_deviation` | `0.5` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |
| `open_loop.lane_keep.max_continuous_violation_time` | `2.0` | `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml` |

## Dependencies

`ament_cmake`, `ament_index_cpp`, `ament_lint_auto`, `autoware_cmake`, `autoware_global_parameter_loader`, `autoware_internal_planning_msgs`, `autoware_lanelet2_extension`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_map_msgs`, `autoware_motion_utils`, `autoware_object_recognition_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_route_handler`, `autoware_test_utils`, `autoware_traffic_light_utils`, `autoware_utils`, `autoware_utils_math`, `autoware_utils_rclcpp`, `autoware_vehicle_info_utils`, `autoware_vehicle_msgs`, `eigen3_cmake_module`, `nlohmann-json-dev`, `rclcpp`, `rosbag2_cpp`, `std_srvs`, `tf2_geometry_msgs`, `tf2_msgs`, `tf2_ros`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 214 |
| `lifecycle` | 0 |
| `component` | 2 |
| `tf2` | 20 |
| `pluginlib` | 0 |
| `parameters` | 38 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/README.md`
- `sources/autoware_workspace/tools/planning/autoware_planning_data_analyzer/docs/metrics/displacement_based_metrics.md`
