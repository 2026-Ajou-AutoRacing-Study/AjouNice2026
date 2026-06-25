> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_planning_validator

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/planning_validator/autoware_planning_validator`
- Version: `0.51.0`
- Package description: ros node for autoware_planning_validator
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::planning_validator::PlanningValidatorNode` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| rclcpp_component | `autoware::planning_validator::InvalidTrajectoryPublisherNode` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/invalid_trajectory_publisher/invalid_trajectory_publisher.cpp` |

## Purpose And Role

The `autoware_planning_validator` node is the last module executed in the planning component, it responsible for checking the validity of the planning trajectory before it is published to control component. The status of the validation can be viewed in the `/diagnostics` and `/validation_status` topics. When an invalidity is detected, the `autoware_planning_validator` will process the trajectory following the selected option: "0. publish the trajectory as it is", "1. stop publishing the trajectory", "2. publish the last validated trajectory".

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/input/trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| `~/input/route` | `LaneletRoute, autoware_utils::polling_policy::Newest` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp` |
| `~/input/lanelet_map_bin` | `LaneletMapBin, autoware_utils::polling_policy::Newest` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp` |
| `~/input/pointcloud` | `PointCloud2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp` |
| `~/input/kinematics` | `Odometry` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp` |
| `~/input/acceleration` | `AccelWithCovarianceStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp` |
| `~/input/operational_mode_state` | `OperationModeState` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp` |
| `~/input/traffic_signals` | `TrafficLightGroupArray` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp` |
| `~/input/trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/invalid_trajectory_publisher/invalid_trajectory_publisher.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/marker` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/debug_marker.cpp` |
| `~/virtual_wall` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/debug_marker.cpp` |
| `~/output/trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| `~/output/validation_status` | `PlanningValidatorStatus` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| `~/output/markers` | `visualization_msgs::msg::MarkerArray` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| `~/debug/processing_time_ms` | `Float64Stamped` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| `~/output/trajectory` | `Trajectory` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/invalid_trajectory_publisher/invalid_trajectory_publisher.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `default_handling_type` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| `publish_diag` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| `diag_error_count_threshold` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| `display_on_terminal` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| `soft_stop_deceleration` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| `soft_stop_jerk_lim` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |
| `th_traffic_light_timeout` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/src/node.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `planning_hz` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/config/planning_validator.param.yaml` |
| `default_handling_type` | `0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/config/planning_validator.param.yaml` |
| `publish_diag` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/config/planning_validator.param.yaml` |
| `diag_error_count_threshold` | `0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/config/planning_validator.param.yaml` |
| `display_on_terminal` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/config/planning_validator.param.yaml` |
| `soft_stop_deceleration` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/config/planning_validator.param.yaml` |
| `soft_stop_jerk_lim` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/config/planning_validator.param.yaml` |
| `th_traffic_light_timeout` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/config/planning_validator.param.yaml` |

## Dependencies

`agnocast_components`, `ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `angles`, `autoware_adapi_v1_msgs`, `autoware_cmake`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_factor_interface`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_planning_validator_test_utils`, `autoware_route_handler`, `autoware_utils`, `autoware_vehicle_info_utils`, `diagnostic_updater`, `geometry_msgs`, `libboost-dev`, `nav_msgs`, `pluginlib`, `rclcpp`, `rclcpp_components`, `rosidl_default_generators`, `rosidl_default_runtime`, `sensor_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 54 |
| `lifecycle` | 0 |
| `component` | 7 |
| `tf2` | 5 |
| `pluginlib` | 3 |
| `parameters` | 8 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator/README.md`
