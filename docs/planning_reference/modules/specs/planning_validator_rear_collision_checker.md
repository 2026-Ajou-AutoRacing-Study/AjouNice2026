> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_planning_validator_rear_collision_checker

## Identity

- Source set: `release_workspace`
- Repository: `universe/autoware_universe`
- Package path: `universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker`
- Version: `0.51.0`
- Package description: The autoware_planning_validator_rear_collision_checker package
- Primary README: `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| pluginlib_plugin | `autoware::planning_validator::RearCollisionChecker -> autoware::planning_validator::PluginInterface` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/src/rear_collision_checker.cpp` |

## Purpose And Role

The `rear_collision_checker` is a plugin module of the `autoware_planning_validator` node. It validates the planned trajectory by verifying that it does **not** lead to a collision with other road users (primarily vehicles, but also pedestrian/cyclists/motorcycles) approaching from lateral/rear directions. In particular, it is designed to detect potential rear-end collisions and entrapment risks when the ego vehicle is making a right/left turn or merging into an adjacent lane.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/rear_collision_checker/debug/cluster_points` | `PointCloud2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/src/rear_collision_checker.cpp` |
| `~/rear_collision_checker/debug/voxel_points` | `PointCloud2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/src/rear_collision_checker.cpp` |
| `~/rear_collision_checker/debug/state` | `StringStamped` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/src/rear_collision_checker.cpp` |
| `~/rear_collision_checker/debug/processing_time_detail_ms` | `autoware_utils::ProcessingTimeDetail` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/src/rear_collision_checker.cpp` |

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
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| `common.on_time_buffer` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.off_time_buffer` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.check_on_unstoppable` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.range.dead_zone` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.range.buffer` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.crop_box_filter.x.max` | `30.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.crop_box_filter.x.min` | `-100.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.crop_box_filter.z.max` | `-1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.crop_box_filter.z.min` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.voxel_grid_filter.x` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.voxel_grid_filter.y` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.voxel_grid_filter.z` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.clustering.cluster_tolerance` | `0.15` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.clustering.min_cluster_height` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.clustering.min_cluster_size` | `5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.clustering.max_cluster_size` | `10000` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.velocity_estimation.observation_time` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.velocity_estimation.max_acceleration` | `10.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.pointcloud.latency` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.filter.min_velocity` | `1.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.filter.moving_time` | `0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.time_to_collision.margin` | `2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.ego.reaction_time` | `1.2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.ego.min_velocity` | `1.38` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.ego.max_velocity` | `16.6` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.ego.max_acceleration` | `1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.ego.max_deceleration` | `-4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.ego.max_positive_jerk` | `5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.ego.max_negative_jerk` | `-5.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.ego.nominal_deceleration` | `-1.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.ego.nominal_positive_jerk` | `0.6` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.ego.nominal_negative_jerk` | `-0.6` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.blind_spot.lookahead_time` | `4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.blind_spot.metric` | `ttc` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.blind_spot.check.front` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.blind_spot.check.left` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.blind_spot.check.right` | `false` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.blind_spot.check.yaw_th` | `0.78` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.blind_spot.offset.inner` | `0.1` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.blind_spot.offset.outer` | `0.3` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.blind_spot.participants.reaction_time` | `1.2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.blind_spot.participants.max_velocity` | `5.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.blind_spot.participants.max_deceleration` | `-2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.adjacent_lane.lookahead_time` | `4.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.adjacent_lane.metric` | `rss` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.adjacent_lane.check.front` | `true` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.adjacent_lane.offset.left` | `-0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.adjacent_lane.offset.right` | `-0.5` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.adjacent_lane.participants.reaction_time` | `1.2` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.adjacent_lane.participants.max_velocity` | `16.6` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |
| `common.adjacent_lane.participants.max_deceleration` | `-2.0` | `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/config/rear_collision_checker.param.yaml` |

## Dependencies

`ament_cmake_auto`, `ament_cmake_ros`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_lanelet2_extension`, `autoware_lanelet2_utils`, `autoware_lint_common`, `autoware_motion_utils`, `autoware_planning_factor_interface`, `autoware_planning_msgs`, `autoware_planning_test_manager`, `autoware_planning_validator`, `autoware_route_handler`, `autoware_signal_processing`, `autoware_utils`, `autoware_utils_geometry`, `autoware_vehicle_info_utils`, `diagnostic_msgs`, `eigen`, `generate_parameter_library`, `libpcl-all-dev`, `pcl_conversions`, `rclcpp`, `rclcpp_components`, `sensor_msgs`, `std_msgs`, `tf2`, `tf2_eigen`, `tf2_geometry_msgs`, `tf2_ros`, `tier4_planning_msgs`, `visualization_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 23 |
| `lifecycle` | 0 |
| `component` | 1 |
| `tf2` | 5 |
| `pluginlib` | 2 |
| `parameters` | 1 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker/README.md`
