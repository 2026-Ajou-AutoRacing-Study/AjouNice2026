> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# planning_debug_tools

## Identity

- Source set: `release_workspace`
- Repository: `tools`
- Package path: `tools/planning/planning_debug_tools`
- Version: `0.7.0`
- Package description: The planning_debug_tools package
- Primary README: `sources/autoware_workspace/tools/planning/planning_debug_tools/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `planning_debug_tools::StopReasonVisualizerNode` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/stop_reason_visualizer.cpp` |
| rclcpp_component | `planning_debug_tools::TrajectoryAnalyzerNode` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/trajectory_analyzer.cpp` |
| executable_main | `main()` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_replayer_node.cpp` |
| executable_main | `main()` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_reproducer_node.cpp` |

## Purpose And Role

This package contains several planning-related debug tools. - **Trajectory analyzer**: visualizes the information (speed, curvature, yaw, etc) along the trajectory - **Closest velocity checker**: prints the velocity information indicated by each modules - **Perception reproducer**: generates perception objects and reference images from rosbag data in planning simulator environment - **processing time checker**: displays processing_time of modules on the terminal

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `/planning/scenario_planning/status/stop_reasons` | `StopReasonArray` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/stop_reason_visualizer.cpp` |
| `ego_kinematics` | `Odometry` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/trajectory_analyzer.cpp` |
| `/initialpose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_reproducer.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `~/debug/markers` | `MarkerArray` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/stop_reason_visualizer.cpp` |
| `/perception_reproducer/rosbag_ego_odom` | `Odometry` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_replayer_common.cpp` |
| `/perception/object_recognition/tracking/objects` | `TrackedObjects` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_replayer_common.cpp` |
| `/perception/object_recognition/objects` | `PredictedObjects` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_replayer_common.cpp` |
| `/perception/traffic_light_recognition/traffic_signals` | `TrafficLightGroupArray` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_replayer_common.cpp` |
| `/perception/occupancy_grid_map/map` | `OccupancyGrid` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_replayer_common.cpp` |
| `/planning/mission_planning/route` | `LaneletRoute` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_replayer_common.cpp` |
| `/planning/mission_planning/state` | `RouteState` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_replayer_common.cpp` |
| `/initialpose` | `PoseWithCovarianceStamped` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_replayer_common.cpp` |
| `/planning/mission_planning/goal` | `PoseStamped` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/perception_replayer/perception_replayer_common.cpp` |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Parameters

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Parameter | Source |
|---|---|
| `path_topics` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/trajectory_analyzer.cpp` |
| `path_with_lane_id_topics` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/trajectory_analyzer.cpp` |
| `trajectory_topics` | `sources/autoware_workspace/tools/planning/planning_debug_tools/src/trajectory_analyzer.cpp` |

## Non-Runtime Detections

These detections come from tests, configs, or docs and should not be treated as node contracts without manual confirmation.

| Context | Kind | Name | Type | Source |
|---|---|---|---|---|
| _none_ |  |  |  |  |

## Config Parameter Defaults

Flattened from `*.param.yaml` files under this package. These are defaults/config evidence, not proof that every key is read by this exact runtime surface.

| Parameter | Default | Source |
|---|---|---|
| _none detected_ |  |  |

## Dependencies

`ament_cmake_auto`, `ament_lint_auto`, `autoware_cmake`, `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`, `autoware_lint_common`, `autoware_localization_msgs`, `autoware_motion_utils`, `autoware_perception_msgs`, `autoware_planning_msgs`, `autoware_utils_geometry`, `autoware_utils_visualization`, `eigen3_cmake_module`, `geometry_msgs`, `libqt5-core`, `libqt5-gui`, `libqt5-widgets`, `nav_msgs`, `qtbase5-dev`, `rclcpp`, `rclcpp_components`, `rosbag2_cpp`, `rosbag2_storage`, `rosbag2_transport`, `rosidl_default_generators`, `rosidl_default_runtime`, `sensor_msgs`, `tier4_planning_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 140 |
| `lifecycle` | 0 |
| `component` | 5 |
| `tf2` | 12 |
| `pluginlib` | 0 |
| `parameters` | 3 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/tools/planning/planning_debug_tools/README.md`
- `sources/autoware_workspace/tools/planning/planning_debug_tools/doc-stop-reason-visualizer.md`
- `sources/autoware_workspace/tools/planning/planning_debug_tools/scripts/trajectory_visualizer/README.md`
