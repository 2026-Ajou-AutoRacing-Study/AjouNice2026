> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware_rtc_replayer

## Identity

- Source set: `release_workspace`
- Repository: `tools`
- Package path: `tools/planning/autoware_rtc_replayer`
- Version: `0.7.0`
- Package description: The autoware_rtc_replayer package
- Primary README: `sources/autoware_workspace/tools/planning/autoware_rtc_replayer/README.md`

## Runtime Surfaces

Package-level specs may include multiple ROS 2 components, plugins, or executable entry points. Split ROS 1 rebuild work by these surfaces when present.

| Surface kind | Symbol | Source |
|---|---|---|
| rclcpp_component | `autoware::rtc_replayer::RTCReplayerNode` | `sources/autoware_workspace/tools/planning/autoware_rtc_replayer/src/autoware_rtc_replayer_node.cpp` |

## Purpose And Role

The current issue for RTC commands is that service is not recorded to rosbag, so it's very hard to analyze what was happened exactly. So this package makes it possible to replay rtc commands service from rosbag rtc status topic to resolve that issue. This package can't replay CooperateCommands correctly if CooperateStatusArray is not stable. And this replay is always later one step than actual however it will not affect much for behavior. tbd.

## Inputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| `/debug/rtc_status` | `CooperateStatusArray` | `sources/autoware_workspace/tools/planning/autoware_rtc_replayer/src/autoware_rtc_replayer_node.cpp` |

## Outputs

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Topic | Type | Source |
|---|---|---|
| _none detected by literal scan_ |  |  |

## Services And Clients

Runtime source detections only. See non-runtime detections below for tests/config/docs.

| Name | Type | Source |
|---|---|---|
| `/api/external/set/rtc_commands` | `CooperateCommands` | `sources/autoware_workspace/tools/planning/autoware_rtc_replayer/src/autoware_rtc_replayer_node.cpp` |

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
| _none detected_ |  |  |

## Dependencies

`ament_cmake_auto`, `ament_lint_auto`, `autoware_cmake`, `autoware_lint_common`, `rclcpp`, `rclcpp_components`, `tier4_rtc_msgs`

## ROS 2 Constructs To Replace

| Construct | Count |
|---|---:|
| `rclcpp` | 8 |
| `lifecycle` | 0 |
| `component` | 3 |
| `tf2` | 0 |
| `pluginlib` | 0 |
| `parameters` | 0 |

## ROS 1 Rebuild Notes

- Preserve Autoware message names, field names, topic names, and semantic contracts unless an adapter is explicitly documented.
- Replace `rclcpp` publishers, subscriptions, timers, parameters, and QoS with roscpp equivalents while keeping callback behavior traceable to the source.
- Replace ROS 2 components/lifecycle behavior with an explicit ROS 1 node, nodelet, or wrapper lifecycle plan.
- Re-check every row in the inputs, outputs, services, and parameters tables against source before implementing.

## Source Evidence

- `sources/autoware_workspace/tools/planning/autoware_rtc_replayer/README.md`
