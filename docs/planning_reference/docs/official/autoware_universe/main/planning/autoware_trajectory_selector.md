---
source_type: rendered_autoware_docs
site: autoware_universe
title: Trajectory Selector
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_selector/
rendered_html: rendered/autoware_universe_html/index-183.html
generated: 2026-06-24T14:59:33.337142+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Trajectory Selector

## Purpose/Role

`trajectory_selector_node` is the ROS 2 node that orchestrates the full candidate trajectory pipeline. It buffers trajectory candidates from multiple upstream planners via `autoware_trajectory_concatenator`, validates each candidate against the safety and traffic-rule plugins of `autoware_trajectory_validator`, and publishes the surviving trajectories for downstream processing.

## Algorithm Overview

1. Candidate trajectories arrive on two separate input topics. `~/input/trajectories_generative` acts as an **anchor**: receiving a message on this topic triggers an immediate concatenation and validation cycle. `~/input/trajectories_backup` messages are buffered and processed in the next cycle.
2. A 100 ms timer also fires periodically to ensure output is published even if the anchor input is delayed or missing. The timer is reset whenever an anchor message is received.
3. During each cycle (triggered by the anchor or the timer), the concatenator prunes stale entries (older than `duration_time`) and returns the merged trajectory set from all registered generators.
4. If any mandatory sensor input (odometry, predicted objects, acceleration, or HD map) is unavailable, the cycle returns early without publishing.
5. The validator runs all loaded plugins against each trajectory. Trajectories rejected by any enforced plugin (listed in `filter_names`) are removed from the output set.
6. The surviving trajectories are published to `~/output/trajectories`.

## Interface

### Topics

| Direction | Topic name | Message Type | Description |
| --- | --- | --- | --- |
| Subscriber | `~/input/trajectories_generative` | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectories from generative planners |
| Subscriber | `~/input/trajectories_backup` | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectories from backup planners |
| Subscriber | `~/input/lanelet2_map` | `autoware_map_msgs/msg/LaneletMapBin` | HD map (transient local QoS; loaded once at startup) |
| Subscriber | `~/input/odometry` | `nav_msgs/msg/Odometry` | Current ego pose and velocity |
| Subscriber | `~/input/objects` | `autoware_perception_msgs/msg/PredictedObjects` | Surrounding dynamic obstacles |
| Subscriber | `~/input/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | Current ego acceleration |
| Subscriber | `~/input/traffic_signals` | `autoware_perception_msgs/msg/TrafficLightGroupArray` | Traffic light states (optional; missing data does not block the timer) |
| Publisher | `~/output/trajectories` | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectories that passed all enforced validator plugins |
| Publisher | `~/debug/processing_time_detail_ms/trajectory_selector` | `autoware_internal_debug_msgs/msg/ProcessingTimeTree` | Per-function processing time breakdown |

### Parameters

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| fallback\_period\_ms | integer | fallback timer period in milliseconds | 150 | ≥1 |

Other configuration is provided through the sub-package parameter files loaded at launch:

- Concatenation parameters: see [`autoware_trajectory_concatenator`](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_concatenator/#parameters)
- Validation parameters: see [`autoware_trajectory_validator`](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_validator/#parameters)
