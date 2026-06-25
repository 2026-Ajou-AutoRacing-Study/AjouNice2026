---
source_type: rendered_autoware_docs
site: autoware_universe
title: Trajectory Adapter
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_adapter/
rendered_html: rendered/autoware_universe_html/index-172.html
generated: 2026-06-24T14:59:32.654860+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Trajectory Adapter

## Purpose/Role

This node converts a set of ranked candidate trajectories into a single, cleaned‑up [autoware\_planning\_msgs/msg/Trajectory](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/Trajectory.msg).

## Algorithm Overview

Upon receiving a message, the node finds the candidate with the highest score.
If no candidate is found an empty list would be published. The selected trajectory’s points are then processed, ensuring that successive points match the conditions to conduct control modules.

## Interface

### Topics

| Direction | Topic Name | Message Type | Description |
| --- | --- | --- | --- |
| Subscribe | `~/input/trajectories` | `autoware_internal_planning_msgs/msg/ScoredCandidateTrajectories` | Candidate trajectories with scores |
| Publish | `~/output/trajectory` | `autoware_planning_msgs/msg/Trajectory` | Re-arranged best‑score trajectory |

### Parameters

The current implementation does not expose any ROS parameters. All behavior is hard‑coded.

## Future Work

The following features and improvements are planned for future development:

- **Turn Signal Control**: Implement proper turn signal activation based on trajectory curvature and lane change intentions
- **Hazard Light Control**: Add logic to activate hazard lights in emergency situations or when the vehicle needs to warn other traffic participants
