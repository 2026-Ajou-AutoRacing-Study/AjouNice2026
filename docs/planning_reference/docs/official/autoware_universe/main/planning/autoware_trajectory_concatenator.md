---
source_type: rendered_autoware_docs
site: autoware_universe
title: Trajectory Concatenator
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_concatenator/
rendered_html: rendered/autoware_universe_html/index-173.html
generated: 2026-06-24T14:59:32.693480+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Trajectory Concatenator

## Purpose/Role

This node aggregates trajectory candidates from multiple trajectory generators into a single `autoware_internal_planning_msgs/msg/CandidateTrajectories` message. It is intended to be placed between trajectory generators and the selector/ranker.

## Algorithm Overview

When a `CandidateTrajectories` message is received (from either `~/input/trajectories_generative` or `~/input/trajectories_backup`), the library splits it by `generator_id` and updates an in-memory buffer so that only the most recent trajectory set for each generator is retained.

A 100 ms timer then scans this buffer and drops any entry whose header stamp is older than the configured duration\_time. Immediately after pruning, the timer concatenates all remaining trajectories and their accompanying `generator_info` arrays, publishes the aggregated message.

## Interface

### Topics

| Direction | Topic name | Message Type | Description |
| --- | --- | --- | --- |
| Subscriber | `~/input/trajectories_generative` | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectory sets from generative planners |
| Subscriber | `~/input/trajectories_backup` | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectory sets from backup planners |
| Publisher | `~/output/trajectories` | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Concatenated list of all buffered trajectories |

### Parameters

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| duration\_time | float | Duration time to keep the trajectory (it will be deleted once exceeding this value) | 0.2 | ≥0.0 |
