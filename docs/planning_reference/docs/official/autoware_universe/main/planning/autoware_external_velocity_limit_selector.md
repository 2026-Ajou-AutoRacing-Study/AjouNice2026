---
source_type: rendered_autoware_docs
site: autoware_universe
title: External Velocity Limit Selector
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_external_velocity_limit_selector/
rendered_html: rendered/autoware_universe_html/index-155.html
generated: 2026-06-24T14:59:31.601777+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# External Velocity Limit Selector

## Purpose

The `external_velocity_limit_selector_node` is a node that keeps consistency of external velocity limits. This module subscribes

1. velocity limit command sent by **API**,
2. velocity limit command sent by **Autoware internal modules**.

VelocityLimit.msg contains not only **max velocity** but also information about the **acceleration/jerk constraints** on deceleration. The `external_velocity_limit_selector_node` integrates the lowest velocity limit and the highest jerk constraint to calculate the **hardest velocity limit** that protects all the deceleration points and max velocities sent by API and Autoware internal modules.

![selector algorithm](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_external_velocity_limit_selector/image/external_velocity_limit_selector.png)

## Inner-workings / Algorithms

WIP

## Inputs

| Name | Type | Description |
| --- | --- | --- |
| `~input/velocity_limit_from_api` | autoware\_internal\_planning\_msgs::msg::VelocityLimit | velocity limit from api |
| `~input/velocity_limit_from_internal` | autoware\_internal\_planning\_msgs::msg::VelocityLimit | velocity limit from autoware internal modules |
| `~input/velocity_limit_clear_command_from_internal` | autoware\_internal\_planning\_msgs::msg::VelocityLimitClearCommand | velocity limit clear command |

## Outputs

| Name | Type | Description |
| --- | --- | --- |
| `~output/max_velocity` | autoware\_internal\_planning\_msgs::msg::VelocityLimit | current information of the hardest velocity limit |

## Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `max_velocity` | double | default max velocity [m/s] |
| `normal.min_acc` | double | minimum acceleration [m/ss] |
| `normal.max_acc` | double | maximum acceleration [m/ss] |
| `normal.min_jerk` | double | minimum jerk [m/sss] |
| `normal.max_jerk` | double | maximum jerk [m/sss] |
| `limit.min_acc` | double | minimum acceleration to be observed [m/ss] |
| `limit.max_acc` | double | maximum acceleration to be observed [m/ss] |
| `limit.min_jerk` | double | minimum jerk to be observed [m/sss] |
| `limit.max_jerk` | double | maximum jerk to be observed [m/sss] |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
