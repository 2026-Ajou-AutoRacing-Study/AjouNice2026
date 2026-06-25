---
source_type: rendered_autoware_docs
site: autoware_universe
title: Autoware Hazard Lights Selector
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_hazard_lights_selector/
rendered_html: rendered/autoware_universe_html/index-159.html
generated: 2026-06-24T14:59:31.878648+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Autoware Hazard Lights Selector

## Purpose

The Hazard Lights Selector is a node that manages and selects hazard light commands from multiple sources (planning and system) to determine the final hazard lights state of the vehicle.

## Inner-workings / Algorithms

The node operates on a timer-based update cycle and implements the following logic:

1. Receives hazard light commands from both planning and system sources
2. If either source requests hazard lights to be enabled, the final command will be to enable hazard lights
3. If neither source requests hazard lights to be enabled, the final command will be to disable hazard lights
4. The selected command is published at a configurable update rate

## Inputs / Outputs

### Input

| Name | Type | Description |
| --- | --- | --- |
| `input/planning/hazard_lights_command` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | Hazard lights command from the planning system |
| `input/system/hazard_lights_command` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | Hazard lights command from the system |

### Output

| Name | Type | Description |
| --- | --- | --- |
| `output/hazard_lights_command` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | The selected hazard lights command |

## Parameters

| Name | Type | Default Value | Description |
| --- | --- | --- | --- |
| `update_rate` | int | 10 | The update rate in Hz for publishing hazard lights commands |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
