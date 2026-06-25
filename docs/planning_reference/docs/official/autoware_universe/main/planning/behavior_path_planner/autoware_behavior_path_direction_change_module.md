---
source_type: rendered_autoware_docs
site: autoware_universe
title: Direction Change Module
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_direction_change_module/
rendered_html: rendered/autoware_universe_html/index-188.html
generated: 2026-06-24T14:59:33.649961+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Direction Change Module

## Overview

This module enables the behavior path planner to handle paths with cusp points (direction changes) by detecting cusp points and reversing path point orientations (yaw angles) to indicate reverse direction segments. This enables bidirectional path following through orientation-based direction indication.

## Features

- **Cusp Detection**: Automatically detects points in the path where direction changes occur (typically 180-degree turns) using configurable angle thresholds
- **Orientation Reversal**: Reverses path point yaw angles at cusp points to indicate reverse direction segments
- **Direction Change Handling**: Enables the vehicle to follow paths with direction changes (forward and reverse segments)
- **Map-Based Activation**: Activates only when `direction_change` is set to `"yes"` in lanelet map and ego is away from the goal pose.

## Parameters

See `config/direction_change.param.yaml` for detailed parameter descriptions.

## Integration

This module is automatically registered as a plugin in the behavior path planner. To enable it, add it to the module list in the behavior path planner configuration.

## Usage

The module activates automatically when:

- **A `direction_change` attribute is set to `"yes"` in the lanelet map** (required)
- The path is within lanelets containing this tag

## Map Requirements

See the [document](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_direction_change_module/autoware_lanelet2_extension/autoware_lanelet2_extension/docs/lanelet2_format_extension.md)
