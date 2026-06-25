---
source_type: rendered_autoware_docs
site: autoware_core
title: Planning Topic Converter
canonical_url: https://autowarefoundation.github.io/autoware_core/main/planning/autoware_planning_topic_converter/
rendered_html: rendered/autoware_core_html/index-61.html
generated: 2026-06-24T15:00:04.893651+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Planning Topic Converter

## Purpose

This package provides tools that convert topic type among types are defined in <https://github.com/autowarefoundation/autoware_msgs>.

## Inner-workings / Algorithms

### Usage example

The tools in this package are provided as composable ROS 2 component nodes, so that they can be spawned into an existing process, launched from launch files, or invoked from the command line.

```xml

```

## Parameters

| Name | Type | Description |
| --- | --- | --- |
| `input_topic` | string | input topic name. |
| `output_topic` | string | output topic name. |

## Assumptions / Known limits

## Future extensions / Unimplemented parts
