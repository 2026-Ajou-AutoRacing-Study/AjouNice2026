---
source_type: rendered_autoware_docs
site: autoware_documentation
title: /api/planning/velocity_factors
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/list/api/planning/velocity_factors/
rendered_html: rendered/autoware_documentation_html/index-165.html
generated: 2026-06-24T14:59:50.014268+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# /api/planning/velocity\_factors

## Status

- Latest Version: not released
- Method: realtime stream
- Data Type: [autoware\_adapi\_v1\_msgs/msg/VelocityFactorArray](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/types/autoware_adapi_v1_msgs/msg/VelocityFactorArray/)

## Description

Get the velocity factors, sorted in ascending order of distance.
For details, see the [planning factors](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/planning-factors/).

## Message

| Name | Type | Description |
| --- | --- | --- |
| factors.pose | geometry\_msgs/msg/Pose | The base link pose related to the velocity factor. |
| factors.distance | float32 | The distance from the base link to the above pose. |
| factors.status | uint16 | The status of the velocity factor. |
| factors.behavior | string | The behavior type of the velocity factor. |
| factors.sequence | string | The sequence type of the velocity factor. |
| factors.detail | string | The additional information of the velocity factor. |
| factors.cooperation | autoware\_adapi\_v1\_msgs/msg/CooperationStatus[<=1] | The cooperation status if the module supports. |
